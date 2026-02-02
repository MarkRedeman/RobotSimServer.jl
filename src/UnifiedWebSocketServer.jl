# UnifiedWebSocketServer - Single-port WebSocket server with path-based routing
#
# Provides a unified WebSocket server that handles both robot control and camera
# streaming on a single port using URL path routing:
#
#   ws://localhost:8080/{robot}/control          - Robot control commands
#   ws://localhost:8080/{robot}/control?leader=X - Control with specific leader type
#   ws://localhost:8080/{robot}/cameras/{name}   - Camera JPEG streams
#
# Per-Client Leader Types:
#   Each control client can specify their own leader type via query parameter.
#   This enables different clients to use different joint naming conventions:
#     - Client A: ws://...?leader=so101  -> receives SO101 joint names
#     - Client B: ws://...?leader=trossen -> receives Trossen joint names
#
# Usage:
#   include("src/UnifiedWebSocketServer.jl")
#
#   server = UnifiedServer(port=8080, robot="lekiwi", fps=30.0)
#   register_camera!(server, "front")
#   register_camera!(server, "wrist")
#   
#   # Set up per-client context factory (optional)
#   set_context_factory!(server, leader_type -> create_teleop_context(...))
#   
#   start!(server, get_state_callback)
#
#   # In control loop:
#   process_commands!(server, data, actuator_map, apply_command!)
#   maybe_broadcast!(server, model, data, get_state)
#
#   # Camera frames are sent via:
#   broadcast_frame!(server, "front", jpeg_bytes)

using HTTP
using HTTP.WebSockets
using JSON

# =============================================================================
# Types
# =============================================================================

"""
    CameraEndpoint

Manages WebSocket clients for a single camera stream.
"""
mutable struct CameraEndpoint
    name::String
    clients::Vector{HTTP.WebSockets.WebSocket}
    clients_lock::ReentrantLock
end

function CameraEndpoint(name::String)
    return CameraEndpoint(name, HTTP.WebSockets.WebSocket[], ReentrantLock())
end

"""
    UnifiedServer

Single-port WebSocket server with path-based routing for robot control and cameras.

Supports per-client leader types via query parameters, allowing different clients
to use different joint naming conventions simultaneously.

# Fields
- `port::Int`: Server port (default 8080)
- `robot::String`: Robot namespace prefix (e.g., "lekiwi", "trossen/wxai")
- `control_channel::Channel{Dict{String, Any}}`: Channel for incoming control commands
- `control_clients::Set{Any}`: Connected control WebSocket clients
- `control_clients_lock::ReentrantLock`: Lock for thread-safe client access
- `cameras::Dict{String, CameraEndpoint}`: Registered camera endpoints
- `cameras_lock::ReentrantLock`: Lock for camera registration
- `last_broadcast_time::Ref{Float64}`: Time of last state broadcast
- `broadcast_interval::Float64`: Minimum time between broadcasts (1/fps)
- `prev_state::Dict{String, Float64}`: Previous state for change detection
- `state_change_threshold::Float64`: Minimum change to trigger broadcast
- `model_ref::Ref{Any}`: Reference to MuJoCo model
- `data_ref::Ref{Any}`: Reference to MuJoCo data
- `server_task::Union{Task, Nothing}`: Background server task
- `running::Bool`: Server running state
- `client_contexts::Dict{Any, Any}`: Per-client teleop contexts (ws → context)
- `client_contexts_lock::ReentrantLock`: Lock for client context access
- `context_factory::Union{Function, Nothing}`: Factory for creating per-client contexts
- `default_leader_type::Any`: Default leader type when not specified in query
"""
mutable struct UnifiedServer
    port::Int
    robot::String
    control_channel::Channel{Dict{String, Any}}
    control_clients::Set{Any}
    control_clients_lock::ReentrantLock
    cameras::Dict{String, CameraEndpoint}
    cameras_lock::ReentrantLock
    last_broadcast_time::Ref{Float64}
    broadcast_interval::Float64
    prev_state::Dict{String, Float64}
    state_change_threshold::Float64
    model_ref::Ref{Any}
    data_ref::Ref{Any}
    server_task::Union{Task, Nothing}
    running::Bool
    # Per-client context support
    client_contexts::Dict{Any, Any}
    client_contexts_lock::ReentrantLock
    context_factory::Union{Function, Nothing}
    default_leader_type::Any
end

"""
    UnifiedServer(; port=8080, robot="robot", fps=30.0, state_change_threshold=0.01)

Create a new unified WebSocket server.

# Arguments
- `port`: Port to listen on (default 8080)
- `robot`: Robot namespace prefix for URL routing (e.g., "lekiwi", "trossen/wxai")
- `fps`: Maximum state broadcast rate in Hz (default 30.0)
- `state_change_threshold`: Minimum joint change in degrees to trigger broadcast
"""
function UnifiedServer(; port::Int = 8080, robot::String = "robot",
        fps::Float64 = 30.0, state_change_threshold::Float64 = 0.01)
    return UnifiedServer(
        port,
        robot,
        Channel{Dict{String, Any}}(10),
        Set{Any}(),
        ReentrantLock(),
        Dict{String, CameraEndpoint}(),
        ReentrantLock(),
        Ref(0.0),
        1.0 / fps,
        Dict{String, Float64}(),
        state_change_threshold,
        Ref{Any}(nothing),
        Ref{Any}(nothing),
        nothing,
        false,
        # Per-client context fields
        Dict{Any, Any}(),
        ReentrantLock(),
        nothing,
        nothing
    )
end

"""
    set_context_factory!(server::UnifiedServer, factory::Function, default_leader_type)

Set a factory function for creating per-client teleop contexts.

When set, each new control client connection will get its own teleop context
based on the `?leader=X` query parameter (or default_leader_type if not specified).

# Arguments
- `server`: UnifiedServer instance
- `factory`: Function `(leader_type::Type) -> TeleoperatorContext`
- `default_leader_type`: Default leader type when query param not specified

# Example
```julia
set_context_factory!(server, 
    leader_type -> create_teleop_context(leader_type, FollowerType, model, data),
    SO101)
```
"""
function set_context_factory!(server::UnifiedServer, factory::Function, default_leader_type)
    server.context_factory = factory
    server.default_leader_type = default_leader_type
end

"""
    get_client_context(server::UnifiedServer, ws) -> Union{Any, Nothing}

Get the teleop context for a specific client, if one exists.
"""
function get_client_context(server::UnifiedServer, ws)
    return @lock server.client_contexts_lock get(server.client_contexts, ws, nothing)
end

# =============================================================================
# Camera Registration
# =============================================================================

"""
    register_camera!(server::UnifiedServer, name::String)

Register a camera endpoint. Creates the path /{robot}/cameras/{name}.
Must be called before start!().
"""
function register_camera!(server::UnifiedServer, name::String)
    @lock server.cameras_lock begin
        if haskey(server.cameras, name)
            @warn "Camera '$name' already registered"
            return
        end
        server.cameras[name] = CameraEndpoint(name)
    end
    println("Registered camera endpoint: /$(server.robot)/cameras/$name")
end

# =============================================================================
# Server Start/Stop
# =============================================================================

"""
    start!(server::UnifiedServer, get_state::Function)

Start the unified WebSocket server with path-based routing.

# Arguments
- `server`: The unified server instance
- `get_state`: Function (model, data) -> Dict{String, Float64} for control state
"""
function start!(server::UnifiedServer, get_state::Function)
    server.running = true
    robot_prefix = "/" * server.robot

    server.server_task = @async begin
        try
            HTTP.listen("0.0.0.0", server.port) do http
                handle_request!(server, http, robot_prefix, get_state)
            end
        catch e
            if server.running
                @warn "UnifiedServer error" exception=(e, catch_backtrace())
            end
        end
    end

    println("\nUnifiedServer listening on port $(server.port)")
    println("  Control: ws://localhost:$(server.port)$(robot_prefix)/control")
    for name in keys(server.cameras)
        println("  Camera '$name': ws://localhost:$(server.port)$(robot_prefix)/cameras/$name")
    end
end

"""
    stop!(server::UnifiedServer)

Stop the unified server and close all connections.
"""
function stop!(server::UnifiedServer)
    server.running = false

    # Close control clients
    @lock server.control_clients_lock begin
        for ws in server.control_clients
            try
                close(ws)
            catch
            end
        end
        empty!(server.control_clients)
    end

    # Close camera clients
    @lock server.cameras_lock begin
        for (_, endpoint) in server.cameras
            @lock endpoint.clients_lock begin
                for ws in endpoint.clients
                    try
                        close(ws)
                    catch
                    end
                end
                empty!(endpoint.clients)
            end
        end
    end

    println("UnifiedServer stopped")
end

# =============================================================================
# Request Routing
# =============================================================================

"""
    handle_request!(server, http, robot_prefix, get_state)

Route incoming HTTP/WebSocket requests based on path.
Query parameters are passed to control client handlers for per-client configuration.
"""
function handle_request!(
        server::UnifiedServer, http, robot_prefix::String, get_state::Function)
    target = http.message.target
    # Parse path (strip query string if present)
    path = split(target, "?")[1]

    # Only handle WebSocket upgrades
    if !HTTP.WebSockets.isupgrade(http.message)
        HTTP.setstatus(http, 404)
        HTTP.startwrite(http)
        write(http, "Not found - WebSocket endpoints only")
        return
    end

    # Must start with robot prefix
    if !startswith(path, robot_prefix * "/")
        HTTP.setstatus(http, 404)
        HTTP.startwrite(http)
        write(http, "Unknown robot: expected prefix '$robot_prefix/'")
        return
    end

    # Extract subpath after robot prefix
    subpath = path[(length(robot_prefix) + 1):end]

    # Route: /{robot}/control or /{robot}/control?leader=X
    if subpath == "/control"
        HTTP.WebSockets.upgrade(http) do ws
            # Pass full target (with query string) for per-client context setup
            handle_control_client!(server, ws, get_state, target)
        end
        return
    end

    # Route: /{robot}/cameras/{name}
    if startswith(subpath, "/cameras/")
        camera_name = subpath[10:end]  # Strip "/cameras/"

        # Check if camera is registered
        endpoint = @lock server.cameras_lock get(server.cameras, camera_name, nothing)

        if endpoint !== nothing
            HTTP.WebSockets.upgrade(http) do ws
                handle_camera_client!(server, endpoint, ws)
            end
            return
        end
    end

    # Unknown path
    HTTP.setstatus(http, 404)
    HTTP.startwrite(http)
    write(http, "Unknown endpoint: $path")
end

# =============================================================================
# Control Endpoint Handlers
# =============================================================================

"""
    handle_control_client!(server, ws, get_state, target)

Handle a control WebSocket client connection.

Creates a per-client teleop context if a context factory is configured,
using the `?leader=X` query parameter to determine the leader type.

# Arguments
- `server`: UnifiedServer instance
- `ws`: WebSocket connection
- `get_state`: Function to get joint state (used as fallback)
- `target`: HTTP request target including query string
"""
function handle_control_client!(
        server::UnifiedServer, ws, get_state::Function, target::String)
    # Create per-client context if factory is configured
    client_ctx = nothing
    leader_type = server.default_leader_type

    if server.context_factory !== nothing && server.default_leader_type !== nothing
        # Parse leader type from query param
        leader_str = parse_query_param(target, "leader")
        if !isempty(leader_str)
            # Try to resolve leader type (requires ROBOT_TYPE_MAP from CLIUtils)
            # The context factory should handle the type lookup
            leader_type = leader_str
        end

        try
            client_ctx = server.context_factory(leader_type)
            @lock server.client_contexts_lock begin
                server.client_contexts[ws] = client_ctx
            end
        catch e
            @warn "Failed to create client context" exception = e
        end
    end

    # Log connection with leader info
    if client_ctx !== nothing
        println("Control client connected (leader=$leader_type)")
    else
        println("Control client connected")
    end

    @lock server.control_clients_lock push!(server.control_clients, ws)

    # Send initial state to new client (using client's context if available)
    if server.model_ref[] !== nothing && server.data_ref[] !== nothing
        send_state_to_client!(server, ws, server.model_ref[], server.data_ref[], get_state)
    end

    try
        for msg in ws
            try
                raw = JSON.parse(String(msg))
                # Include source websocket reference for per-client command processing
                raw["_ws"] = ws
                put!(server.control_channel, raw)

                # Handle ping
                if get(raw, "command", "") == "ping"
                    HTTP.WebSockets.send(
                        ws, JSON.json(Dict(
                            "event" => "pong",
                            "timestamp" => time()
                        )))
                end
            catch e
                @warn "Control message error: $e"
            end
        end
    finally
        @lock server.control_clients_lock delete!(server.control_clients, ws)
        @lock server.client_contexts_lock delete!(server.client_contexts, ws)
        println("Control client disconnected")
    end
end

"""
    send_state_to_client!(server, ws, model, data, get_state)

Send current state to a specific control client.

Uses the client's teleop context if available for per-client state format.
"""
function send_state_to_client!(server::UnifiedServer, ws, model, data, get_state::Function)
    # Get client-specific context if available
    client_ctx = get_client_context(server, ws)

    state = if client_ctx !== nothing && hasproperty(client_ctx, :follower_joint_map)
        # Use per-client context for state formatting
        # This requires get_state_for_leader from TeleoperatorMapping
        # The example scripts will provide this via get_state callback
        get_state(model, data, client_ctx)
    else
        get_state(model, data)
    end

    msg = JSON.json(Dict(
        "event" => "state_was_updated",
        "timestamp" => time(),
        "state" => state,
        "is_controlled" => false
    ))
    try
        HTTP.WebSockets.send(ws, msg)
    catch e
        @warn "Failed to send state to client: $e"
    end
end

"""
    process_commands!(server::UnifiedServer, data, actuator_map::Dict, apply_command!::Function)

Process pending commands from the control channel.

Commands now include a `_ws` field identifying the source client, enabling
per-client command interpretation when a context factory is configured.
"""
function process_commands!(
        server::UnifiedServer, data, actuator_map::Dict, apply_command!::Function)
    while isready(server.control_channel)
        raw = take!(server.control_channel)
        cmd = get(raw, "command", "")

        if cmd == "set_joints_state"
            joints = get(raw, "joints", Dict())
            # Get source client's context for per-client joint mapping
            ws = get(raw, "_ws", nothing)
            client_ctx = ws !== nothing ? get_client_context(server, ws) : nothing

            # Let apply_command! handle the context (if provided)
            if client_ctx !== nothing
                apply_command!(data, actuator_map, joints, client_ctx)
            else
                apply_command!(data, actuator_map, joints)
            end
        end
    end
end

"""
    broadcast_control_state!(server::UnifiedServer, model, data, get_state::Function)

Broadcast state to all connected control clients using per-client formatting.

Each client receives state in their own leader's joint naming convention
if a per-client context is available.
"""
function broadcast_control_state!(
        server::UnifiedServer, model, data, get_state::Function)
    current_time = time()

    @lock server.control_clients_lock begin
        dead_clients = []
        for ws in server.control_clients
            try
                # Get client-specific context for state formatting
                client_ctx = get_client_context(server, ws)

                state = if client_ctx !== nothing
                    # Try calling get_state with context
                    try
                        get_state(model, data, client_ctx)
                    catch
                        # Fallback if get_state doesn't accept context
                        get_state(model, data)
                    end
                else
                    get_state(model, data)
                end

                msg = JSON.json(Dict(
                    "event" => "state_was_updated",
                    "timestamp" => current_time,
                    "state" => state,
                    "is_controlled" => false
                ))
                HTTP.WebSockets.send(ws, msg)
            catch e
                push!(dead_clients, ws)
            end
        end
        for ws in dead_clients
            delete!(server.control_clients, ws)
            @lock server.client_contexts_lock delete!(server.client_contexts, ws)
        end
    end
end

# Legacy method for backward compatibility (single state dict)
function broadcast_control_state!(server::UnifiedServer, state::Dict)
    msg = JSON.json(Dict(
        "event" => "state_was_updated",
        "timestamp" => time(),
        "state" => state,
        "is_controlled" => false
    ))

    @lock server.control_clients_lock begin
        dead_clients = []
        for ws in server.control_clients
            try
                HTTP.WebSockets.send(ws, msg)
            catch e
                push!(dead_clients, ws)
            end
        end
        for ws in dead_clients
            delete!(server.control_clients, ws)
        end
    end
end

"""
    maybe_broadcast!(server::UnifiedServer, model, data, get_state::Function)

Broadcast control state if enough time has passed and state has changed.

Supports per-client state formatting when a context factory is configured.
"""
function maybe_broadcast!(server::UnifiedServer, model, data, get_state::Function)
    # Update refs for new client state sends
    if server.model_ref[] === nothing
        server.model_ref[] = model
        server.data_ref[] = data
    end

    # Throttle broadcasts
    current_time = time()
    if current_time - server.last_broadcast_time[] < server.broadcast_interval
        return
    end

    # Check if any clients have per-client contexts
    has_per_client_contexts = !isempty(server.client_contexts)

    if has_per_client_contexts
        # Use per-client broadcasting (each client gets their own state format)
        # First check if state has changed using default format
        state = get_state(model, data)
        if !state_changed(state, server.prev_state, server.state_change_threshold)
            return
        end
        merge!(server.prev_state, state)

        # Broadcast with per-client formatting
        broadcast_control_state!(server, model, data, get_state)
    else
        # Use simple broadcasting (all clients get same state)
        state = get_state(model, data)
        if !state_changed(state, server.prev_state, server.state_change_threshold)
            return
        end
        merge!(server.prev_state, state)
        broadcast_control_state!(server, state)
    end

    server.last_broadcast_time[] = current_time
end

"""
    state_changed(current, previous, threshold)

Check if state has changed beyond threshold.
"""
function state_changed(current::Dict, previous::Dict, threshold::Float64)
    isempty(previous) && return true
    for (name, val) in current
        prev_val = get(previous, name, val + threshold * 2)
        if abs(val - prev_val) > threshold
            return true
        end
    end
    return false
end

# =============================================================================
# Camera Endpoint Handlers
# =============================================================================

"""
    handle_camera_client!(server, endpoint, ws)

Handle a camera WebSocket client connection.
"""
function handle_camera_client!(server::UnifiedServer, endpoint::CameraEndpoint, ws)
    println("Camera '$(endpoint.name)' client connected")

    @lock endpoint.clients_lock push!(endpoint.clients, ws)

    try
        # Keep connection alive - we don't expect messages from camera clients
        for msg in ws
            # Consume but ignore
        end
    catch e
        if !(e isa HTTP.WebSockets.WebSocketError || e isa EOFError)
            @warn "Camera client error" exception=e
        end
    finally
        @lock endpoint.clients_lock filter!(c -> c !== ws, endpoint.clients)
        println("Camera '$(endpoint.name)' client disconnected")
    end
end

"""
    broadcast_frame!(server::UnifiedServer, camera_name::String, jpeg_bytes::Vector{UInt8})

Broadcast a JPEG frame to all clients connected to the specified camera.
"""
function broadcast_frame!(
        server::UnifiedServer, camera_name::String, jpeg_bytes::Vector{UInt8})
    endpoint = @lock server.cameras_lock get(server.cameras, camera_name, nothing)

    if endpoint === nothing
        @warn "Unknown camera: $camera_name"
        return
    end

    clients = @lock endpoint.clients_lock copy(endpoint.clients)

    if isempty(clients)
        return  # No clients, skip
    end

    failed_clients = HTTP.WebSockets.WebSocket[]
    for client in clients
        try
            HTTP.WebSockets.send(client, jpeg_bytes)
        catch
            push!(failed_clients, client)
        end
    end

    # Remove failed clients
    if !isempty(failed_clients)
        @lock endpoint.clients_lock begin
            filter!(c -> !(c in failed_clients), endpoint.clients)
        end
    end
end

"""
    get_camera_client_count(server::UnifiedServer, camera_name::String) -> Int

Get the number of connected clients for a camera (useful for skipping encoding).
"""
function get_camera_client_count(server::UnifiedServer, camera_name::String)
    endpoint = @lock server.cameras_lock get(server.cameras, camera_name, nothing)
    if endpoint === nothing
        return 0
    end
    return @lock endpoint.clients_lock length(endpoint.clients)
end
