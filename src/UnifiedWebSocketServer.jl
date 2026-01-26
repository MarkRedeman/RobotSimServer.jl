# UnifiedWebSocketServer - Single-port WebSocket server with path-based routing
#
# Provides a unified WebSocket server that handles both robot control and camera
# streaming on a single port using URL path routing:
#
#   ws://localhost:8080/{robot}/control          - Robot control commands
#   ws://localhost:8080/{robot}/cameras/{name}   - Camera JPEG streams
#
# Usage:
#   include("src/UnifiedWebSocketServer.jl")
#
#   server = UnifiedServer(port=8080, robot="lekiwi", fps=30.0)
#   register_camera!(server, "front")
#   register_camera!(server, "wrist")
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
        false
    )
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

    # Route: /{robot}/control
    if subpath == "/control"
        HTTP.WebSockets.upgrade(http) do ws
            handle_control_client!(server, ws, get_state)
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
    handle_control_client!(server, ws, get_state)

Handle a control WebSocket client connection.
"""
function handle_control_client!(server::UnifiedServer, ws, get_state::Function)
    println("Control client connected")
    @lock server.control_clients_lock push!(server.control_clients, ws)

    # Send initial state to new client
    if server.model_ref[] !== nothing && server.data_ref[] !== nothing
        send_state_to_client!(server, ws, server.model_ref[], server.data_ref[], get_state)
    end

    try
        for msg in ws
            try
                raw = JSON.parse(String(msg))
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
        println("Control client disconnected")
    end
end

"""
    send_state_to_client!(server, ws, model, data, get_state)

Send current state to a specific control client.
"""
function send_state_to_client!(server::UnifiedServer, ws, model, data, get_state::Function)
    state = get_state(model, data)
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
"""
function process_commands!(
        server::UnifiedServer, data, actuator_map::Dict, apply_command!::Function)
    while isready(server.control_channel)
        raw = take!(server.control_channel)
        cmd = get(raw, "command", "")

        if cmd == "set_joints_state"
            joints = get(raw, "joints", Dict())
            apply_command!(data, actuator_map, joints)
        end
    end
end

"""
    broadcast_control_state!(server::UnifiedServer, state::Dict)

Broadcast state to all connected control clients.
"""
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

    state = get_state(model, data)

    # Only broadcast if state has changed
    if !state_changed(state, server.prev_state, server.state_change_threshold)
        return
    end

    merge!(server.prev_state, state)
    broadcast_control_state!(server, state)
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
