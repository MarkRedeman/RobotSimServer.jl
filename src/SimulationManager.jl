# SimulationManager.jl - Multi-robot simulation orchestration
#
# Manages multiple concurrent robot simulations with a unified HTTP/WebSocket interface.
# Handles lazy startup, graceful shutdown, and request routing.
#
# Architecture:
#   ┌─────────────────────────────────────────────────────────────────────────┐
#   │                     SimulationManager                                    │
#   │                                                                          │
#   │  HTTP/WebSocket Router (single port)                                    │
#   │    ws://host/{robot}/control?leader=X  → WebSocket control              │
#   │    ws://host/{robot}/cameras/{cam}     → WebSocket camera stream        │
#   │    GET http://host/{robot}/urdf        → HTTP URDF file                 │
#   │    GET http://host/{robot}/meshes/*    → HTTP mesh files                │
#   │                                                                          │
#   │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                   │
#   │  │ SO101 Sim    │  │ LeKiwi Sim   │  │ Trossen Sim  │  ...              │
#   │  │ (headless)   │  │ (headless)   │  │ (headless)   │                   │
#   │  │ SimInstance  │  │ SimInstance  │  │ SimInstance  │                   │
#   │  └──────────────┘  └──────────────┘  └──────────────┘                   │
#   │                                                                          │
#   │  Lifecycle: start on first client → stop 30s after last disconnect     │
#   └─────────────────────────────────────────────────────────────────────────┘
#
# Usage:
#   include("src/SimulationManager.jl")
#
#   manager = SimulationManager(8080, project_root)
#   start!(manager)
#   # Server runs until Ctrl+C
#   stop!(manager)

using HTTP
using HTTP.WebSockets
using JSON

# Assumes these are already loaded:
# include("HeadlessRenderer.jl")
# include("RobotConfigs.jl")
# include("SimulationInstance.jl")
# include("AssetServer.jl")
# include("URDFGenerator.jl")

# =============================================================================
# Types
# =============================================================================

"""
    SimulationManager

Orchestrates multiple concurrent robot simulations.

# Responsibilities
- Listens on a single port for all robots
- Routes requests to the appropriate SimulationInstance
- Creates instances on-demand (lazy startup)
- Monitors instances for shutdown eligibility
- Serves static assets (URDF, meshes)

# Fields
- `port::Int`: Port to listen on
- `project_root::String`: Path to project root directory
- `instances::Dict{String, SimulationInstance}`: Active simulation instances
- `instances_lock::ReentrantLock`: Lock for instances dictionary
- `server::Union{HTTP.Server, Nothing}`: HTTP server handle
- `running::Bool`: Whether the manager is running
- `shutdown_check_interval::Float64`: Seconds between shutdown eligibility checks
- `urdf_cache_dir::String`: Directory for caching generated URDFs
"""
mutable struct SimulationManager
    port::Int
    project_root::String
    instances::Dict{String, SimulationInstance}
    instances_lock::ReentrantLock
    server::Union{HTTP.Server, Nothing}
    server_task::Union{Task, Nothing}
    cleanup_task::Union{Task, Nothing}
    running::Bool
    shutdown_check_interval::Float64
    urdf_cache_dir::String
end

"""
    SimulationManager(port::Int, project_root::String;
                      shutdown_check_interval::Float64=5.0) -> SimulationManager

Create a new simulation manager.

# Arguments
- `port`: Port to listen on (e.g., 8080)
- `project_root`: Path to project root directory
- `shutdown_check_interval`: Seconds between shutdown checks (default: 5.0)

# Returns
- `SimulationManager`: Ready to start
"""
function SimulationManager(port::Int, project_root::String;
        shutdown_check_interval::Float64 = 5.0)
    urdf_cache_dir = joinpath(project_root, ".cache", "urdf")

    return SimulationManager(
        port,
        project_root,
        Dict{String, SimulationInstance}(),
        ReentrantLock(),
        nothing,
        nothing,
        nothing,
        false,
        shutdown_check_interval,
        urdf_cache_dir
    )
end

# =============================================================================
# Lifecycle Management
# =============================================================================

"""
    start!(manager::SimulationManager)

Start the simulation manager.

Begins listening for HTTP/WebSocket connections and starts the cleanup task.
"""
function start!(manager::SimulationManager)
    if manager.running
        @warn "SimulationManager already running"
        return
    end

    manager.running = true

    # Create URDF cache directory
    if !isdir(manager.urdf_cache_dir)
        mkpath(manager.urdf_cache_dir)
    end

    # Print available robots
    available_robots = list_available_robots()
    println("SimulationManager starting on port $(manager.port)")
    println("Available robots: $(join(available_robots, ", "))")
    println("\nEndpoints:")
    for robot_id in available_robots
        println("  ws://localhost:$(manager.port)/$(robot_id)/control")
        println("  ws://localhost:$(manager.port)/$(robot_id)/cameras/{camera_name}")
        println("  GET http://localhost:$(manager.port)/$(robot_id)/urdf")
        println("  GET http://localhost:$(manager.port)/$(robot_id)/meshes/{path}")
    end
    println()

    # Start cleanup task
    manager.cleanup_task = @async cleanup_loop!(manager)

    # Start HTTP server
    manager.server_task = @async begin
        try
            HTTP.serve!(manager.port; stream = true) do http
                handle_request!(manager, http)
            end
        catch e
            if manager.running
                @error "Server error" exception = (e, catch_backtrace())
            end
        end
    end

    println("SimulationManager started")
end

"""
    stop!(manager::SimulationManager)

Stop the simulation manager and all active simulations.
"""
function stop!(manager::SimulationManager)
    if !manager.running
        return
    end

    println("SimulationManager stopping...")
    manager.running = false

    # Stop all instances
    @lock manager.instances_lock begin
        for (robot_id, instance) in manager.instances
            println("  Stopping $robot_id...")
            stop!(instance)
        end
        empty!(manager.instances)
    end

    # Wait for tasks to finish
    if manager.cleanup_task !== nothing
        try
            wait(manager.cleanup_task)
        catch
        end
    end

    # Close server (this is a bit tricky with HTTP.jl)
    # The server task will end when we stop accepting connections

    println("SimulationManager stopped")
end

"""
    cleanup_loop!(manager::SimulationManager)

Periodically check for and stop idle simulation instances.
"""
function cleanup_loop!(manager::SimulationManager)
    println("Cleanup loop started (interval: $(manager.shutdown_check_interval)s)")

    while manager.running
        try
            sleep(manager.shutdown_check_interval)

            # Check each instance for shutdown eligibility
            instances_to_stop = String[]

            @lock manager.instances_lock begin
                for (robot_id, instance) in manager.instances
                    if should_shutdown(instance)
                        push!(instances_to_stop, robot_id)
                    end
                end
            end

            # Stop eligible instances
            for robot_id in instances_to_stop
                println("Shutting down idle instance: $robot_id")
                @lock manager.instances_lock begin
                    if haskey(manager.instances, robot_id)
                        instance = manager.instances[robot_id]
                        stop!(instance)
                        delete!(manager.instances, robot_id)
                    end
                end
            end
        catch e
            if manager.running
                @warn "Cleanup loop error" exception = (e, catch_backtrace())
            end
        end
    end

    println("Cleanup loop ended")
end

# =============================================================================
# Instance Management
# =============================================================================

"""
    get_or_create_instance!(manager::SimulationManager, robot_id::AbstractString) -> SimulationInstance

Get an existing simulation instance or create a new one.

# Arguments
- `manager`: SimulationManager
- `robot_id`: Robot identifier (e.g., "so101", "fanuc/m10ia")

# Returns
- `SimulationInstance`: Running simulation instance

# Throws
- `ArgumentError`: If robot_id is not recognized
"""
function get_or_create_instance!(manager::SimulationManager, robot_id::AbstractString)
    @lock manager.instances_lock begin
        if haskey(manager.instances, robot_id)
            return manager.instances[robot_id]
        end
    end

    # Create new instance outside the lock (creation can be slow)
    println("Creating new SimulationInstance for $robot_id...")

    config = get_robot_config(robot_id, manager.project_root)
    instance = SimulationInstance(config, manager.project_root)
    start!(instance)

    @lock manager.instances_lock begin
        # Check again in case another task created it while we were building
        if haskey(manager.instances, robot_id)
            # Another task beat us - stop our instance and use theirs
            stop!(instance)
            return manager.instances[robot_id]
        end
        manager.instances[robot_id] = instance
    end

    return instance
end

"""
    get_instance(manager::SimulationManager, robot_id::AbstractString) -> Union{SimulationInstance, Nothing}

Get an existing simulation instance without creating a new one.
"""
function get_instance(manager::SimulationManager, robot_id::AbstractString)
    @lock manager.instances_lock begin
        return get(manager.instances, robot_id, nothing)
    end
end

# =============================================================================
# Request Routing
# =============================================================================

"""
    handle_request!(manager::SimulationManager, http)

Route incoming HTTP/WebSocket requests to the appropriate handler.

# URL Patterns
- `/{robot}/control?leader=X` - WebSocket control endpoint
- `/{robot}/cameras/{camera}` - WebSocket camera stream
- `/{robot}/urdf` - HTTP URDF file
- `/{robot}/meshes/{path}` - HTTP mesh files
- `/` - Root info page
- `/robots` - JSON list of available robots
"""
function handle_request!(manager::SimulationManager, http)
    try
        # Handle OPTIONS preflight for CORS
        if handle_options_preflight!(http)
            return
        end

        # Parse the request path
        target = http.message.target
        path = HTTP.URIs.URI(target).path
        segments = filter(!isempty, split(path, '/'))

        if isempty(segments)
            # Root path - serve info page
            handle_root!(manager, http)
            return
        end

        first_segment = segments[1]

        # Check for global endpoints
        if first_segment == "robots"
            handle_robots_list!(manager, http)
            return
        end

        if first_segment == "health"
            handle_health!(manager, http)
            return
        end

        # Robot-specific endpoints: /{robot_id}/...
        robot_id = first_segment

        # Handle fanuc variants: fanuc/m10ia becomes the robot_id
        if first_segment == "fanuc" && length(segments) >= 2
            # Check if second segment is a fanuc variant or an endpoint
            second = segments[2]
            if second ∉ ["control", "cameras", "urdf", "meshes"]
                robot_id = "fanuc/$second"
                segments = vcat([robot_id], segments[3:end])
            end
        end

        # Similarly for trossen variants
        if first_segment == "trossen" && length(segments) >= 2
            second = segments[2]
            if second ∉ ["control", "cameras", "urdf", "meshes"]
                robot_id = "trossen/$second"
                segments = vcat([robot_id], segments[3:end])
            end
        end

        if length(segments) < 2
            # Just robot ID, redirect to control
            serve_404!(http, "Missing endpoint. Try: /$robot_id/control")
            return
        end

        endpoint = length(segments) >= 2 ? segments[2] : ""

        # Validate robot ID
        if !is_valid_robot_id(robot_id)
            serve_404!(http, "Unknown robot: $robot_id. Available: $(join(list_available_robots(), ", "))")
            return
        end

        # Route based on endpoint
        if endpoint == "control"
            handle_control_websocket!(manager, http, robot_id)
        elseif endpoint == "cameras"
            camera_name = length(segments) >= 3 ? segments[3] : ""
            if isempty(camera_name)
                serve_404!(http, "Missing camera name. Try: /$robot_id/cameras/front")
                return
            end
            handle_camera_websocket!(manager, http, robot_id, camera_name)
        elseif endpoint == "urdf"
            handle_urdf!(manager, http, robot_id)
        elseif endpoint == "meshes" || endpoint == "assets"
            # Support both /meshes/ and /assets/ paths for URDF compatibility
            # For /assets/ endpoint, the subpath doesn't include "assets/" but files are in assets/ dir
            asset_path = length(segments) >= 3 ? join(segments[3:end], "/") : ""
            if endpoint == "assets"
                asset_path = "assets/" * asset_path
            end
            handle_mesh!(manager, http, robot_id, asset_path)
        else
            # Check if this looks like an asset file request (has a file extension)
            # This handles cases where URDF references paths like "meshes/foo.stl" directly
            full_path = join(segments[2:end], "/")
            if occursin(r"\.(stl|obj|dae|png|jpg|jpeg|xml)$"i, full_path)
                handle_mesh!(manager, http, robot_id, full_path)
            else
                serve_404!(http, "Unknown endpoint: $endpoint")
            end
        end
    catch e
        @error "Request handling error" exception = (e, catch_backtrace())
        try
            serve_500!(http, "Internal server error")
        catch
        end
    end
end

"""
    is_valid_robot_id(robot_id::AbstractString) -> Bool

Check if a robot ID is valid (known configuration exists).
"""
function is_valid_robot_id(robot_id::AbstractString)
    try
        # Try to get config - will throw if invalid
        # But don't actually create it, just check
        if haskey(ROBOT_CONFIGS, robot_id)
            return true
        end
        # Check for fanuc variants
        if startswith(robot_id, "fanuc/")
            return true  # We accept any fanuc variant
        end
        return false
    catch
        return false
    end
end

# =============================================================================
# HTTP Handlers
# =============================================================================

"""
    handle_root!(manager::SimulationManager, http)

Serve the root info page.
"""
function handle_root!(manager::SimulationManager, http)
    available = list_available_robots()

    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Unified Robot Simulation Server</title>
        <style>
            body { font-family: system-ui, -apple-system, sans-serif; margin: 40px; }
            h1 { color: #333; }
            h2 { color: #666; margin-top: 30px; }
            .robot { margin: 10px 0; padding: 10px; background: #f5f5f5; border-radius: 5px; }
            .robot-name { font-weight: bold; color: #2563eb; }
            .endpoint { margin-left: 20px; font-family: monospace; font-size: 13px; color: #555; }
            .active { color: #16a34a; font-weight: bold; }
        </style>
    </head>
    <body>
        <h1>🤖 Unified Robot Simulation Server</h1>
        <p>Multi-robot MuJoCo simulation server with WebSocket control.</p>
        
        <h2>Available Robots</h2>
        $(join(["<div class='robot'><span class='robot-name'>$r</span>" *
                "<div class='endpoint'>ws://localhost:$(manager.port)/$r/control</div>" *
                "<div class='endpoint'>ws://localhost:$(manager.port)/$r/cameras/{name}</div>" *
                "<div class='endpoint'>GET http://localhost:$(manager.port)/$r/urdf</div>" *
                "</div>" for r in available], "\n"))
        
        <h2>Active Simulations</h2>
        $(let
            active = @lock manager.instances_lock collect(keys(manager.instances))
            if isempty(active)
                "<p>No active simulations. Connect to a robot to start one.</p>"
            else
                join(["<div class='robot'><span class='active'>● $r</span></div>" for r in active], "\n")
            end
        end)
        
        <h2>API Endpoints</h2>
        <ul>
            <li><code>GET /robots</code> - List available robots (JSON)</li>
            <li><code>GET /health</code> - Health check</li>
            <li><code>GET /{robot}/urdf</code> - Get robot URDF</li>
            <li><code>GET /{robot}/meshes/{path}</code> - Get mesh file</li>
            <li><code>WS /{robot}/control?leader=X</code> - Control WebSocket</li>
            <li><code>WS /{robot}/cameras/{name}</code> - Camera stream</li>
        </ul>
    </body>
    </html>
    """

    serve_content!(http, html, "text/html")
end

"""
    handle_robots_list!(manager::SimulationManager, http)

Return JSON list of available robots.
"""
function handle_robots_list!(manager::SimulationManager, http)
    available = list_available_robots()
    active = @lock manager.instances_lock collect(keys(manager.instances))

    response = Dict(
        "available" => available,
        "active" => active
    )

    serve_content!(http, JSON.json(response), "application/json")
end

"""
    handle_health!(manager::SimulationManager, http)

Health check endpoint.
"""
function handle_health!(manager::SimulationManager, http)
    active_count = @lock manager.instances_lock length(manager.instances)

    response = Dict(
        "status" => "ok",
        "running" => manager.running,
        "active_simulations" => active_count
    )

    serve_content!(http, JSON.json(response), "application/json")
end

"""
    handle_urdf!(manager::SimulationManager, http, robot_id::AbstractString)

Serve URDF file for a robot (generating from MJCF if needed).
"""
function handle_urdf!(manager::SimulationManager, http, robot_id::AbstractString)
    try
        config = get_robot_config(robot_id, manager.project_root)

        # Check if robot has a pre-existing URDF
        urdf_path = get_full_urdf_path(config, manager.project_root)

        if urdf_path !== nothing && isfile(urdf_path)
            # Serve existing URDF
            serve_urdf!(http, urdf_path, robot_id)
        else
            # Generate URDF from MJCF
            mjcf_path = get_full_mjcf_path(config, manager.project_root)
            urdf_content = get_or_generate_urdf(
                mjcf_path, manager.urdf_cache_dir, manager.project_root)
            serve_content!(http, urdf_content, "application/xml")
        end
    catch e
        @error "URDF serving error" robot_id exception = (e, catch_backtrace())
        serve_500!(http, "Failed to serve URDF for $robot_id: $(sprint(showerror, e))")
    end
end

"""
    handle_mesh!(manager::SimulationManager, http, robot_id::AbstractString, mesh_path::AbstractString)

Serve mesh file for a robot.

Supports two path resolution strategies:
1. Paths starting with known project directories (robots/, examples/) are served from project root
2. Other paths are served relative to the robot's assets_dir
"""
function handle_mesh!(manager::SimulationManager, http, robot_id::AbstractString, mesh_path::AbstractString)
    try
        # Check if this is a project-root-relative path (used by generated URDFs)
        # These paths start with known project directories
        project_root_prefixes = ["robots/", "examples/", "src/"]
        is_project_relative = any(startswith(mesh_path, prefix)
        for prefix in project_root_prefixes)

        if is_project_relative
            # Serve directly from project root
            serve_mesh!(http, manager.project_root, mesh_path)
        else
            # Serve relative to robot's assets directory
            config = get_robot_config(robot_id, manager.project_root)
            assets_dir = joinpath(manager.project_root, config.assets_dir)
            serve_mesh!(http, assets_dir, mesh_path)
        end
    catch e
        if e isa ArgumentError
            serve_404!(http, "Unknown robot: $robot_id")
        else
            @error "Mesh serving error" robot_id mesh_path exception = (
                e, catch_backtrace())
            serve_500!(http, "Failed to serve mesh: $(sprint(showerror, e))")
        end
    end
end

# =============================================================================
# WebSocket Handlers
# =============================================================================

"""
    handle_control_websocket!(manager::SimulationManager, http, robot_id::AbstractString)

Handle WebSocket control connection for a robot.
"""
function handle_control_websocket!(manager::SimulationManager, http, robot_id::AbstractString)
    # Parse leader type from query string
    uri = HTTP.URIs.URI(http.message.target)
    query_params = HTTP.URIs.queryparams(uri)
    leader_type = get(query_params, "leader", "")

    # Get or create the simulation instance
    instance = try
        get_or_create_instance!(manager, robot_id)
    catch e
        @error "Failed to create instance" robot_id exception = (e, catch_backtrace())
        serve_500!(http, "Failed to start simulation for $robot_id")
        return
    end

    # Upgrade to WebSocket
    try
        WebSockets.upgrade(http) do ws
            handle_control_connection!(instance, ws, leader_type)
        end
    catch e
        @error "WebSocket upgrade failed" robot_id exception = (e, catch_backtrace())
    end
end

"""
    handle_control_connection!(instance::SimulationInstance, ws::WebSocket, leader_type::AbstractString)

Handle a control WebSocket connection lifecycle.
"""
function handle_control_connection!(
        instance::SimulationInstance, ws::HTTP.WebSockets.WebSocket,
        leader_type::AbstractString)
    # Add client
    add_control_client!(instance, ws, leader_type)

    try
        # Message loop
        while !HTTP.WebSockets.isclosed(ws) && instance.running
            msg = try
                HTTP.WebSockets.receive(ws)
            catch e
                if e isa HTTP.WebSockets.WebSocketError || e isa EOFError
                    break
                end
                rethrow()
            end

            # Parse and process command
            try
                data = JSON.parse(String(msg))
                cmd = get(data, "command", "")

                if cmd == "ping"
                    HTTP.WebSockets.send(ws, JSON.json(Dict(
                        "event" => "pong",
                        "timestamp" => time()
                    )))
                elseif cmd == "set_joints_state"
                    # Add WebSocket reference for per-client mapping
                    data["_ws"] = ws
                    submit_command!(instance, data)
                end
            catch e
                @warn "Failed to parse control message" exception = e
            end
        end
    finally
        remove_control_client!(instance, ws)
    end
end

"""
    handle_camera_websocket!(manager::SimulationManager, http, robot_id::AbstractString, camera_name::AbstractString)

Handle WebSocket camera stream connection.
"""
function handle_camera_websocket!(
        manager::SimulationManager, http, robot_id::AbstractString,
        camera_name::AbstractString)
    # Get existing instance (don't create just for camera)
    instance = get_instance(manager, robot_id)

    if instance === nothing
        # Try to create instance for camera connection
        instance = try
            get_or_create_instance!(manager, robot_id)
        catch e
            @error "Failed to create instance" robot_id exception = (e, catch_backtrace())
            serve_500!(http, "Simulation not running for $robot_id. Connect to control first.")
            return
        end
    end

    # Check if camera exists
    if !haskey(instance.cameras, camera_name)
        available_cams = collect(keys(instance.cameras))
        serve_404!(http, "Unknown camera: $camera_name. Available: $(join(available_cams, ", "))")
        return
    end

    # Upgrade to WebSocket
    try
        WebSockets.upgrade(http) do ws
            handle_camera_connection!(instance, ws, camera_name)
        end
    catch e
        @error "WebSocket upgrade failed" robot_id camera_name exception = (
            e, catch_backtrace())
    end
end

"""
    handle_camera_connection!(instance::SimulationInstance, ws::WebSocket, camera_name::AbstractString)

Handle a camera WebSocket connection lifecycle.
"""
function handle_camera_connection!(
        instance::SimulationInstance, ws::HTTP.WebSockets.WebSocket,
        camera_name::AbstractString)
    # Add camera client
    add_camera_client!(instance, camera_name, ws)

    try
        # Keep connection open until closed
        while !HTTP.WebSockets.isclosed(ws) && instance.running
            try
                # Just wait for close, camera frames are pushed from broadcast_loop
                msg = HTTP.WebSockets.receive(ws)
                # Ignore any incoming messages (camera is output-only)
            catch e
                if e isa HTTP.WebSockets.WebSocketError || e isa EOFError
                    break
                end
                # For other errors, just continue
            end
        end
    finally
        remove_camera_client!(instance, camera_name, ws)
    end
end

# =============================================================================
# Utility Functions
# =============================================================================

"""
    get_active_robots(manager::SimulationManager) -> Vector{String}

Get list of currently active robot simulations.
"""
function get_active_robots(manager::SimulationManager)
    @lock manager.instances_lock begin
        return collect(keys(manager.instances))
    end
end

"""
    get_status(manager::SimulationManager) -> Dict

Get manager status as a dictionary.
"""
function get_status(manager::SimulationManager)
    active_robots = @lock manager.instances_lock begin
        Dict(
            robot_id => Dict(
                "running" => instance.running,
                "control_clients" => get_control_client_count(instance),
                "total_clients" => get_client_count(instance)
            )
        for (robot_id, instance) in manager.instances
        )
    end

    return Dict(
        "running" => manager.running,
        "port" => manager.port,
        "available_robots" => list_available_robots(),
        "active_robots" => active_robots
    )
end
