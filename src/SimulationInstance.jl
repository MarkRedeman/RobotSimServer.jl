# SimulationInstance.jl - Single robot simulation encapsulation
#
# Encapsulates a complete MuJoCo robot simulation including:
# - Model and data
# - Headless rendering for camera streams
# - WebSocket client management
# - Physics loop
# - Lifecycle management (auto-shutdown)
#
# Usage:
#   include("src/SimulationInstance.jl")
#
#   config = get_robot_config("so101", project_root)
#   instance = SimulationInstance(config, project_root)
#   start!(instance)
#   # ... clients connect/disconnect ...
#   stop!(instance)

using MuJoCo
using MuJoCo.LibMuJoCo
using HTTP
using HTTP.WebSockets
using JSON
using Images
using FileIO

# Assumes these are already loaded:
# include("HeadlessRenderer.jl")
# include("RobotConfigs.jl")
# include("SceneBuilder.jl")
# include("RobotTypes.jl")
# include("TeleoperatorMapping.jl")
# include("CLIUtils.jl")

# =============================================================================
# Types
# =============================================================================

"""
    CameraState

State for a single camera in the simulation.
"""
mutable struct CameraState
    name::String
    camera::MuJoCo.VisualiserCamera
    clients::Set{HTTP.WebSockets.WebSocket}
    clients_lock::ReentrantLock
    last_frame_time::Float64
end

"""
    SimulationInstance

Encapsulates a complete robot simulation with headless rendering and WebSocket support.

# Lifecycle
1. Created on first client connection to the robot's control endpoint
2. Physics loop runs in background task
3. Cameras render and stream to connected clients
4. Shuts down after grace period when all clients disconnect

# Fields
- `config::RobotConfig`: Robot configuration
- `model::MuJoCo.Model`: MuJoCo model
- `data::MuJoCo.Data`: MuJoCo simulation data
- `renderer::HeadlessRenderer`: Offscreen renderer for cameras
- `cameras::Dict{String, CameraState}`: Camera states by name
- `control_clients::Set{WebSocket}`: Connected control clients
- `control_clients_lock::ReentrantLock`: Lock for control_clients
- `control_channel::Channel`: Channel for incoming commands
- `client_contexts::Dict{WebSocket, Any}`: Per-client teleop contexts
- `client_contexts_lock::ReentrantLock`: Lock for client_contexts
- `default_teleop_ctx::Any`: Default teleop context
- `actuator_map::Dict{String, Int}`: Actuator name to index mapping
- `running::Bool`: Whether simulation is running
- `physics_task::Union{Task, Nothing}`: Background physics task
- `last_client_disconnect::Float64`: Time of last client disconnect
- `shutdown_grace_period::Float64`: Seconds to wait before auto-shutdown
"""
mutable struct SimulationInstance
    config::RobotConfig
    project_root::String
    model::MuJoCo.Model
    data::MuJoCo.Data
    renderer::HeadlessRenderer
    cameras::Dict{String, CameraState}
    control_clients::Set{HTTP.WebSockets.WebSocket}
    control_clients_lock::ReentrantLock
    control_channel::Channel{Dict{String, Any}}
    client_contexts::Dict{Any, Any}
    client_contexts_lock::ReentrantLock
    default_teleop_ctx::Any
    actuator_map::Dict{String, Int}
    running::Bool
    physics_task::Union{Task, Nothing}
    broadcast_task::Union{Task, Nothing}
    last_client_disconnect::Float64
    shutdown_grace_period::Float64
    fps::Float64
    last_broadcast_time::Float64
    prev_state::Dict{String, Float64}
end

# =============================================================================
# Construction
# =============================================================================

"""
    SimulationInstance(config::RobotConfig, project_root::String;
                       shutdown_grace_period::Float64=30.0,
                       fps::Float64=30.0) -> SimulationInstance

Create a new simulation instance for the given robot configuration.

Loads the MuJoCo model, initializes the headless renderer, sets up cameras,
and prepares for client connections. Does NOT start the physics loop -
call `start!(instance)` for that.

# Arguments
- `config`: Robot configuration from RobotConfigs
- `project_root`: Path to project root directory
- `shutdown_grace_period`: Seconds to wait after last client before shutdown (default: 30.0)
- `fps`: Target broadcast/capture rate in Hz (default: 30.0)

# Returns
- `SimulationInstance`: Ready to start simulation
"""
function SimulationInstance(config::RobotConfig, project_root::String;
        shutdown_grace_period::Float64 = 30.0,
        fps::Float64 = 30.0)
    println("Creating SimulationInstance for $(config.robot_id)...")

    # Build the scene
    mjcf_path = joinpath(project_root, config.mjcf_path)
    println("  Loading model: $mjcf_path")

    # Generate cubes
    cubes = generate_cubes(5;
        radius_min = config.cube_config.radius_min,
        radius_max = config.cube_config.radius_max,
        size = config.cube_config.size,
        z = config.cube_config.z)

    # Convert body cameras to BodyCamera objects
    body_cameras = [BodyCamera(
                        name = bc.name,
                        body = bc.body,
                        pos = bc.pos,
                        quat = bc.quat,
                        fovy = bc.fovy
                    ) for bc in config.body_cameras]

    # Get gripper collisions if needed
    collisions = config.gripper_collisions ? default_gripper_collisions() :
                 CollisionPrimitive[]

    # Build scene
    model,
    data = build_scene(mjcf_path, cubes;
        cameras = body_cameras,
        collisions = collisions)

    println("  Scene built: $(model.nbody) bodies, $(model.njnt) joints")

    # Create actuator map
    actuator_names = [unsafe_string(mj_id2name(model, Int32(LibMuJoCo.mjOBJ_ACTUATOR), i -
                                                                                       1))
                      for i in 1:(model.nu)]
    actuator_map = Dict(name => i for (i, name) in enumerate(actuator_names))
    println("  Actuators: $(join(actuator_names, ", "))")

    # Create headless renderer
    renderer = HeadlessRenderer(model, 640, 480)

    # Set up cameras
    cameras = Dict{String, CameraState}()
    for cam_spec in config.cameras
        if cam_spec.mode == :fixed
            # Fixed camera from model
            mj_cam = create_fixed_camera(model, cam_spec.model_camera)
        else
            # Free camera
            mj_cam = create_free_camera(
                lookat = cam_spec.lookat,
                distance = cam_spec.distance,
                azimuth = cam_spec.azimuth,
                elevation = cam_spec.elevation
            )
        end

        cameras[cam_spec.name] = CameraState(
            cam_spec.name,
            mj_cam,
            Set{HTTP.WebSockets.WebSocket}(),
            ReentrantLock(),
            0.0
        )
    end
    println("  Cameras: $(join(keys(cameras), ", "))")

    # Create default teleop context
    default_teleop_ctx = create_teleop_context(
        config.default_leader_type,
        config.follower_type,
        model, data;
        project_root = project_root)
    println("  Teleop: $(config.default_leader_type) → $(config.follower_type)")

    instance = SimulationInstance(
        config,
        project_root,
        model,
        data,
        renderer,
        cameras,
        Set{HTTP.WebSockets.WebSocket}(),
        ReentrantLock(),
        Channel{Dict{String, Any}}(100),
        Dict{Any, Any}(),
        ReentrantLock(),
        default_teleop_ctx,
        actuator_map,
        false,  # running
        nothing,  # physics_task
        nothing,  # broadcast_task
        0.0,  # last_client_disconnect
        shutdown_grace_period,
        fps,
        0.0,  # last_broadcast_time
        Dict{String, Float64}()  # prev_state
    )

    println("SimulationInstance created for $(config.robot_id)")
    return instance
end

# =============================================================================
# Lifecycle Management
# =============================================================================

"""
    start!(instance::SimulationInstance)

Start the simulation physics loop and camera capture.

Spawns background tasks for:
1. Physics stepping
2. State broadcasting
3. Camera capture and streaming
"""
function start!(instance::SimulationInstance)
    if instance.running
        @warn "SimulationInstance already running" robot_id = instance.config.robot_id
        return
    end

    instance.running = true
    println("Starting SimulationInstance for $(instance.config.robot_id)...")

    # Start physics loop
    instance.physics_task = @async physics_loop!(instance)

    # Start broadcast loop
    instance.broadcast_task = @async broadcast_loop!(instance)

    println("SimulationInstance started for $(instance.config.robot_id)")
end

"""
    stop!(instance::SimulationInstance)

Stop the simulation and clean up resources.

Stops all background tasks, closes client connections, and frees
MuJoCo/OpenGL resources.
"""
function stop!(instance::SimulationInstance)
    if !instance.running
        return
    end

    println("Stopping SimulationInstance for $(instance.config.robot_id)...")
    instance.running = false

    # Close control clients
    @lock instance.control_clients_lock begin
        for ws in instance.control_clients
            try
                close(ws)
            catch
            end
        end
        empty!(instance.control_clients)
    end

    # Close camera clients
    for (_, cam_state) in instance.cameras
        @lock cam_state.clients_lock begin
            for ws in cam_state.clients
                try
                    close(ws)
                catch
                end
            end
            empty!(cam_state.clients)
        end
    end

    # Wait for tasks to finish
    if instance.physics_task !== nothing
        try
            wait(instance.physics_task)
        catch
        end
    end
    if instance.broadcast_task !== nothing
        try
            wait(instance.broadcast_task)
        catch
        end
    end

    # Clean up renderer
    cleanup!(instance.renderer)

    println("SimulationInstance stopped for $(instance.config.robot_id)")
end

"""
    should_shutdown(instance::SimulationInstance) -> Bool

Check if the instance should be shut down due to inactivity.

Returns true if:
1. No control clients are connected
2. Grace period has elapsed since last client disconnect
"""
function should_shutdown(instance::SimulationInstance)
    num_clients = @lock instance.control_clients_lock length(instance.control_clients)

    if num_clients > 0
        return false
    end

    if instance.last_client_disconnect == 0.0
        return false
    end

    elapsed = time() - instance.last_client_disconnect
    return elapsed >= instance.shutdown_grace_period
end

# =============================================================================
# Physics Loop
# =============================================================================

"""
    physics_loop!(instance::SimulationInstance)

Main physics loop - steps simulation and processes commands.
"""
function physics_loop!(instance::SimulationInstance)
    timestep = instance.model.opt.timestep
    println("Physics loop started (timestep=$(timestep)s)")

    while instance.running
        try
            # Process pending commands
            process_commands!(instance)

            # Step physics
            step!(instance.model, instance.data)

            # Sleep to approximate real-time
            sleep(timestep)
        catch e
            if instance.running
                @warn "Physics loop error" exception = (e, catch_backtrace())
            end
        end
    end

    println("Physics loop ended")
end

"""
    process_commands!(instance::SimulationInstance)

Process pending commands from the control channel.
"""
function process_commands!(instance::SimulationInstance)
    while isready(instance.control_channel)
        raw = take!(instance.control_channel)
        cmd = get(raw, "command", "")

        if cmd == "set_joints_state"
            joints = get(raw, "joints", Dict())
            ws = get(raw, "_ws", nothing)

            # Get client context
            client_ctx = ws !== nothing ?
                         (@lock instance.client_contexts_lock get(
                instance.client_contexts, ws, nothing)) :
                         nothing
            teleop_ctx = client_ctx !== nothing ? client_ctx : instance.default_teleop_ctx

            # Apply command
            apply_joint_command!(instance, joints, teleop_ctx)
        end
    end
end

"""
    apply_joint_command!(instance::SimulationInstance, joints::AbstractDict, teleop_ctx)

Apply joint commands to the simulation.
"""
function apply_joint_command!(instance::SimulationInstance, joints::AbstractDict, teleop_ctx)
    joints_float = Dict{String, Float64}(String(k) => Float64(v) for (k, v) in joints)
    mapped_joints = map_joints(teleop_ctx, joints_float, instance.model, instance.data)

    for (name, val) in mapped_joints
        if haskey(instance.actuator_map, name)
            idx = instance.actuator_map[name]
            # Check joint type for conversion
            joint_id = mj_name2id(instance.model, Int32(LibMuJoCo.mjOBJ_JOINT), name)
            if joint_id >= 0 && instance.model.jnt_type[joint_id + 1] == 2  # mjJNT_SLIDE
                instance.data.ctrl[idx] = val  # Already in meters
            else
                instance.data.ctrl[idx] = deg2rad(val)
            end
        end
    end
end

# =============================================================================
# Broadcast Loop
# =============================================================================

"""
    broadcast_loop!(instance::SimulationInstance)

Periodically broadcast state to control clients and capture/stream cameras.
"""
function broadcast_loop!(instance::SimulationInstance)
    interval = 1.0 / instance.fps
    println("Broadcast loop started ($(instance.fps) fps)")

    while instance.running
        try
            current_time = time()

            if current_time - instance.last_broadcast_time >= interval
                # Broadcast state to control clients
                broadcast_state!(instance)

                # Capture and stream cameras
                capture_and_stream_cameras!(instance)

                instance.last_broadcast_time = current_time
            end

            sleep(0.001)  # Small sleep to prevent busy-waiting
        catch e
            if instance.running
                @warn "Broadcast loop error" exception = (e, catch_backtrace())
            end
        end
    end

    println("Broadcast loop ended")
end

"""
    broadcast_state!(instance::SimulationInstance)

Broadcast current state to all control clients.
"""
function broadcast_state!(instance::SimulationInstance)
    clients = @lock instance.control_clients_lock copy(instance.control_clients)

    if isempty(clients)
        return
    end

    current_time = time()
    dead_clients = HTTP.WebSockets.WebSocket[]

    for ws in clients
        try
            # Get client-specific context
            client_ctx = @lock instance.client_contexts_lock get(
                instance.client_contexts, ws, nothing)
            teleop_ctx = client_ctx !== nothing ? client_ctx : instance.default_teleop_ctx

            # Get state in leader format
            state = get_state_for_leader(teleop_ctx, instance.model, instance.data)

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

    # Remove dead clients
    if !isempty(dead_clients)
        @lock instance.control_clients_lock begin
            for ws in dead_clients
                delete!(instance.control_clients, ws)
            end
        end
        @lock instance.client_contexts_lock begin
            for ws in dead_clients
                delete!(instance.client_contexts, ws)
            end
        end

        # Update last disconnect time
        if @lock instance.control_clients_lock isempty(instance.control_clients)
            instance.last_client_disconnect = time()
        end
    end
end

"""
    capture_and_stream_cameras!(instance::SimulationInstance)

Capture frames from all cameras and stream to connected clients.
"""
function capture_and_stream_cameras!(instance::SimulationInstance)
    for (name, cam_state) in instance.cameras
        clients = @lock cam_state.clients_lock copy(cam_state.clients)

        if isempty(clients)
            continue  # Skip cameras with no clients
        end

        # Render camera
        rgb_data = render_camera_flipped!(
            instance.renderer,
            instance.model,
            instance.data,
            cam_state.camera
        )

        # Encode as JPEG
        jpeg_bytes = encode_jpeg(rgb_data, instance.renderer.width, instance.renderer.height)

        # Send to clients
        dead_clients = HTTP.WebSockets.WebSocket[]
        for ws in clients
            try
                HTTP.WebSockets.send(ws, jpeg_bytes)
            catch
                push!(dead_clients, ws)
            end
        end

        # Remove dead clients
        if !isempty(dead_clients)
            @lock cam_state.clients_lock begin
                for ws in dead_clients
                    delete!(cam_state.clients, ws)
                end
            end
        end
    end
end

"""
    encode_jpeg(rgb_data::Vector{UInt8}, width::Int, height::Int) -> Vector{UInt8}

Encode RGB data as JPEG.
"""
function encode_jpeg(rgb_data::Vector{UInt8}, width::Int, height::Int)
    # Reshape to image format
    img_raw = reshape(rgb_data, 3, width, height)
    img_n0f8 = reinterpret(N0f8, img_raw)

    # Create RGB image
    img = Matrix{RGB{N0f8}}(undef, height, width)
    for j in 1:height
        for i in 1:width
            img[j, i] = RGB(img_n0f8[1, i, j], img_n0f8[2, i, j], img_n0f8[3, i, j])
        end
    end

    # Encode to JPEG
    io = IOBuffer()
    FileIO.save(FileIO.Stream{FileIO.format"JPEG"}(io), img)
    return take!(io)
end

# =============================================================================
# Client Management
# =============================================================================

"""
    add_control_client!(instance::SimulationInstance, ws::WebSocket, leader_type)

Add a control client to the simulation.

Creates a per-client teleop context based on the leader type.
"""
function add_control_client!(instance::SimulationInstance, ws::HTTP.WebSockets.WebSocket,
        leader_type)
    # Create per-client context
    resolved_type = if leader_type isa Type
        leader_type
    elseif leader_type isa AbstractString && !isempty(leader_type)
        get(ROBOT_TYPE_MAP, lowercase(leader_type), instance.config.default_leader_type)
    else
        instance.config.default_leader_type
    end

    client_ctx = create_teleop_context(
        resolved_type,
        instance.config.follower_type,
        instance.model,
        instance.data;
        project_root = instance.project_root)

    @lock instance.client_contexts_lock begin
        instance.client_contexts[ws] = client_ctx
    end

    @lock instance.control_clients_lock begin
        push!(instance.control_clients, ws)
    end

    println("Control client connected (leader=$(resolved_type))")
end

"""
    remove_control_client!(instance::SimulationInstance, ws::WebSocket)

Remove a control client from the simulation.
"""
function remove_control_client!(instance::SimulationInstance, ws::HTTP.WebSockets.WebSocket)
    @lock instance.control_clients_lock begin
        delete!(instance.control_clients, ws)
    end

    @lock instance.client_contexts_lock begin
        delete!(instance.client_contexts, ws)
    end

    # Update last disconnect time if no more clients
    num_clients = @lock instance.control_clients_lock length(instance.control_clients)
    if num_clients == 0
        instance.last_client_disconnect = time()
    end

    println("Control client disconnected")
end

"""
    add_camera_client!(instance::SimulationInstance, camera_name::String, ws::WebSocket)

Add a camera stream client.
"""
function add_camera_client!(instance::SimulationInstance, camera_name::AbstractString,
        ws::HTTP.WebSockets.WebSocket)
    if !haskey(instance.cameras, camera_name)
        @warn "Unknown camera" camera_name
        return false
    end

    cam_state = instance.cameras[camera_name]
    @lock cam_state.clients_lock begin
        push!(cam_state.clients, ws)
    end

    println("Camera client connected: $camera_name")
    return true
end

"""
    remove_camera_client!(instance::SimulationInstance, camera_name::String, ws::WebSocket)

Remove a camera stream client.
"""
function remove_camera_client!(instance::SimulationInstance, camera_name::AbstractString,
        ws::HTTP.WebSockets.WebSocket)
    if !haskey(instance.cameras, camera_name)
        return
    end

    cam_state = instance.cameras[camera_name]
    @lock cam_state.clients_lock begin
        delete!(cam_state.clients, ws)
    end

    println("Camera client disconnected: $camera_name")
end

"""
    get_client_count(instance::SimulationInstance) -> Int

Get the total number of connected clients (control + cameras).
"""
function get_client_count(instance::SimulationInstance)
    count = @lock instance.control_clients_lock length(instance.control_clients)

    for (_, cam_state) in instance.cameras
        count += @lock cam_state.clients_lock length(cam_state.clients)
    end

    return count
end

"""
    get_control_client_count(instance::SimulationInstance) -> Int

Get the number of connected control clients.
"""
function get_control_client_count(instance::SimulationInstance)
    return @lock instance.control_clients_lock length(instance.control_clients)
end

# =============================================================================
# Command Handling
# =============================================================================

"""
    submit_command!(instance::SimulationInstance, command::AbstractDict)

Submit a command to be processed by the physics loop.
"""
function submit_command!(instance::SimulationInstance, command::AbstractDict)
    if instance.running
        put!(instance.control_channel, command)
    end
end
