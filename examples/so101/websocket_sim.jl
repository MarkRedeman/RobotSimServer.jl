# SO101 Robot Arm Simulation with WebSocket Control and Multi-Camera Capture
#
# This script runs a MuJoCo simulation of the SO101 robot arm with:
# - Unified WebSocket server on port 8080 with path-based routing
# - Multi-camera capture (video files and WebSocket streams)
# - Gripper-mounted camera for first-person view
# - Graspable cubes in the environment
# - Interactive 3D visualization
#
# Usage:
#   julia --project=. -t 4 examples/so101/websocket_sim.jl
#
# Connect with test client:
#   julia --project=. examples/clients/ws_client.jl

using MuJoCo
using MuJoCo.LibMuJoCo

# Load shared modules
include("../../src/SceneBuilder.jl")
include("../../src/UnifiedWebSocketServer.jl")
include("../../src/capture/Capture.jl")
include("../../src/RobotTypes.jl")
include("../../src/Kinematics.jl")
include("../../src/TeleoperatorMapping.jl")
include("../../src/CLIUtils.jl")

# --- Teleoperation Configuration ---
const DefaultLeaderType = parse_leader_type(ARGS; default = SO101)
const FollowerType = SO101

# Create a context factory for per-client teleop contexts
function create_client_context(leader_type)
    resolved_type = if leader_type isa Type
        leader_type
    elseif leader_type isa AbstractString
        get(ROBOT_TYPE_MAP, lowercase(leader_type), DefaultLeaderType)
    else
        DefaultLeaderType
    end
    println("Created teleop context: $(resolved_type) → $(FollowerType)")
    return create_teleop_context(resolved_type, FollowerType, model, data;
        project_root = project_root)
end

# --- Scene Setup ---
xml_path = joinpath(
    @__DIR__, "..", "..", "robots", "SO-ARM100", "Simulation", "SO101", "scene.xml")
println("Loading model from: $xml_path")

# Generate cubes in front of the robot, within its reach
cubes = generate_cubes(50, radius_min = 0.10, radius_max = 0.38)

# --- Gripper Camera ---
# Camera inside the gripper body, looking forward along the gripper's grasping direction.
# In gripper frame, -Z points toward the gripper tips.
gripper_camera = BodyCamera(
    name = "gripper_cam",
    body = "gripper",
    pos = [0.0, 0.0, -0.04],           # 4cm toward tips
    quat = [1.0, 0.0, 0.0, 0.0],       # Identity - looks along -Z
    fovy = 90.0
)

# --- Gripper Collision Boxes ---
# Add simple box colliders for reliable collision detection (mesh colliders are unreliable)
gripper_collisions = default_gripper_collisions()

# Build scene with cubes, gripper camera, and collision primitives
model,
data = build_scene(
    xml_path, cubes, cameras = [gripper_camera], collisions = gripper_collisions)

# --- Actuator Mapping ---
actuator_names = [unsafe_string(mj_id2name(model, Int32(LibMuJoCo.mjOBJ_ACTUATOR), i - 1))
                  for i in 1:(model.nu)]
actuator_map = Dict(name => i for (i, name) in enumerate(actuator_names))
println("Available actuators: ", actuator_names)

# --- Teleoperator Context ---
const project_root = joinpath(@__DIR__, "..", "..")
const default_teleop_ctx = create_teleop_context(
    DefaultLeaderType, FollowerType, model, data;
    project_root = project_root)
println(describe(default_teleop_ctx))

# --- WebSocket Server (Unified - single port with path-based routing) ---
server = UnifiedServer(port = 8080, robot = "so101", fps = 30.0)

# Enable per-client leader types via ?leader=X query parameter
set_context_factory!(server, create_client_context, DefaultLeaderType)

# Robot-specific: how to get joint state (in degrees, in LEADER's joint names)
function get_joint_state(model, data, ctx = nothing)
    teleop_ctx = ctx !== nothing ? ctx : default_teleop_ctx
    return get_state_for_leader(teleop_ctx, model, data)
end

# Robot-specific: how to apply joint commands (mapped from leader to follower)
function apply_joint_command!(data, actuator_map, joints, ctx = nothing)
    teleop_ctx = ctx !== nothing ? ctx : default_teleop_ctx

    # Convert Any to Float64 Dict
    joints_float = Dict{String, Float64}(String(k) => Float64(v) for (k, v) in joints)

    # Map from leader's joint format to follower's
    mapped_joints = map_joints(teleop_ctx, joints_float, model, data)

    # Apply mapped joints (now in follower's native format)
    for (name, val) in mapped_joints
        if haskey(actuator_map, name)
            idx = actuator_map[name]
            # Check if this is a prismatic joint (don't convert deg->rad)
            joint_id = mj_name2id(model, Int32(LibMuJoCo.mjOBJ_JOINT), name)
            if joint_id >= 0 && model.jnt_type[joint_id + 1] == 2  # mjJNT_SLIDE
                data.ctrl[idx] = val  # Already in meters
            else
                data.ctrl[idx] = deg2rad(val)
            end
        else
            @warn "Unknown actuator: $name"
        end
    end
end

# Start WebSocket server
start!(server, get_joint_state)

# --- Controller Function ---
function ctrl!(m, d)
    process_commands!(server, d, actuator_map, apply_joint_command!)
    maybe_broadcast!(server, m, d, get_joint_state)
end

# --- Camera Configuration ---
# Multi-camera capture with WebSocket streaming for all cameras
capture_config = CaptureConfig(
    width = 640,
    height = 480,
    fps = 30.0,
    cameras = [
        # Front camera: external view from the front
        CameraSpec(
            name = "front",
            lookat = [0.0, 0.0, 0.2],
            distance = 1.2,
            azimuth = 180.0,
            elevation = -20.0,
            output = WebSocketOutput(server = server)
        ),
        # Side camera: external view from the side
        CameraSpec(
            name = "side",
            lookat = [0.0, 0.0, 0.2],
            distance = 1.2,
            azimuth = 90.0,
            elevation = -20.0,
            output = WebSocketOutput(server = server)
        ),
        # Orbit camera: rotating external view
        CameraSpec(
            name = "orbit",
            lookat = [0.0, 0.0, 0.2],
            distance = 1.5,
            azimuth = 0.0,
            elevation = -30.0,
            orbiting = true,
            orbit_speed = 30.0,
            output = WebSocketOutput(server = server)
        ),
        # Gripper camera: first-person view from inside the gripper
        CameraSpec(
            name = "gripper",
            mode = :fixed,
            model_camera = "gripper_cam",
            output = WebSocketOutput(server = server)
        )
    ]
)

# Register camera endpoints with the unified server
for cam in capture_config.cameras
    if cam.output isa WebSocketOutput && cam.output.server !== nothing
        register_camera!(server, cam.name)
    end
end

# --- Run Visualization ---
println("\nInitializing visualizer...")
init_visualiser()

println("\n" * "="^70)
println("SO101 Robot Arm Simulation")
println("="^70)
print_teleop_banner(DefaultLeaderType, FollowerType, default_teleop_ctx.strategy)
println("\nWebSocket Endpoints (all on port 8080):")
println("  Control:        ws://localhost:8080/so101/control")
println("  Control:        ws://localhost:8080/so101/control?leader=<type>")
println("  Front camera:   ws://localhost:8080/so101/cameras/front")
println("  Side camera:    ws://localhost:8080/so101/cameras/side")
println("  Orbit camera:   ws://localhost:8080/so101/cameras/orbit")
println("  Gripper camera: ws://localhost:8080/so101/cameras/gripper")
println("\nPer-client leader type support:")
println("  Default (CLI):     --leader=so101")
println("  Per-connection:    ?leader=so101, ?leader=trossen, ?leader=lekiwi")
println("="^70)
println("\nPress 'Space' to pause/unpause, 'F1' for help, close window to exit\n")

run_with_capture!(model, data,
    controller = ctrl!,
    capture = capture_config
)

println("Simulation ended.")
