# Trossen Robot Arm Simulation with WebSocket Control and Multi-Camera Capture
#
# This script runs a MuJoCo simulation of the Trossen WXAI robot arm with:
# - Unified WebSocket server on port 8080 with path-based routing
# - Multi-camera capture (WebSocket streams)
# - Gripper-mounted camera for first-person view
# - Graspable cubes in the environment
# - Interactive 3D visualization
#
# COMPATIBILITY: Uses TeleoperatorMapping for flexible leader robot support.
# By default, accepts SO101 joint names and maps them to Trossen joints internally.
#
# Usage:
#   julia --project=. -t 4 examples/trossen/websocket_sim.jl
#   julia --project=. -t 4 examples/trossen/websocket_sim.jl --leader=so101
#   julia --project=. -t 4 examples/trossen/websocket_sim.jl --leader=trossen
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

# --- Scene Setup ---
xml_path = joinpath(@__DIR__, "..", "..", "robots", "trossen_arm_mujoco",
    "trossen_arm_mujoco", "assets", "wxai", "scene.xml")
println("Loading Trossen model from: $xml_path")

# Generate cubes in front of the robot, within its reach
# Trossen has longer reach than SO101, so we use larger radius
cubes = generate_cubes(50, radius_min = 0.12, radius_max = 0.55)

# --- Gripper Camera ---
# Camera on the end-effector body (link_6), looking forward along the gripper direction.
# In the Trossen model, the gripper extends in the +X direction from link_6.
gripper_camera = BodyCamera(
    name = "gripper_cam",
    body = "link_6",
    pos = [0.05, 0.0, 0.0],              # 5cm along gripper direction
    quat = [0.5, 0.5, -0.5, -0.5],       # Rotate to look along +X (gripper direction)
    fovy = 90.0
)

# Build scene with cubes and gripper camera
# Note: Trossen model already has collision boxes on gripper fingers
model, data = build_scene(xml_path, cubes, cameras = [gripper_camera])

# --- Actuator Mapping ---
# Get Trossen actuator names and create lookup
trossen_actuator_names = [unsafe_string(mj_id2name(
                              model, Int32(LibMuJoCo.mjOBJ_ACTUATOR), i - 1))
                          for i in 1:(model.nu)]
trossen_actuator_map = Dict(name => i for (i, name) in enumerate(trossen_actuator_names))
println("Trossen actuators: ", trossen_actuator_names)

# --- Teleoperation Configuration ---
const DefaultLeaderType = parse_leader_type(ARGS; default = SO101)  # Default to SO101 for backward compat
const FollowerType = TrossenWXAI
const project_root = joinpath(@__DIR__, "..", "..")

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

# Default context for fallback
const default_teleop_ctx = create_teleop_context(
    DefaultLeaderType, FollowerType, model, data;
    project_root = project_root)
println(describe(default_teleop_ctx))

# --- WebSocket Server (Unified - single port with path-based routing) ---
server = UnifiedServer(port = 8080, robot = "trossen/wxai", fps = 30.0)

# Enable per-client leader types via ?leader=X query parameter
set_context_factory!(server, create_client_context, DefaultLeaderType)

# Robot-specific: how to get joint state (in leader's joint name format)
function get_joint_state(model, data, ctx = nothing)
    teleop_ctx = ctx !== nothing ? ctx : default_teleop_ctx
    return get_state_for_leader(teleop_ctx, model, data)
end

# Robot-specific: how to apply joint commands (leader format -> Trossen)
function apply_joint_command!(data, actuator_map, joints, ctx = nothing)
    teleop_ctx = ctx !== nothing ? ctx : default_teleop_ctx
    joints_float = Dict{String, Float64}(String(k) => Float64(v) for (k, v) in joints)
    mapped_joints = map_joints(teleop_ctx, joints_float, model, data)

    for (name, val) in mapped_joints
        if haskey(actuator_map, name)
            idx = actuator_map[name]
            # Trossen gripper (left_gripper) is a slide joint in meters
            if name == "left_gripper"
                data.ctrl[idx] = val  # Already in meters
            else
                data.ctrl[idx] = deg2rad(val)
            end
        end
    end
end

# Start WebSocket server
start!(server, get_joint_state)

# --- Controller Function ---
function ctrl!(m, d)
    process_commands!(server, d, trossen_actuator_map, apply_joint_command!)

    # Keep joint_4 (wrist yaw) at 0 - no SO101 equivalent
    if haskey(trossen_actuator_map, "joint_4")
        d.ctrl[trossen_actuator_map["joint_4"]] = 0.0
    end

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
            lookat = [0.0, 0.0, 0.25],    # Slightly higher for taller Trossen arm
            distance = 1.4,               # Slightly further back
            azimuth = 180.0,
            elevation = -20.0,
            output = WebSocketOutput(server = server)
        ),
        # Side camera: external view from the side
        CameraSpec(
            name = "side",
            lookat = [0.0, 0.0, 0.25],
            distance = 1.4,
            azimuth = 90.0,
            elevation = -20.0,
            output = WebSocketOutput(server = server)
        ),
        # Orbit camera: rotating external view
        CameraSpec(
            name = "orbit",
            lookat = [0.0, 0.0, 0.25],
            distance = 1.8,
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

println("\n" * "=" ^ 70)
println("Trossen WXAI Robot Arm Simulation")
println("=" ^ 70)
print_teleop_banner(DefaultLeaderType, FollowerType, default_teleop_ctx.strategy)
println("\nWebSocket Endpoints (all on port 8080):")
println("  Control:        ws://localhost:8080/trossen/wxai/control")
println("  Control:        ws://localhost:8080/trossen/wxai/control?leader=<type>")
println("  Front camera:   ws://localhost:8080/trossen/wxai/cameras/front")
println("  Side camera:    ws://localhost:8080/trossen/wxai/cameras/side")
println("  Orbit camera:   ws://localhost:8080/trossen/wxai/cameras/orbit")
println("  Gripper camera: ws://localhost:8080/trossen/wxai/cameras/gripper")
println("\nPer-client leader type support:")
println("  Default (CLI):     --leader=so101")
println("  Per-connection:    ?leader=so101, ?leader=trossen, ?leader=lekiwi")
println("=" ^ 70)
println("\nPress 'Space' to pause/unpause, 'F1' for help, close window to exit\n")

run_with_capture!(model, data,
    controller = ctrl!,
    capture = capture_config
)

println("Simulation ended.")
