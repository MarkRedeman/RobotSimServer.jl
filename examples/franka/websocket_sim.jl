# Franka Panda Robot Simulation with WebSocket Control and Multi-Camera Capture
#
# This script runs a MuJoCo simulation of the Franka Panda 7-DOF robot arm with:
# - Unified WebSocket server on port 8080 with path-based routing
# - Multi-camera capture (WebSocket streams)
# - Gripper-mounted camera for first-person view
# - Graspable cubes in the environment
# - Interactive 3D visualization
#
# Uses the MuJoCo Menagerie Franka Emika Panda model with TeleoperatorMapping
# for cross-robot teleoperation support.
#
# TELEOPERATION MODES:
# - --leader=so101 (default): Accepts SO101 joint names, uses IK-based mapping
# - --leader=franka: Accepts native Franka joint names (direct control)
#
# Usage:
#   julia --project=. -t 4 examples/franka/websocket_sim.jl
#   julia --project=. -t 4 examples/franka/websocket_sim.jl --leader=so101
#   julia --project=. -t 4 examples/franka/websocket_sim.jl --leader=franka
#
# Connect with test client:
#   julia --project=. examples/clients/ws_client.jl
#
# WebSocket API:
#   Commands (joint format depends on leader type):
#     SO101 leader: {"command": "set_joints_state", "joints": {"shoulder_pan": 45.0, ...}}
#     Franka leader: {"command": "set_joints_state", "joints": {"joint1": 45.0, ...}}
#   All joint values in degrees, gripper in degrees (-10 to 100 for SO101 format).
#
#   Events:
#     {"event": "state_was_updated", "timestamp": ..., "state": {"shoulder_pan": 45.0, ...}}
#   Reports joints in leader's naming convention.

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

# =============================================================================
# Configuration
# =============================================================================

# End-effector body for IK targeting (reference)
const EE_BODY = "link7"

# Gripper parameters (for reference - used by TeleoperatorMapping)
const FRANKA_GRIPPER_MIN = 0.0      # Fully closed (actuator units = 0)
const FRANKA_GRIPPER_MAX = 255.0    # Fully open (actuator units = 255)
const SO101_GRIPPER_MIN = -10.0     # Fully closed (degrees)
const SO101_GRIPPER_MAX = 100.0     # Fully open (degrees)

# =============================================================================
# Scene Setup
# =============================================================================
# Use scene.xml which includes a floor (panda.xml alone has no ground plane)
xml_path = joinpath(@__DIR__, "..", "..", "robots", "google-deepmind",
    "franka_emika_panda", "scene.xml")
println("Loading Franka Panda from: $xml_path")

# Generate cubes in front of the robot, within its reach
# Franka has ~0.855m reach (vs SO101 ~0.38m), so use larger radius
cubes = generate_cubes(50,
    radius_min = 0.30,
    radius_max = 0.70,
    size = 0.025,
    z = 0.03)

# --- Cameras ---
# The menagerie model doesn't have a built-in wrist camera, so we add one.
# Also add a gripper camera for close-up view.
wrist_camera = BodyCamera(
    name = "wrist_cam",
    body = "link7",
    pos = [0.05, -0.05, 0.15],            # Offset from link7
    quat = [0.707, 0.0, 0.707, 0.0],      # Look forward
    fovy = 60.0
)

gripper_camera = BodyCamera(
    name = "gripper_cam",
    body = "hand",
    pos = [0.0, 0.0, 0.10],               # 10cm forward (toward fingertips)
    quat = [0.707, 0.0, 0.707, 0.0],      # Rotate to look along +Z
    fovy = 90.0
)

# Build scene with cubes and cameras
model, data = build_scene(xml_path, cubes, cameras = [wrist_camera, gripper_camera])

println("Loaded model with $(model.nq) DOF, $(model.nu) actuators")

# =============================================================================
# Teleoperation Configuration
# =============================================================================
const LeaderType = parse_leader_type(ARGS; default = SO101)
const FollowerType = FrankaPanda
const project_root = joinpath(@__DIR__, "..", "..")
const teleop_ctx = create_teleop_context(LeaderType, FollowerType, model, data;
    project_root = project_root)
println(describe(teleop_ctx))

# =============================================================================
# Actuator Mapping
# =============================================================================
# Menagerie Franka actuators:
#   1-7: actuator1 through actuator7 (arm, position control)
#   8: actuator8 (gripper via tendon, 0-255 range)

actuator_names = [unsafe_string(mj_id2name(model, Int32(LibMuJoCo.mjOBJ_ACTUATOR), i - 1))
                  for i in 1:(model.nu)]
actuator_map = Dict(name => i for (i, name) in enumerate(actuator_names))
println("Actuators: ", join(actuator_names, ", "))

# =============================================================================
# WebSocket Server (Unified - single port with path-based routing)
# =============================================================================
server = UnifiedServer(port = 8080, robot = "franka", fps = 30.0)

# Get joint state using TeleoperatorMapping (reports in leader's naming convention)
function get_joint_state(model, data)
    return get_state_for_leader(teleop_ctx, model, data)
end

# Apply joint commands using TeleoperatorMapping
function apply_joint_command!(franka_data, actuator_map_unused, joints)
    joints_float = Dict{String, Float64}(String(k) => Float64(v) for (k, v) in joints)
    mapped_joints = map_joints(teleop_ctx, joints_float, model, data)

    # Apply computed joint angles to Franka arm actuators (first 7)
    for i in 1:7
        joint_name = "joint$i"
        if haskey(mapped_joints, joint_name)
            franka_data.ctrl[i] = deg2rad(mapped_joints[joint_name])
        end
    end

    # Handle gripper
    if haskey(mapped_joints, "gripper")
        # Franka uses 0-255 actuator units
        franka_data.ctrl[8] = mapped_joints["gripper"]
    end
end

# Start WebSocket server
start!(server, get_joint_state)

# =============================================================================
# Controller Function
# =============================================================================
function ctrl!(m, d)
    process_commands!(server, d, actuator_map, apply_joint_command!)
    maybe_broadcast!(server, m, d, get_joint_state)
end

# =============================================================================
# Camera Configuration
# =============================================================================
# Franka is larger than SO101, so cameras need to be further back
base_distance = 2.0
lookat_height = 0.5

capture_config = CaptureConfig(
    width = 640,
    height = 480,
    fps = 30.0,
    cameras = [
        # Front camera: external view from the front
        CameraSpec(
            name = "front",
            lookat = [0.0, 0.0, lookat_height],
            distance = base_distance,
            azimuth = 180.0,
            elevation = -20.0,
            output = WebSocketOutput(server = server)
        ),
        # Side camera: external view from the side
        CameraSpec(
            name = "side",
            lookat = [0.0, 0.0, lookat_height],
            distance = base_distance,
            azimuth = 90.0,
            elevation = -20.0,
            output = WebSocketOutput(server = server)
        ),
        # Orbit camera: rotating external view
        CameraSpec(
            name = "orbit",
            lookat = [0.0, 0.0, lookat_height],
            distance = base_distance * 1.3,
            azimuth = 0.0,
            elevation = -30.0,
            orbiting = true,
            orbit_speed = 30.0,
            output = WebSocketOutput(server = server)
        ),
        # Gripper camera: added to hand body (matches SO101/Trossen port assignment)
        CameraSpec(
            name = "gripper",
            mode = :fixed,
            model_camera = "gripper_cam",
            output = WebSocketOutput(server = server)
        ),
        # Wrist camera: added to link7 (Franka-specific extra camera)
        CameraSpec(
            name = "wrist",
            mode = :fixed,
            model_camera = "wrist_cam",
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

# =============================================================================
# Run Visualization
# =============================================================================
println("\nInitializing visualizer...")
init_visualiser()

println("\n" * "="^70)
println("Franka Panda Simulation with Teleoperation")
println("="^70)
print_teleop_banner(LeaderType, FollowerType, teleop_ctx.strategy)
println("\nWorkspace scale: $(round(workspace_scale(teleop_ctx), digits=2))x")
println("\nWebSocket Endpoints (all on port 8080):")
println("  Control:        ws://localhost:8080/franka/control")
println("  Front camera:   ws://localhost:8080/franka/cameras/front")
println("  Side camera:    ws://localhost:8080/franka/cameras/side")
println("  Orbit camera:   ws://localhost:8080/franka/cameras/orbit")
println("  Gripper camera: ws://localhost:8080/franka/cameras/gripper")
println("  Wrist camera:   ws://localhost:8080/franka/cameras/wrist")
println("\nUsage:")
println("  --leader=so101  (default) Accept SO101 joint names (IK mapping)")
println("  --leader=franka Accept Franka native joint names")
println("="^70)
println("\nPress 'Space' to pause/unpause, 'F1' for help, close window to exit\n")

run_with_capture!(model, data,
    controller = ctrl!,
    capture = capture_config
)

println("Simulation ended.")
