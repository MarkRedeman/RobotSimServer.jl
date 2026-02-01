# Fanuc Industrial Robot Simulation with WebSocket Control and Multi-Camera Capture
#
# This script runs a MuJoCo simulation of Fanuc industrial robot arms with:
# - Unified WebSocket server on port 8080 with path-based routing
# - Multi-camera capture (WebSocket streams)
# - Interactive 3D visualization
# - Support for 19 robot variants (5-12 DOF)
#
# IK-BASED CONTROL: Uses the TeleoperatorMapping system to accept leader robot
# joint commands and map them via inverse kinematics. This provides natural motion
# mapping despite different robot kinematics (joint axes, link lengths, etc.).
#
# The mapping works as follows:
# 1. Apply leader joint angles to a "shadow" leader model
# 2. Compute leader gripper position via forward kinematics
# 3. Scale position from leader workspace to Fanuc workspace
# 4. Solve inverse kinematics to get Fanuc joint angles
# 5. Apply to Fanuc actuators
#
# Usage:
#   julia --project=. -t 4 examples/fanuc/websocket_sim.jl [robot_name] [--leader=TYPE]
#
# Examples:
#   julia --project=. -t 4 examples/fanuc/websocket_sim.jl           # Default: m10ia, leader: SO101
#   julia --project=. -t 4 examples/fanuc/websocket_sim.jl crx10ial  # CRX collaborative robot
#   julia --project=. -t 4 examples/fanuc/websocket_sim.jl m900ib700 # Large 12-DOF robot
#   julia --project=. -t 4 examples/fanuc/websocket_sim.jl m10ia --leader=trossen
#
# WebSocket API:
#   Commands (leader robot format - mapped via IK):
#     {"command": "set_joints_state", "joints": {"shoulder_pan": 45.0, "shoulder_lift": -30.0, ...}}
#   All values in degrees.
#
#   Events:
#     {"event": "state_was_updated", "timestamp": ..., "state": {"shoulder_pan": 45.0, ...}}

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

# --- Robot Discovery ---
function discover_robots()
    fanuc_dir = joinpath(@__DIR__, "..", "..", "robots", "fanuc_mujoco")
    if !isdir(fanuc_dir)
        return String[]
    end
    robots = String[]
    for entry in readdir(fanuc_dir)
        scene_path = joinpath(fanuc_dir, entry, "scene.xml")
        if isfile(scene_path)
            push!(robots, entry)
        end
    end
    return sort(robots)
end

const ROBOTS = discover_robots()
const DEFAULT_ROBOT = "m10ia"  # Classic yellow Fanuc industrial robot

# Get robot from command line argument or use default
# Filter out --leader= and other flag arguments
function get_robot_from_args(args, default)
    for arg in args
        if !startswith(arg, "--")
            return arg
        end
    end
    return default
end
robot = get_robot_from_args(ARGS, DEFAULT_ROBOT)

# Validate robot selection
if robot ∉ ROBOTS
    println("Unknown robot: $robot")
    println()
    println("Available robots:")
    for r in ROBOTS
        println("  - $r")
    end
    println()
    println("To generate additional variants:")
    println("  python3 scripts/convert_fanuc_industrial.py <variant>")
    println()
    println("List all available variants:")
    println("  python3 scripts/convert_fanuc_industrial.py --list")
    exit(1)
end

# --- Scene Setup ---
xml_path = joinpath(@__DIR__, "..", "..", "robots", "fanuc_mujoco", robot, "scene.xml")
println("Loading Fanuc $robot from: $xml_path")

# Calculate cube placement based on robot size
# Larger robots have longer reach, so cubes should be placed further out
cube_scale = if occursin("m900", robot)
    3.0  # Very large robot
elseif occursin("m710", robot) || occursin("r2000", robot) || occursin("r1000", robot)
    2.0  # Large robot
elseif occursin("crx", robot) || occursin("lrmate", robot) || occursin("cr7", robot)
    1.0  # Small/medium robot
else
    1.5  # Default industrial size
end

# Generate cubes in front of the robot, scaled to robot reach
# Fanuc robots are much larger than SO101, so use bigger cubes
cubes = generate_cubes(50,
    radius_min = 0.15 * cube_scale,
    radius_max = 0.50 * cube_scale,
    size = 0.03 * cube_scale,      # Bigger cubes for industrial robots
    z = 0.04 * cube_scale)          # Higher off the ground

# --- Gripper Camera ---
# Camera attached to the end-effector (link_6/flange), looking forward.
# Fanuc robots have the tool flange along the +X axis of link_6.
# We position the camera slightly forward and use a quaternion to look along +X.
# Quaternion [0.5, 0.5, -0.5, 0.5] rotates camera from -Z to +X viewing direction.
gripper_camera = BodyCamera(
    name = "gripper_cam",
    body = "link_6",
    pos = [0.05, 0.0, 0.0],              # 5cm forward from flange
    quat = [0.5, 0.5, -0.5, 0.5],        # Look along +X axis
    fovy = 90.0
)

# Build scene with cubes and gripper camera
model, data = build_scene(xml_path, cubes, cameras = [gripper_camera])

println("Loaded model with $(model.nq) DOF, $(model.nu) actuators")

# --- Joint and Actuator Mapping ---
# Get joint names from the model (joint_1, joint_2, etc.)
joint_names_list = String[]
for i in 0:(model.njnt - 1)
    name = unsafe_string(mj_id2name(model, Int32(LibMuJoCo.mjOBJ_JOINT), i))
    if !isempty(name) && startswith(name, "joint_")
        push!(joint_names_list, name)
    end
end

# Get actuator names and create lookup
actuator_names = [unsafe_string(mj_id2name(model, Int32(LibMuJoCo.mjOBJ_ACTUATOR), i - 1))
                  for i in 1:(model.nu)]
actuator_map = Dict(name => i for (i, name) in enumerate(actuator_names))

# Create joint name -> actuator index mapping (accepts both joint_1 and joint_1_actuator)
joint_to_actuator = Dict{String, Int}()
for (i, act_name) in enumerate(actuator_names)
    joint_to_actuator[act_name] = i
    # Also accept joint name without _actuator suffix
    if endswith(act_name, "_actuator")
        joint_name = replace(act_name, "_actuator" => "")
        joint_to_actuator[joint_name] = i
    end
end

println("Joints: ", join(joint_names_list, ", "))

# --- Teleoperation Configuration ---
const LeaderType = parse_leader_type(ARGS; default = SO101)
const FollowerType = FanucArm
const project_root = joinpath(@__DIR__, "..", "..")
const teleop_ctx = create_teleop_context(LeaderType, FollowerType, model, data;
    project_root = project_root)
println(describe(teleop_ctx))

# --- Unified WebSocket Server ---
server = UnifiedServer(port = 8080, robot = "fanuc/$(robot)", fps = 30.0)

# Robot-specific: how to get joint state (in leader format for compatibility)
function get_joint_state(model, data)
    return get_state_for_leader(teleop_ctx, model, data)
end

# Robot-specific: how to apply joint commands using teleop mapping
# Uses TeleoperatorMapping to convert leader joints -> Fanuc joints via IK
function apply_joint_command!(fanuc_data, actuator_map_unused, joints)
    joints_float = Dict{String, Float64}(String(k) => Float64(v) for (k, v) in joints)
    mapped_joints = map_joints(teleop_ctx, joints_float, model, data)

    # Apply computed joint angles to Fanuc actuators
    num_joints = min(6, model.nu)
    for i in 1:num_joints
        joint_name = "joint_$i"
        if haskey(mapped_joints, joint_name)
            fanuc_data.ctrl[i] = deg2rad(mapped_joints[joint_name])
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
# Calculate camera distances based on robot size (larger robots need further cameras)
# M900iB/700 is huge, CRX robots are small
robot_scale = if occursin("m900", robot)
    2.5  # Very large robot
elseif occursin("m710", robot) || occursin("r2000", robot) || occursin("r1000", robot)
    1.8  # Large robot
elseif occursin("crx", robot) || occursin("lrmate", robot) || occursin("cr7", robot)
    1.0  # Small/medium robot
else
    1.3  # Default industrial size
end

base_distance = 1.5 * robot_scale
lookat_height = 0.4 * robot_scale

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
        # Gripper camera: first-person view from the end-effector
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

# Print banner with teleop info
leader_name = get_robot_name(LeaderType)
scale_factor = round(workspace_scale(teleop_ctx), digits = 2)

println("\n" * "=" ^ 70)
println("Fanuc Industrial Robot Simulation - $(uppercase(robot))")
println("=" ^ 70)
println("\nWebSocket Endpoints (all on port 8080):")
println("  Control:       ws://localhost:8080/fanuc/$(robot)/control")
println("  Front camera:  ws://localhost:8080/fanuc/$(robot)/cameras/front")
println("  Side camera:   ws://localhost:8080/fanuc/$(robot)/cameras/side")
println("  Orbit camera:  ws://localhost:8080/fanuc/$(robot)/cameras/orbit")
println("  Gripper cam:   ws://localhost:8080/fanuc/$(robot)/cameras/gripper")
println("\nTeleoperation:")
println("  Leader type:       $(leader_name) ($(LeaderType))")
println("  Mapping strategy:  $(typeof(teleop_ctx.strategy))")
println("  Workspace scale:   $(scale_factor)x ($(leader_name) -> Fanuc)")
println("\nJoint Commands ($(leader_name) format):")
println("  shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper")
println("=" ^ 70)
println("\nPress 'Space' to pause/unpause, 'F1' for help, close window to exit\n")

run_with_capture!(model, data,
    controller = ctrl!,
    capture = capture_config
)

println("Simulation ended.")
