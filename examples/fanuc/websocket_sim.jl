# Fanuc Industrial Robot Simulation with WebSocket Control and Multi-Camera Capture
#
# This script runs a MuJoCo simulation of Fanuc industrial robot arms with:
# - WebSocket server for external joint control (port 8081)
# - Multi-camera capture (WebSocket streams)
# - Interactive 3D visualization
# - Support for 19 robot variants (5-12 DOF)
#
# IK-BASED CONTROL: Accepts SO101 joint commands and uses inverse kinematics to
# map end-effector positions between robots. This provides natural motion mapping
# despite different robot kinematics (joint axes, link lengths, etc.).
#
# The mapping works as follows:
# 1. Apply SO101 joint angles to a "shadow" SO101 model
# 2. Compute SO101 gripper position via forward kinematics
# 3. Scale position from SO101 workspace to Fanuc workspace
# 4. Solve inverse kinematics to get Fanuc joint angles
# 5. Apply to Fanuc actuators
#
# Usage:
#   julia --project=. -t 4 examples/fanuc/websocket_sim.jl [robot_name]
#
# Examples:
#   julia --project=. -t 4 examples/fanuc/websocket_sim.jl           # Default: m10ia
#   julia --project=. -t 4 examples/fanuc/websocket_sim.jl crx10ial  # CRX collaborative robot
#   julia --project=. -t 4 examples/fanuc/websocket_sim.jl m900ib700 # Large 12-DOF robot
#
# Connect with test client:
#   julia --project=. examples/clients/ws_client.jl
#
# WebSocket API:
#   Commands (SO101 format - mapped via IK):
#     {"command": "set_joints_state", "joints": {"shoulder_pan": 45.0, "shoulder_lift": -30.0, ...}}
#   All values in degrees.
#
#   Events:
#     {"event": "state_was_updated", "timestamp": ..., "state": {"shoulder_pan": 45.0, ...}}

using MuJoCo
using MuJoCo.LibMuJoCo

# Load shared modules
include("../../src/SceneBuilder.jl")
include("../../src/WebSocketServer.jl")
include("../../src/capture/Capture.jl")
include("../../src/Kinematics.jl")

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
robot = if length(ARGS) > 0
    ARGS[1]
else
    DEFAULT_ROBOT
end

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
joint_names = String[]
for i in 0:(model.njnt - 1)
    name = unsafe_string(mj_id2name(model, Int32(LibMuJoCo.mjOBJ_JOINT), i))
    if !isempty(name) && startswith(name, "joint_")
        push!(joint_names, name)
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

println("Joints: ", join(joint_names, ", "))

# --- IK-Based Cross-Robot Control ---
# Instead of direct joint mapping (which doesn't work well due to different kinematics),
# we use inverse kinematics to map end-effector positions between robots:
#
# 1. Load SO101 as a "shadow" model for forward kinematics
# 2. When SO101 joint commands arrive:
#    a. Apply joints to SO101 shadow model
#    b. Compute SO101 gripper position via FK
#    c. Scale position from SO101 workspace to Fanuc workspace
#    d. Solve IK to get Fanuc joint angles
#    e. Apply to Fanuc actuators
#
# This provides natural motion mapping despite different robot kinematics.

# Load SO101 shadow model for FK (just the robot, no scene objects)
const SO101_XML = joinpath(@__DIR__, "..", "..", "robots", "SO-ARM100",
    "Simulation", "SO101", "so101_new_calib.xml")
println("Loading SO101 shadow model from: $SO101_XML")

const so101_model = load_model(SO101_XML)
const so101_data = init_data(so101_model)

# SO101 joint name -> qpos index mapping
const SO101_JOINTS = Dict{String, Int}(
    "shoulder_pan" => 1,
    "shoulder_lift" => 2,
    "elbow_flex" => 3,
    "wrist_flex" => 4,
    "wrist_roll" => 5,
    "gripper" => 6
)

# Compute home positions for workspace scaling
const SO101_HOME = get_home_position(so101_model, so101_data, "gripper")
const FANUC_HOME = get_home_position(model, data, "link_6")
const WORKSPACE_SCALE = norm(FANUC_HOME) / norm(SO101_HOME)

println("SO101 home position: $SO101_HOME ($(round(norm(SO101_HOME), digits=3))m from origin)")
println("Fanuc home position: $FANUC_HOME ($(round(norm(FANUC_HOME), digits=3))m from origin)")
println("Workspace scale factor: $(round(WORKSPACE_SCALE, digits=2))x")

# IK configuration - tuned for real-time control
const IK_CONFIG = IKConfig(
    max_iter = 50,      # Fewer iterations for real-time (30fps)
    tol = 0.005,        # 5mm tolerance
    step_size = 0.5,
    damping = 0.01
)

# Fanuc joint name -> SO101 name for state reporting
const FANUC_TO_SO101_MAP = Dict{String, String}(
    "joint_1" => "shoulder_pan",
    "joint_2" => "shoulder_lift",
    "joint_3" => "elbow_flex",
    "joint_4" => "wrist_flex",
    "joint_5" => "wrist_flex2",   # Extra DOF, reported with unique name
    "joint_6" => "wrist_roll"
)

# --- WebSocket Server ---
server = WebSocketControlServer(port = 8081, fps = 30.0)

# Robot-specific: how to get joint state (in degrees)
# Reports using SO101 joint names for compatibility, plus any unmapped Fanuc joints
function get_joint_state(model, data)
    state = Dict{String, Float64}()
    for name in joint_names
        joint_id = mj_name2id(model, Int32(LibMuJoCo.mjOBJ_JOINT), name)
        if joint_id >= 0
            addr = model.jnt_qposadr[joint_id + 1] + 1
            val_deg = rad2deg(data.qpos[addr])

            # Use SO101 name if available, otherwise use Fanuc name
            report_name = get(FANUC_TO_SO101_MAP, name, name)
            state[report_name] = val_deg
        end
    end
    return state
end

# Robot-specific: how to apply joint commands using IK-based mapping
# 1. Apply SO101 joints to shadow model
# 2. Compute SO101 gripper position via FK
# 3. Scale to Fanuc workspace
# 4. Solve IK for Fanuc joint angles
# 5. Apply to Fanuc actuators
function apply_joint_command!(fanuc_data, actuator_map_unused, joints)
    # Step 1: Apply joints to SO101 shadow model (degrees -> radians)
    for (name, val) in joints
        name_str = String(name)
        if haskey(SO101_JOINTS, name_str)
            idx = SO101_JOINTS[name_str]
            so101_data.qpos[idx] = deg2rad(Float64(val))
        end
    end

    # Step 2: Compute SO101 gripper position via FK
    so101_pos = forward_kinematics(so101_model, so101_data, "gripper")

    # Step 3: Scale position to Fanuc workspace
    target_pos = scale_position(so101_pos, SO101_HOME, FANUC_HOME)

    # Step 4: Solve IK for Fanuc joint angles
    # Note: We work on a copy of qpos to avoid corrupting the Fanuc simulation state
    # during IK iterations. After IK converges, we apply to ctrl.
    ik_data = init_data(model)
    ik_data.qpos .= fanuc_data.qpos  # Start from current position for continuity

    result = inverse_kinematics!(model, ik_data, "link_6", target_pos; config = IK_CONFIG)

    # Step 5: Apply computed joint angles to Fanuc actuators
    # Use the first 6 joints (or however many the robot has)
    num_joints = min(6, model.nu)
    for i in 1:num_joints
        fanuc_data.ctrl[i] = ik_data.qpos[i]
    end

    # Debug output (occasionally)
    if rand() < 0.01  # ~1% of the time
        @info "IK mapping" so101_pos=round.(so101_pos, digits = 3) target_pos=round.(
            target_pos, digits = 3) converged=result.success error_mm=round(
            result.final_error * 1000, digits = 1) iters=result.iterations
    end
end

# Start WebSocket server
start_server!(server, get_joint_state)

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
            output = WebSocketOutput(port = 8082)
        ),
        # Side camera: external view from the side
        CameraSpec(
            name = "side",
            lookat = [0.0, 0.0, lookat_height],
            distance = base_distance,
            azimuth = 90.0,
            elevation = -20.0,
            output = WebSocketOutput(port = 8083)
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
            output = WebSocketOutput(port = 8084)
        ),
        # Gripper camera: first-person view from the end-effector
        CameraSpec(
            name = "gripper",
            mode = :fixed,
            model_camera = "gripper_cam",
            output = WebSocketOutput(port = 8085)
        )
    ]
)

# --- Run Visualization ---
println("\nInitializing visualizer...")
init_visualiser()

println("Starting Fanuc $robot simulation with IK-based control...")
println("WebSocket control:  ws://localhost:8081")
println("Camera streams:     ws://localhost:8082 (front), 8083 (side), 8084 (orbit), 8085 (gripper)")
println()
println("IK-Based Mapping: SO101 joint commands → FK → scale → IK → Fanuc joints")
println("Accepts SO101 joints: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper")
println("Workspace scaling: $(round(WORKSPACE_SCALE, digits=2))x (SO101 → Fanuc)")
println()
println("Press 'Space' to pause/unpause, 'F1' for help, close window to exit\n")

run_with_capture!(model, data,
    controller = ctrl!,
    capture = capture_config
)

println("Simulation ended.")
