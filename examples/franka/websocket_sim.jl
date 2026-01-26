# Franka Panda Robot Simulation with WebSocket Control and Multi-Camera Capture
#
# This script runs a MuJoCo simulation of the Franka Panda 7-DOF robot arm with:
# - WebSocket server for external joint control (port 8081)
# - Multi-camera capture (WebSocket streams)
# - Gripper-mounted camera for first-person view
# - Graspable cubes in the environment
# - Interactive 3D visualization
#
# Uses the MuJoCo Menagerie Franka Emika Panda model.
#
# IK-BASED CONTROL: Accepts SO101 joint commands and uses inverse kinematics to
# map end-effector positions between robots. This provides natural motion mapping
# despite different robot kinematics (7-DOF Franka vs 5-DOF SO101).
#
# The mapping works as follows:
# 1. Apply SO101 joint angles to a "shadow" SO101 model
# 2. Compute SO101 gripper position via forward kinematics
# 3. Scale position from SO101 workspace to Franka workspace
# 4. Solve inverse kinematics to get Franka joint angles
# 5. Apply to Franka actuators
#
# Gripper control: The Franka has 2 finger joints controlled symmetrically via
# a tendon. Send a single "gripper" command (in degrees, SO101 format) and both
# fingers move together.
#
# Usage:
#   julia --project=. -t 4 examples/franka/websocket_sim.jl
#
# Connect with test client:
#   julia --project=. examples/clients/ws_client.jl
#
# WebSocket API:
#   Commands (SO101 format - mapped via IK):
#     {"command": "set_joints_state", "joints": {"shoulder_pan": 45.0, "gripper": 50.0, ...}}
#   All joint values in degrees, gripper in degrees (-10 to 100).
#
#   Events:
#     {"event": "state_was_updated", "timestamp": ..., "state": {"shoulder_pan": 45.0, ...}}
#   Reports all 7 arm joints with unique names plus gripper in degrees.

using MuJoCo
using MuJoCo.LibMuJoCo

# Load shared modules
include("../../src/SceneBuilder.jl")
include("../../src/WebSocketServer.jl")
include("../../src/capture/Capture.jl")
include("../../src/Kinematics.jl")

# =============================================================================
# Configuration
# =============================================================================

# End-effector body for IK targeting
# Options: "link7" (wrist flange) or "hand" (gripper base)
const EE_BODY = "link7"

# Gripper parameters
# Menagerie model: actuator8 uses 0-255 range mapped to 0-0.04m
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
# Actuator Mapping
# =============================================================================
# Menagerie Franka actuators:
#   1-7: actuator1 through actuator7 (arm, position control)
#   8: actuator8 (gripper via tendon, 0-255 range)

actuator_names = [unsafe_string(mj_id2name(model, Int32(LibMuJoCo.mjOBJ_ACTUATOR), i - 1))
                  for i in 1:(model.nu)]
actuator_map = Dict(name => i for (i, name) in enumerate(actuator_names))
println("Actuators: ", join(actuator_names, ", "))

# Arm joint names (for state reporting)
const FRANKA_ARM_JOINTS = [
    "joint1", "joint2", "joint3", "joint4",
    "joint5", "joint6", "joint7"
]

# =============================================================================
# Gripper Mapping Functions
# =============================================================================

"""
    map_gripper_so101_to_franka(so101_deg::Float64) -> Float64

Convert SO101 gripper position (degrees) to Franka actuator units (0-255).

SO101 gripper: -10° (closed) to 100° (open)
Franka gripper: 0 (closed) to 255 (open)
"""
function map_gripper_so101_to_franka(so101_deg::Float64)
    normalized = clamp(
        (so101_deg - SO101_GRIPPER_MIN) /
        (SO101_GRIPPER_MAX - SO101_GRIPPER_MIN), 0.0, 1.0)
    return FRANKA_GRIPPER_MIN + normalized * (FRANKA_GRIPPER_MAX - FRANKA_GRIPPER_MIN)
end

"""
    map_gripper_franka_to_so101(franka_pos::Float64) -> Float64

Convert Franka finger position (meters) to SO101 gripper position (degrees).

Franka finger position: 0m (closed) to 0.04m (open)
SO101 gripper: -10° (closed) to 100° (open)
"""
function map_gripper_franka_to_so101(franka_pos::Float64)
    # Finger joints have 0-0.04m range
    normalized = clamp(franka_pos / 0.04, 0.0, 1.0)
    return SO101_GRIPPER_MIN + normalized * (SO101_GRIPPER_MAX - SO101_GRIPPER_MIN)
end

# =============================================================================
# IK-Based Cross-Robot Control
# =============================================================================
# Load SO101 shadow model for forward kinematics

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
const FRANKA_HOME = get_home_position(model, data, EE_BODY)
const WORKSPACE_SCALE = norm(FRANKA_HOME) / norm(SO101_HOME)

println("SO101 home position: $(round.(SO101_HOME, digits=3)) ($(round(norm(SO101_HOME), digits=3))m)")
println("Franka home position: $(round.(FRANKA_HOME, digits=3)) ($(round(norm(FRANKA_HOME), digits=3))m)")
println("Workspace scale factor: $(round(WORKSPACE_SCALE, digits=2))x")

# IK configuration - tuned for real-time control
const IK_CONFIG = IKConfig(
    max_iter = 50,      # Fewer iterations for real-time (30fps)
    tol = 0.005,        # 5mm tolerance
    step_size = 0.5,
    damping = 0.01
)

# Franka joint -> SO101 name mapping for state reporting
# Franka has 7 arm joints; SO101 has 5. Extra DOFs get unique names.
const FRANKA_TO_SO101_MAP = Dict{String, String}(
    "joint1" => "shoulder_pan",
    "joint2" => "shoulder_lift",
    "joint3" => "elbow_flex",
    "joint4" => "wrist_flex",
    "joint5" => "wrist_roll",
    "joint6" => "wrist_yaw",      # Extra DOF - unique name
    "joint7" => "wrist_twist"     # Extra DOF - unique name
)

# =============================================================================
# WebSocket Server
# =============================================================================
server = WebSocketControlServer(port = 8081, fps = 30.0)

# Get joint state in degrees, using SO101-compatible names where possible
function get_joint_state(model, data)
    state = Dict{String, Float64}()

    # Report arm joints
    for franka_name in FRANKA_ARM_JOINTS
        joint_id = mj_name2id(model, Int32(LibMuJoCo.mjOBJ_JOINT), franka_name)
        if joint_id >= 0
            addr = model.jnt_qposadr[joint_id + 1] + 1
            val_deg = rad2deg(data.qpos[addr])

            # Use SO101-compatible name
            report_name = get(FRANKA_TO_SO101_MAP, franka_name, franka_name)
            state[report_name] = val_deg
        end
    end

    # Report gripper (average of both fingers, converted to SO101 degrees)
    finger1_id = mj_name2id(model, Int32(LibMuJoCo.mjOBJ_JOINT), "finger_joint1")
    finger2_id = mj_name2id(model, Int32(LibMuJoCo.mjOBJ_JOINT), "finger_joint2")
    if finger1_id >= 0 && finger2_id >= 0
        addr1 = model.jnt_qposadr[finger1_id + 1] + 1
        addr2 = model.jnt_qposadr[finger2_id + 1] + 1
        avg_pos = (data.qpos[addr1] + data.qpos[addr2]) / 2.0
        state["gripper"] = map_gripper_franka_to_so101(avg_pos)
    end

    return state
end

# Apply joint commands using IK-based mapping
function apply_joint_command!(franka_data, actuator_map_unused, joints)
    gripper_cmd = nothing

    # Step 1: Apply joints to SO101 shadow model (degrees -> radians)
    for (name, val) in joints
        name_str = String(name)

        # Handle gripper separately
        if name_str == "gripper"
            gripper_cmd = Float64(val)
            continue
        end

        # Apply to SO101 shadow model for IK
        if haskey(SO101_JOINTS, name_str)
            idx = SO101_JOINTS[name_str]
            so101_data.qpos[idx] = deg2rad(Float64(val))
        end
    end

    # Step 2: Compute SO101 gripper position via FK
    so101_pos = forward_kinematics(so101_model, so101_data, "gripper")

    # Step 3: Scale position to Franka workspace
    target_pos = scale_position(so101_pos, SO101_HOME, FRANKA_HOME)

    # Step 4: Solve IK for Franka joint angles
    ik_data = init_data(model)
    ik_data.qpos .= franka_data.qpos  # Start from current position for continuity

    result = inverse_kinematics!(model, ik_data, EE_BODY, target_pos; config = IK_CONFIG)

    # Step 5: Apply computed joint angles to Franka arm actuators (first 7)
    for i in 1:7
        franka_data.ctrl[i] = ik_data.qpos[i]
    end

    # Step 6: Apply gripper command (via tendon actuator)
    if gripper_cmd !== nothing
        grip_val = map_gripper_so101_to_franka(gripper_cmd)
        franka_data.ctrl[8] = grip_val  # actuator8 controls gripper tendon
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
        # Gripper camera: added to hand body (matches SO101/Trossen port assignment)
        CameraSpec(
            name = "gripper",
            mode = :fixed,
            model_camera = "gripper_cam",
            output = WebSocketOutput(port = 8085)
        ),
        # Wrist camera: added to link7 (Franka-specific extra camera)
        CameraSpec(
            name = "wrist",
            mode = :fixed,
            model_camera = "wrist_cam",
            output = WebSocketOutput(port = 8086)
        )
    ]
)

# =============================================================================
# Run Visualization
# =============================================================================
println("\nInitializing visualizer...")
init_visualiser()

println("\n" * "="^70)
println("Franka Panda Simulation with IK-Based Control")
println("="^70)
println("\nWebSocket Endpoints:")
println("  Control:        ws://localhost:8081")
println("  Front camera:   ws://localhost:8082")
println("  Side camera:    ws://localhost:8083")
println("  Orbit camera:   ws://localhost:8084")
println("  Gripper camera: ws://localhost:8085")
println("  Wrist camera:   ws://localhost:8086")
println("\nIK-Based Mapping:")
println("  SO101 joints → FK → scale $(round(WORKSPACE_SCALE, digits=2))x → IK → Franka joints")
println("\nAccepts SO101 joints:")
println("  shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper")
println("\nReports Franka joints (7 DOF + gripper):")
println("  shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll,")
println("  wrist_yaw, wrist_twist, gripper")
println("="^70)
println("\nPress 'Space' to pause/unpause, 'F1' for help, close window to exit\n")

run_with_capture!(model, data,
    controller = ctrl!,
    capture = capture_config
)

println("Simulation ended.")
