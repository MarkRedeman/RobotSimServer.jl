# LeKiwi Mobile Robot Simulation with WebSocket Control and Multi-Camera Capture
#
# This script runs a MuJoCo simulation of the LeKiwi mobile manipulator with:
# - WebSocket server for external arm control (port 8081)
# - Autonomous mobile base driving in circles
# - Multi-camera capture (WebSocket streams)
# - Built-in front and wrist cameras
# - Graspable cubes in the environment
# - Interactive 3D visualization
#
# The arm accepts SO101-compatible joint names for cross-robot compatibility:
#   shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
#
# These are mapped internally to LeKiwi's native names:
#   Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw
#
# Usage:
#   julia --project=. -t 4 examples/lekiwi/websocket_sim.jl
#
# Connect with test client:
#   julia --project=. examples/clients/ws_client.jl

using MuJoCo
using MuJoCo.LibMuJoCo

# Load shared modules
include("../../src/SceneBuilder.jl")
include("../../src/WebSocketServer.jl")
include("../../src/capture/Capture.jl")

# ============================================================================
# Scene Setup
# ============================================================================
xml_path = joinpath(@__DIR__, "scene.xml")
println("Loading model from: $xml_path")

# Generate cubes around the robot, within arm reach
# LeKiwi arm has similar reach to SO101 (~0.35m)
cubes = generate_cubes(50, radius_min = 0.15, radius_max = 0.40, z = 0.08)

# Build scene with cubes (cameras are already in the XML)
model, data = build_scene(xml_path, cubes)

# ============================================================================
# Actuator and Joint Mapping
# ============================================================================
# LeKiwi actuators:
#   1: base_left_wheel (velocity)   - autonomous control
#   2: base_right_wheel (velocity)  - autonomous control
#   3: base_back_wheel (velocity)   - autonomous control
#   4: Rotation (position)          - WebSocket control
#   5: Pitch (position)             - WebSocket control
#   6: Elbow (position)             - WebSocket control
#   7: Wrist_Pitch (position)       - WebSocket control
#   8: Wrist_Roll (position)        - WebSocket control
#   9: Jaw (position)               - WebSocket control

actuator_names = [unsafe_string(mj_id2name(model, Int32(LibMuJoCo.mjOBJ_ACTUATOR), i - 1))
                  for i in 1:(model.nu)]
actuator_map = Dict(name => i for (i, name) in enumerate(actuator_names))
println("Available actuators: ", actuator_names)

# Arm actuator names (native LeKiwi names)
const ARM_ACTUATORS = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]

# Joint name mapping: SO101-compatible names -> LeKiwi native names
# This allows clients to use the same joint names across different robots
const JOINT_NAME_MAP = Dict(
    # SO101-compatible names -> LeKiwi native names
    "shoulder_pan" => "Rotation",
    "shoulder_lift" => "Pitch",
    "elbow_flex" => "Elbow",
    "wrist_flex" => "Wrist_Pitch",
    "wrist_roll" => "Wrist_Roll",
    "gripper" => "Jaw",
    # Also accept native LeKiwi names directly
    "Rotation" => "Rotation",
    "Pitch" => "Pitch",
    "Elbow" => "Elbow",
    "Wrist_Pitch" => "Wrist_Pitch",
    "Wrist_Roll" => "Wrist_Roll",
    "Jaw" => "Jaw"
)

# Reverse mapping for state broadcast (native -> SO101-compatible)
const JOINT_NAME_REVERSE_MAP = Dict(
    "Rotation" => "shoulder_pan",
    "Pitch" => "shoulder_lift",
    "Elbow" => "elbow_flex",
    "Wrist_Pitch" => "wrist_flex",
    "Wrist_Roll" => "wrist_roll",
    "Jaw" => "gripper"
)

# ============================================================================
# Omniwheel Kinematics
# ============================================================================
# LeKiwi uses 3 omniwheels at 120° spacing for holonomic motion.
# Given desired body velocity (vx, vy, ω), compute wheel velocities:
#
#   wheel_vel[i] = (1/r) * (-sin(α[i])*vx + cos(α[i])*vy + R*ω)
#
# Where:
#   r = 0.05 m (wheel radius)
#   R = 0.125 m (robot base radius)
#   α = [π/2, π/2 + 2π/3, π/2 + 4π/3] (wheel angles: left, right, back)

const WHEEL_RADIUS = 0.05       # meters
const BASE_RADIUS = 0.125       # meters
const WHEEL_ANGLES = [π / 2, π / 2 + 2π / 3, π / 2 + 4π / 3]

"""
Compute wheel velocities for desired body velocity (vx, vy, omega).
Returns (left, right, back) wheel velocities in rad/s.
"""
function body_to_wheel_velocities(vx::Float64, vy::Float64, omega::Float64)
    wheel_vels = zeros(3)
    for i in 1:3
        α = WHEEL_ANGLES[i]
        wheel_vels[i] = (1 / WHEEL_RADIUS) * (
            -sin(α) * vx + cos(α) * vy + BASE_RADIUS * omega
        )
    end
    return wheel_vels
end

# ============================================================================
# WebSocket Server
# ============================================================================
server = WebSocketControlServer(port = 8081, fps = 30.0)

# Get arm joint state (in degrees) using SO101-compatible names
function get_joint_state(model, data)
    state = Dict{String, Float64}()
    for native_name in ARM_ACTUATORS
        joint_id = mj_name2id(model, Int32(LibMuJoCo.mjOBJ_JOINT), native_name)
        if joint_id >= 0
            addr = model.jnt_qposadr[joint_id + 1] + 1
            # Use SO101-compatible name in the output
            compatible_name = JOINT_NAME_REVERSE_MAP[native_name]
            state[compatible_name] = rad2deg(data.qpos[addr])
        end
    end
    return state
end

# Apply joint commands (degrees -> radians) with name mapping
function apply_joint_command!(data, actuator_map, joints)
    for (name, val) in joints
        name_str = String(name)

        # Map to native name if using SO101-compatible name
        native_name = get(JOINT_NAME_MAP, name_str, nothing)
        if native_name === nothing
            @warn "Unknown joint name: $name_str"
            continue
        end

        if haskey(actuator_map, native_name)
            idx = actuator_map[native_name]
            data.ctrl[idx] = deg2rad(Float64(val))
        else
            @warn "Actuator not found: $native_name"
        end
    end
end

# Start WebSocket server
start_server!(server, get_joint_state)

# ============================================================================
# Controller Function
# ============================================================================
function ctrl!(m, d)
    t = d.time

    # === Autonomous Mobile Base: Drive in circles ===
    vx = 0.05           # Forward velocity (m/s)
    vy = 0.0            # Strafe velocity (m/s)
    omega = 0.3         # Angular velocity (rad/s) - turning left

    wheel_vels = body_to_wheel_velocities(vx, vy, omega)
    d.ctrl[1] = wheel_vels[1]  # left wheel
    d.ctrl[2] = wheel_vels[2]  # right wheel
    d.ctrl[3] = wheel_vels[3]  # back wheel

    # === WebSocket Arm Control ===
    process_commands!(server, d, actuator_map, apply_joint_command!)
    maybe_broadcast!(server, m, d, get_joint_state)
end

# ============================================================================
# Camera Configuration
# ============================================================================
# Multi-camera capture with WebSocket streaming
# Uses built-in model cameras (front, wrist) plus an external orbit camera
capture_config = CaptureConfig(
    width = 640,
    height = 480,
    fps = 30.0,
    cameras = [
        # Front camera: built-in model camera on the base
        CameraSpec(
            name = "front",
            mode = :fixed,
            model_camera = "front",
            output = WebSocketOutput(port = 8082)
        ),
        # Wrist camera: built-in model camera on the gripper
        CameraSpec(
            name = "wrist",
            mode = :fixed,
            model_camera = "wrist",
            output = WebSocketOutput(port = 8083)
        ),
        # Orbit camera: external rotating view
        CameraSpec(
            name = "orbit",
            lookat = [0.0, 0.0, 0.15],
            distance = 1.2,
            azimuth = 0.0,
            elevation = -25.0,
            orbiting = true,
            orbit_speed = 20.0,
            output = WebSocketOutput(port = 8084)
        )
    ]
)

# ============================================================================
# Run Visualization
# ============================================================================
println("\nInitializing visualizer...")
init_visualiser()

println("\n" * "="^60)
println("LeKiwi Mobile Manipulator Simulation")
println("="^60)
println("\nWebSocket Endpoints:")
println("  Control: ws://localhost:8081")
println("  Front camera: ws://localhost:8082")
println("  Wrist camera: ws://localhost:8083")
println("  Orbit camera: ws://localhost:8084")
println("\nJoint names (SO101-compatible):")
println("  shoulder_pan, shoulder_lift, elbow_flex,")
println("  wrist_flex, wrist_roll, gripper")
println("\nMobile base: Autonomous circular motion")
println("="^60)
println("\nPress 'Space' to pause/unpause, 'F1' for help, close window to exit\n")

run_with_capture!(model, data,
    controller = ctrl!,
    capture = capture_config
)

println("Simulation ended.")
