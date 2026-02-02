# LeKiwi Mobile Robot Simulation with WebSocket Control and Multi-Camera Capture
#
# This script runs a MuJoCo simulation of the LeKiwi mobile manipulator with:
# - Unified WebSocket server on port 8080 with path-based routing
# - Keyboard control for mobile base (WASD + QE for strafing, Shift for speed boost)
# - WebSocket base velocity commands (set_base_velocity)
# - Multi-camera capture (WebSocket streams)
# - Built-in front and wrist cameras
# - Graspable cubes in the environment
# - Interactive 3D visualization
# - TeleoperatorMapping system for cross-robot compatibility
#
# The arm accepts joint names based on the --leader flag:
#   --leader=so101  (default): shoulder_pan, shoulder_lift, elbow_flex, etc.
#   --leader=lekiwi: Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw
#
# Keyboard Controls (mobile base):
#   W/S - Forward/backward
#   A/D - Rotate left/right
#   Q/E - Strafe left/right
#   Shift - Double speed
#
# Usage:
#   julia --project=. -t 4 examples/lekiwi/websocket_sim.jl
#   julia --project=. -t 4 examples/lekiwi/websocket_sim.jl --leader=lekiwi
#
# Connect with test client:
#   julia --project=. examples/clients/ws_client.jl

using MuJoCo
using MuJoCo.LibMuJoCo

# Load shared modules
include("../../src/SceneBuilder.jl")
include("../../src/UnifiedWebSocketServer.jl")
include("../../src/BaseController.jl")
include("../../src/capture/Capture.jl")
include("../../src/RobotTypes.jl")
include("../../src/Kinematics.jl")
include("../../src/TeleoperatorMapping.jl")
include("../../src/CLIUtils.jl")

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

# --- Teleoperation Configuration ---
const DefaultLeaderType = parse_leader_type(ARGS; default = SO101)  # Default SO101 for compat
const FollowerType = LeKiwiArm
const project_root = joinpath(@__DIR__, "..", "..")

# Create a context factory for per-client teleop contexts
# This allows each WebSocket client to have their own leader type
function create_client_context(leader_type)
    # Handle both Type and String inputs (query param comes as string)
    resolved_type = if leader_type isa Type
        leader_type
    elseif leader_type isa AbstractString
        get(ROBOT_TYPE_MAP, lowercase(leader_type), DefaultLeaderType)
    else
        DefaultLeaderType
    end

    ctx = create_teleop_context(resolved_type, FollowerType, model, data;
        project_root = project_root)
    println("Created teleop context: $(resolved_type) → $(FollowerType)")
    return ctx
end

# Create default context for legacy/fallback use
const default_teleop_ctx = create_teleop_context(
    DefaultLeaderType, FollowerType, model, data;
    project_root = project_root)
println(describe(default_teleop_ctx))

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

# Wheel joint names and their corresponding state names for broadcasting
const WHEEL_JOINT_NAMES = [
    "base_left_wheel_joint",
    "base_right_wheel_joint",
    "base_back_wheel_joint"
]
const WHEEL_STATE_NAMES = [
    "left_wheel_velocity",
    "right_wheel_velocity",
    "back_wheel_velocity"
]

# Build wheel joint -> qvel index map (populated after model is loaded)
const wheel_qvel_indices = Dict{String, Int}()
for joint_name in WHEEL_JOINT_NAMES
    joint_id = mj_name2id(model, Int32(LibMuJoCo.mjOBJ_JOINT), joint_name)
    if joint_id >= 0
        # jnt_dofadr is 0-indexed in MuJoCo, +1 for Julia array access
        qvel_addr = model.jnt_dofadr[joint_id + 1]
        wheel_qvel_indices[joint_name] = qvel_addr + 1  # Store as 1-indexed for Julia
    end
end

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
# WebSocket Server (Unified - single port with path-based routing)
# ============================================================================
server = UnifiedServer(port = 8080, robot = "lekiwi", fps = 30.0)

# Enable per-client leader types via ?leader=X query parameter
set_context_factory!(server, create_client_context, DefaultLeaderType)

# ============================================================================
# Base Velocity Controller
# ============================================================================
# Handles keyboard and WebSocket input for mobile base control
base_ctrl = BaseVelocityController(
    max_vx = 0.15,      # Max forward/back speed (m/s)
    max_vy = 0.10,      # Max strafe speed (m/s)
    max_omega = 0.8,    # Max angular velocity (rad/s)
    timeout = 0.5       # Seconds before velocities reset to 0
)

# Get joint state: arm joints (leader-compatible names) + wheel velocities (rad/s)
# Supports per-client context for different leader types
function get_joint_state(model, data, ctx = nothing)
    # Use provided context or fall back to default
    teleop_ctx = ctx !== nothing ? ctx : default_teleop_ctx

    # Get arm state in leader-compatible names
    state = get_state_for_leader(teleop_ctx, model, data)

    # Add wheel velocities (rad/s) from MuJoCo simulation
    for (i, joint_name) in enumerate(WHEEL_JOINT_NAMES)
        if haskey(wheel_qvel_indices, joint_name)
            qvel_idx = wheel_qvel_indices[joint_name]
            velocity = data.qvel[qvel_idx]  # rad/s
            state[WHEEL_STATE_NAMES[i]] = velocity
        end
    end

    return state
end

# Apply joint commands (degrees -> radians) with teleop mapping
# Supports per-client context for different leader types
function apply_joint_command!(data, actuator_map, joints, ctx = nothing)
    # Use provided context or fall back to default
    teleop_ctx = ctx !== nothing ? ctx : default_teleop_ctx

    joints_float = Dict{String, Float64}(String(k) => Float64(v) for (k, v) in joints)
    mapped_joints = map_joints(teleop_ctx, joints_float, model, data)

    for (name, val) in mapped_joints
        if haskey(actuator_map, name)
            idx = actuator_map[name]
            data.ctrl[idx] = deg2rad(val)
        end
    end
end

# Handle base velocity commands from WebSocket
function handle_base_velocity_command!(raw::Dict)
    if get(raw, "command", "") == "set_base_velocity"
        vx = Float64(get(raw, "vx", 0.0))
        vy = Float64(get(raw, "vy", 0.0))
        omega = Float64(get(raw, "omega", 0.0))
        update_from_websocket!(base_ctrl, vx, vy, omega)
        return true
    end
    return false
end

# Start WebSocket server
start!(server, get_joint_state)

# ============================================================================
# Controller Function
# ============================================================================
function ctrl!(m, d)
    # === Mobile Base: Get velocities from controller (keyboard or WebSocket) ===
    vx, vy, omega = get_velocities(base_ctrl)

    wheel_vels = body_to_wheel_velocities(vx, vy, omega)
    d.ctrl[1] = wheel_vels[1]  # left wheel
    d.ctrl[2] = wheel_vels[2]  # right wheel
    d.ctrl[3] = wheel_vels[3]  # back wheel

    # === WebSocket Control ===
    # Process commands from channel, handling base velocity commands specially
    while isready(server.control_channel)
        raw = take!(server.control_channel)

        # Check for base velocity command first
        if handle_base_velocity_command!(raw)
            continue
        end

        # Otherwise handle as arm joint command
        cmd = get(raw, "command", "")
        if cmd == "set_joints_state"
            joints = get(raw, "joints", Dict())
            if !isempty(joints)
                # Get per-client context if available
                ws = get(raw, "_ws", nothing)
                client_ctx = ws !== nothing ? get_client_context(server, ws) : nothing
                apply_joint_command!(d, actuator_map, joints, client_ctx)
            end
        end
    end

    maybe_broadcast!(server, m, d, get_joint_state)
end

# ============================================================================
# Keyboard Handler
# ============================================================================
# Called from UI loop after GLFW.PollEvents() to poll WASD keys
function keyboard_handler(window, GLFW)
    update_from_keyboard!(base_ctrl, window, GLFW)
end

# ============================================================================
# Camera Configuration
# ============================================================================
# Multi-camera capture with WebSocket streaming
# Uses built-in model cameras (front, wrist) plus external side views
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
            output = WebSocketOutput(server = server)
        ),
        # Wrist camera: built-in model camera on the gripper
        CameraSpec(
            name = "wrist",
            mode = :fixed,
            model_camera = "wrist",
            output = WebSocketOutput(server = server)
        ),
        # Left side camera: external view from the left
        CameraSpec(
            name = "side_left",
            lookat = [0.0, 0.0, 0.15],
            distance = 1.0,
            azimuth = 90.0,
            elevation = -20.0,
            output = WebSocketOutput(server = server)
        ),
        # Right side camera: external view from the right
        CameraSpec(
            name = "side_right",
            lookat = [0.0, 0.0, 0.15],
            distance = 1.0,
            azimuth = -90.0,
            elevation = -20.0,
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

# ============================================================================
# Run Visualization
# ============================================================================
println("\nInitializing visualizer...")
init_visualiser()

print_teleop_banner(DefaultLeaderType, FollowerType, default_teleop_ctx.strategy)
println("\nPer-client leader type support:")
println("  Default (CLI):     --leader=so101")
println("  Per-connection:    ws://...?leader=so101")
println("                     ws://...?leader=lekiwi")
println("                     ws://...?leader=trossen")

println("\n" * "=" ^ 60)
println("LeKiwi Mobile Manipulator Simulation")
println("=" ^ 60)
println("\nWebSocket Endpoints (all on port 8080):")
println("  Control:           ws://localhost:8080/lekiwi/control")
println("  Control (custom):  ws://localhost:8080/lekiwi/control?leader=<type>")
println("  Front camera:      ws://localhost:8080/lekiwi/cameras/front")
println("  Wrist camera:      ws://localhost:8080/lekiwi/cameras/wrist")
println("  Left side camera:  ws://localhost:8080/lekiwi/cameras/side_left")
println("  Right side camera: ws://localhost:8080/lekiwi/cameras/side_right")
println("\nBase Control (keyboard):")
println("  W/S - Forward/backward")
println("  A/D - Rotate left/right")
println("  Q/E - Strafe left/right")
println("  Shift - Double speed")
println("\nBase Control (WebSocket):")
println("  {\"command\": \"set_base_velocity\", \"vx\": 0.1, \"vy\": 0.0, \"omega\": 0.3}")
println("=" ^ 60)
println("\nPress 'Space' to pause/unpause, 'F1' for help, close window to exit\n")

run_with_capture!(model, data,
    controller = ctrl!,
    capture = capture_config,
    keyboard_handler = keyboard_handler,
    disabled_keys = Set(['W', 'A', 'S', 'D', 'Q', 'E'])
)

println("Simulation ended.")
