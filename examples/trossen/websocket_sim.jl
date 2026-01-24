# Trossen Robot Arm Simulation with WebSocket Control and Multi-Camera Capture
#
# This script runs a MuJoCo simulation of the Trossen WXAI robot arm with:
# - WebSocket server for external joint control (port 8081)
# - Multi-camera capture (WebSocket streams)
# - Gripper-mounted camera for first-person view
# - Graspable cubes in the environment
# - Interactive 3D visualization
#
# COMPATIBILITY: Accepts SO101 joint names and maps them to Trossen joints internally.
# This allows using the same client code for both SO101 and Trossen simulations.
#
# Usage:
#   julia --project=. -t 4 examples/trossen/websocket_sim.jl
#
# Connect with test client:
#   julia --project=. examples/clients/ws_client.jl

using MuJoCo
using MuJoCo.LibMuJoCo

# Load shared modules
include("../../src/SceneBuilder.jl")
include("../../src/WebSocketServer.jl")
include("../../src/capture/Capture.jl")

# --- Scene Setup ---
xml_path = joinpath(@__DIR__, "..", "..", "robots", "trossen_arm_mujoco",
    "trossen_arm_mujoco", "assets", "wxai", "scene.xml")
println("Loading Trossen model from: $xml_path")

# --- SO101 to Trossen Joint Mapping ---
# Maps SO101 joint names to Trossen actuator names and transform functions
# All inputs are in RADIANS (already converted from degrees in the WebSocket handler)

"""
    map_gripper_so101_to_trossen(so101_gripper_rad::Float64) -> Float64

Convert SO101 gripper position (radians, from hinge joint) to Trossen gripper position (meters, slide joint).

SO101 gripper: -10° to 100° (-0.1745 to 1.7453 rad) where -10° = closed, 100° = open
Trossen gripper: 0m to 0.044m where 0m = closed, 0.044m = fully open
"""
function map_gripper_so101_to_trossen(so101_gripper_rad::Float64)
    # Convert radians to degrees for easier reasoning
    so101_deg = rad2deg(so101_gripper_rad)
    # Normalize: -10° → 0.0, 100° → 1.0
    normalized = clamp((so101_deg + 10.0) / 110.0, 0.0, 1.0)
    # Map to Trossen slide range: 0 to 0.044 meters
    return normalized * 0.044
end

"""
    map_shoulder_lift_so101_to_trossen(so101_rad::Float64) -> Float64

Convert SO101 shoulder_lift to Trossen joint_1.

SO101 shoulder_lift: -100° to 100° where 0° = horizontal forward
Trossen joint_1: 0° to 180° where 0° = straight up, 90° = horizontal forward

Mapping: trossen = so101 + π/2
"""
function map_shoulder_lift_so101_to_trossen(so101_rad::Float64)
    return so101_rad + π / 2
end

"""
    map_elbow_flex_so101_to_trossen(so101_rad::Float64) -> Float64

Convert SO101 elbow_flex to Trossen joint_2.

SO101 elbow_flex: -97° to 97° (centered at 0° = straight arm)
Trossen joint_2: 0° to 135° where 0° = arm extended, 135° = fully bent

The axes are inverted and there's an offset.
Mapping: trossen = -so101 + 67.5° (center of Trossen range)
"""
function map_elbow_flex_so101_to_trossen(so101_rad::Float64)
    # Trossen joint_2 range center is ~67.5° (1.178 rad)
    return -so101_rad + deg2rad(67.5)
end

# Mapping table: SO101 joint name → (Trossen actuator name, transform function)
const SO101_TO_TROSSEN_MAP = Dict{String, Tuple{String, Function}}(
    "shoulder_pan" => ("joint_0", identity),                    # Direct mapping (base rotation)
    "shoulder_lift" => ("joint_1", map_shoulder_lift_so101_to_trossen),  # +90° offset
    "elbow_flex" => ("joint_2", map_elbow_flex_so101_to_trossen),     # Inverted + offset
    "wrist_flex" => ("joint_3", identity),                    # Direct mapping
    "wrist_roll" => ("joint_5", identity),                    # Direct (skip joint_4)
    "gripper" => ("left_gripper", map_gripper_so101_to_trossen)   # Angle → distance
)

# Reverse mapping for state reporting: Trossen actuator → SO101 joint name
const TROSSEN_TO_SO101_MAP = Dict{String, Tuple{String, Function}}(
    "joint_0" => ("shoulder_pan", identity),
    "joint_1" => ("shoulder_lift", x -> x - π / 2),
    "joint_2" => ("elbow_flex", x -> -(x - deg2rad(67.5))),
    "joint_3" => ("wrist_flex", identity),
    "joint_5" => ("wrist_roll", identity),
    "left_gripper" => ("gripper", x -> (x / 0.044) * 110.0 - 10.0)  # meters → degrees
)

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

# Also list SO101 joint names we accept
so101_joint_names = collect(keys(SO101_TO_TROSSEN_MAP))
println("Accepting SO101 joints: ", so101_joint_names)

# --- WebSocket Server ---
server = WebSocketControlServer(port = 8081, fps = 30.0)

# Robot-specific: how to get joint state (in degrees, as SO101 joint names)
function get_joint_state(model, data)
    state = Dict{String, Float64}()

    for (trossen_name, (so101_name, inverse_transform)) in TROSSEN_TO_SO101_MAP
        # Get Trossen joint position
        joint_id = mj_name2id(model, Int32(LibMuJoCo.mjOBJ_JOINT), trossen_name)
        if joint_id >= 0
            addr = model.jnt_qposadr[joint_id + 1] + 1
            trossen_val = data.qpos[addr]

            # Apply inverse transform and convert to degrees
            if so101_name == "gripper"
                # Gripper: inverse transform already returns degrees
                state[so101_name] = inverse_transform(trossen_val)
            else
                # Angular joints: apply inverse transform, then convert to degrees
                state[so101_name] = rad2deg(inverse_transform(trossen_val))
            end
        end
    end

    return state
end

# Robot-specific: how to apply joint commands (SO101 names in degrees -> Trossen in radians/meters)
function apply_joint_command!(data, actuator_map, joints)
    for (name, val) in joints
        name_str = String(name)

        # Check if this is an SO101 joint name
        if haskey(SO101_TO_TROSSEN_MAP, name_str)
            trossen_name, transform = SO101_TO_TROSSEN_MAP[name_str]

            if haskey(actuator_map, trossen_name)
                idx = actuator_map[trossen_name]

                if name_str == "gripper"
                    # Gripper: convert degrees to radians first, then apply transform (returns meters)
                    data.ctrl[idx] = transform(deg2rad(Float64(val)))
                else
                    # Angular joints: convert degrees to radians, then apply transform
                    data.ctrl[idx] = transform(deg2rad(Float64(val)))
                end
            else
                @warn "Trossen actuator not found: $trossen_name (mapped from $name_str)"
            end
            # Also accept Trossen joint names directly (for advanced users)
        elseif haskey(actuator_map, name_str)
            idx = actuator_map[name_str]
            # Assume direct control in native units (radians for joints, meters for gripper)
            data.ctrl[idx] = Float64(val)
        else
            @warn "Unknown joint: $name_str (not SO101 or Trossen)"
        end
    end
end

# Start WebSocket server
start_server!(server, get_joint_state)
println("Note: Accepts SO101 joint names, maps internally to Trossen joints")

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
            output = WebSocketOutput(port = 8082)
        ),
        # Side camera: external view from the side
        CameraSpec(
            name = "side",
            lookat = [0.0, 0.0, 0.25],
            distance = 1.4,
            azimuth = 90.0,
            elevation = -20.0,
            output = WebSocketOutput(port = 8083)
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
            output = WebSocketOutput(port = 8084)
        ),
        # Gripper camera: first-person view from inside the gripper
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

println("Starting Trossen simulation with multi-camera capture...")
println("Press 'Space' to pause/unpause, 'F1' for help, close window to exit\n")

run_with_capture!(model, data,
    controller = ctrl!,
    capture = capture_config
)

println("Simulation ended.")
