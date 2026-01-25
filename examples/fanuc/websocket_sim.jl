# Fanuc Industrial Robot Simulation with WebSocket Control and Multi-Camera Capture
#
# This script runs a MuJoCo simulation of Fanuc industrial robot arms with:
# - WebSocket server for external joint control (port 8081)
# - Multi-camera capture (WebSocket streams)
# - Interactive 3D visualization
# - Support for 19 robot variants (5-12 DOF)
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
#   Commands:
#     {"command": "set_joints_state", "joints": {"joint_1": 45.0, "joint_2": -30.0, ...}}
#     All values in degrees.
#
#   Events:
#     {"event": "state_was_updated", "timestamp": ..., "state": {"joint_1": 45.0, ...}}

using MuJoCo
using MuJoCo.LibMuJoCo

# Load shared modules
include("../../src/SceneBuilder.jl")
include("../../src/WebSocketServer.jl")
include("../../src/capture/Capture.jl")

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

# Load model (no scene modifications needed - industrial robots don't have grippers in base config)
model = load_model(xml_path)
data = init_data(model)

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

# --- WebSocket Server ---
server = WebSocketControlServer(port = 8081, fps = 30.0)

# Robot-specific: how to get joint state (in degrees)
# Reports using joint names (joint_1, joint_2, etc.) not actuator names
function get_joint_state(model, data)
    state = Dict{String, Float64}()
    for name in joint_names
        joint_id = mj_name2id(model, Int32(LibMuJoCo.mjOBJ_JOINT), name)
        if joint_id >= 0
            addr = model.jnt_qposadr[joint_id + 1] + 1
            state[name] = rad2deg(data.qpos[addr])
        end
    end
    return state
end

# Robot-specific: how to apply joint commands (degrees -> radians)
# Accepts both joint names (joint_1) and actuator names (joint_1_actuator)
function apply_joint_command!(data, actuator_map_unused, joints)
    for (name, val) in joints
        name_str = String(name)
        if haskey(joint_to_actuator, name_str)
            idx = joint_to_actuator[name_str]
            data.ctrl[idx] = deg2rad(Float64(val))
        else
            @warn "Unknown joint: $name_str (available: $(join(joint_names, ", ")))"
        end
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
        )
    ]
)

# --- Run Visualization ---
println("\nInitializing visualizer...")
init_visualiser()

println("Starting Fanuc $robot simulation with WebSocket control...")
println("WebSocket control:  ws://localhost:8081")
println("Camera streams:     ws://localhost:8082 (front), 8083 (side), 8084 (orbit)")
println()
println("Joint names: $(join(joint_names, ", "))")
println()
println("Press 'Space' to pause/unpause, 'F1' for help, close window to exit\n")

run_with_capture!(model, data,
    controller = ctrl!,
    capture = capture_config
)

println("Simulation ended.")
