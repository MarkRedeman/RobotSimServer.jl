using MuJoCo

# Discover available Fanuc robots by scanning fanuc_mujoco directory
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

# Get available robots
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

println("Loading Fanuc $robot...")

# Path to the MuJoCo scene file
xml_path = joinpath(@__DIR__, "..", "..", "robots", "fanuc_mujoco", robot, "scene.xml")

if !isfile(xml_path)
    error("Scene file not found: $xml_path\nRun 'python3 scripts/convert_fanuc_industrial.py' to generate MuJoCo XMLs.")
end

# Load the model and data
model = load_model(xml_path)
data = init_data(model)

println("Loaded model with $(model.nq) DOF, $(model.nu) actuators")

# Define a controller that moves the arm in a sine wave pattern
function ctrl!(model, data)
    t = data.time

    # Number of actuators varies by robot (typically 5-6 for industrial arms)
    n = min(model.nu, 6)

    # Gentle sine wave motion on all joints
    # Using smaller amplitudes to stay within typical joint limits
    if n >= 1
        data.ctrl[1] = 0.5 * sin(t)           # Joint 1 - Base rotation
    end
    if n >= 2
        data.ctrl[2] = 0.3 * cos(t)           # Joint 2 - Shoulder
    end
    if n >= 3
        data.ctrl[3] = 0.4 * sin(2 * t)       # Joint 3 - Elbow
    end
    if n >= 4
        data.ctrl[4] = 0.5 * sin(t)           # Joint 4 - Wrist 1
    end
    if n >= 5
        data.ctrl[5] = 0.6 * cos(t)           # Joint 5 - Wrist 2
    end
    if n >= 6
        data.ctrl[6] = 1.0 * sin(0.5 * t)     # Joint 6 - Wrist 3 (flange)
    end
end

# Run the visualiser
init_visualiser()
visualise!(model, data; controller = ctrl!)
