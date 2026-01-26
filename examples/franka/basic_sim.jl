# Franka Panda Basic Simulation
#
# A simple demonstration that moves all joints in a sine wave pattern.
# The Franka Panda is a 7-DOF robot arm with a 2-finger parallel gripper.
#
# Uses the MuJoCo Menagerie Franka Emika Panda model.
#
# Usage:
#   julia --project=. examples/franka/basic_sim.jl
#
# Note: Joint 4 has an unusual range of -3.07 to -0.07 radians
# (always negative), so the sine wave is offset to stay within limits.

using MuJoCo

# Path to the Franka Panda model (MuJoCo Menagerie)
xml_path = joinpath(@__DIR__, "..", "..", "robots", "google-deepmind",
    "franka_emika_panda", "panda.xml")
println("Loading Franka Panda from: $xml_path")

# Load the model and data
model = load_model(xml_path)
data = init_data(model)

println("Loaded model with $(model.nq) DOF, $(model.nu) actuators")

# Define a controller that moves all joints in a sine wave pattern
function ctrl!(model, data)
    t = data.time

    # Arm joints (7 DOF) - position controlled
    # Actuators 1-7 control joints 1-7
    data.ctrl[1] = 0.5 * sin(t)             # joint1 - base rotation
    data.ctrl[2] = 0.3 * sin(t * 0.8)       # joint2 - shoulder
    data.ctrl[3] = 0.4 * sin(t * 1.1)       # joint3 - elbow
    data.ctrl[4] = -1.5 + 0.3 * sin(t)      # joint4 - wrist1 (NEGATIVE ONLY: -3.07 to -0.07)
    data.ctrl[5] = 0.5 * cos(t)             # joint5 - wrist2
    data.ctrl[6] = 1.0 + 0.3 * sin(t * 1.3) # joint6 - wrist3 (range: -0.02 to 3.75)
    data.ctrl[7] = 0.8 * cos(t * 0.9)       # joint7 - flange

    # Gripper (actuator 8) controls both fingers via tendon
    # Control range is 0-255 (mapped to 0-0.04m finger opening)
    grip = 127.5 + 127.5 * sin(t * 2)       # Oscillate between closed (0) and open (255)
    data.ctrl[8] = grip
end

# Run the visualiser
init_visualiser()
visualise!(model, data, controller = ctrl!)
