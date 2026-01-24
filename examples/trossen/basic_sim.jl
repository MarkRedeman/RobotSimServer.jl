using MuJoCo

# Path to the Trossen model
xml_path = joinpath(@__DIR__, "..", "..", "robots", "trossen_arm_mujoco",
    "trossen_arm_mujoco", "assets", "wxai", "scene.xml")

# Load the SO101 model and data
model = load_model(xml_path)
data = init_data(model)

# Define a controller that moves the arm in a sine wave
function ctrl!(model, data)
    t = data.time

    # Set target positions for actuators (units are radians for these motors)
    data.ctrl[1] = 1.0 * sin(t)        # shoulder_pan
    data.ctrl[2] = 0.5 * cos(t)        # shoulder_lift
    data.ctrl[3] = 0.8 * sin(2 * t)      # elbow_flex
    data.ctrl[4] = 0.5 * sin(t)        # wrist_flex
    data.ctrl[5] = 1.5 * cos(t)        # wrist_roll
    data.ctrl[6] = 0.8                 # gripper (open/close)
end

# Run the visualiser
init_visualiser()
visualise!(model, data, controller = ctrl!)
