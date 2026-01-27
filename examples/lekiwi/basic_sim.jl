using MuJoCo

# Path to the LeKiwi model (mobile base + SO-ARM100)
# Note: We use a flattened scene.xml because MuJoCo.jl doesn't support
# the model attachment feature (<model>/<attach>) used in the original.
xml_path = joinpath(@__DIR__, "scene.xml")

# Load the LeKiwi model and data
model = load_model(xml_path)
data = init_data(model)

# ============================================================================
# Actuator Mapping (9 actuators total)
# ============================================================================
# Index | Name             | Type   | Control Mode | Range
# ------|------------------|--------|--------------|------------------
#   1   | base_left_wheel  | Wheel  | Velocity     | ±3.14 rad/s
#   2   | base_right_wheel | Wheel  | Velocity     | ±3.14 rad/s
#   3   | base_back_wheel  | Wheel  | Velocity     | ±3.14 rad/s
#   4   | Rotation         | Arm    | Position     | ±1.92 rad (±110°)
#   5   | Pitch            | Arm    | Position     | ±1.747 rad (±100°)
#   6   | Elbow            | Arm    | Position     | ±1.657 rad (±95°)
#   7   | Wrist_Pitch      | Arm    | Position     | ±1.66 rad (±95°)
#   8   | Wrist_Roll       | Arm    | Position     | ±2.79 rad (±160°)
#   9   | Jaw              | Grip   | Position     | 0 to 0.6 rad
# ============================================================================

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
# ============================================================================

const WHEEL_RADIUS = 0.05       # meters
const BASE_RADIUS = 0.125       # meters

# Wheel angles (120° apart): left, right, back
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

# Define a controller that drives in circles while waving the arm
function ctrl!(model, data)
    t = data.time

    # === Mobile Base: Drive in a circle ===
    # Slow forward motion + constant rotation = circular path
    vx = 0.05           # Forward velocity (m/s)
    vy = 0.0            # Strafe velocity (m/s)
    omega = 0.3         # Angular velocity (rad/s) - turning left

    wheel_vels = body_to_wheel_velocities(vx, vy, omega)
    data.ctrl[1] = wheel_vels[1]  # left wheel
    data.ctrl[2] = wheel_vels[2]  # right wheel
    data.ctrl[3] = wheel_vels[3]  # back wheel

    # === Arm: Wave motion ===
    data.ctrl[4] = 0.5 * sin(t)           # Rotation - side to side
    data.ctrl[5] = 0.3 * cos(t)           # Pitch - nod up/down
    data.ctrl[6] = 0.4 * sin(0.5 * t)     # Elbow - slow flex
    data.ctrl[7] = 0.3 * sin(t)           # Wrist_Pitch
    data.ctrl[8] = 0.5 * cos(t)           # Wrist_Roll

    # === Gripper: Open and close periodically ===
    data.ctrl[9] = 0.3 + 0.3 * sin(0.5 * t)  # Jaw: oscillate between 0 and 0.6
end

# Run the visualiser
init_visualiser()
visualise!(model, data; controller = ctrl!)
