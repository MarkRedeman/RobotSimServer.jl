# LeKiwi Teleoperation Client Example
#
# Connects to the unified server and teloperates the LeKiwi mobile robot:
# - Mobile base drives in circles (constant forward velocity + angular velocity)
# - SO-ARM100 arm moves in sine wave patterns
#
# This example validates that external clients can control the robot and
# other viewers (web URDF viewer, camera streams) see the changes.
#
# Usage:
#   julia --project=. examples/clients/teleoperate_lekiwi_client_ws.jl
#
# Make sure the unified server is running first:
#   julia --project=. -t 4 unified_server.jl
#
# Or with mise:
#   mise run server

using HTTP
using JSON

# =============================================================================
# Configuration
# =============================================================================

const SERVER_URL = "ws://localhost:8080/lekiwi/control?leader=lekiwi"

# Base velocity limits (from LeKiwi specs)
const BASE_VX_MAX = 0.15   # m/s forward/backward
const BASE_VY_MAX = 0.10   # m/s strafe
const BASE_OMEGA_MAX = 0.8 # rad/s angular

# Arm joint names (LeKiwi arm uses SO-ARM100 naming)
const ARM_JOINTS = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]

# Wheel actuator names (velocity control, in rad/s)
const WHEEL_JOINTS = ["base_left_wheel", "base_right_wheel", "base_back_wheel"]

# =============================================================================
# Omniwheel Kinematics (LeKiwi 3-wheel omnidrive)
# =============================================================================

# LeKiwi uses 3 omniwheels at 120° spacing for holonomic motion.
# Given desired body velocity (vx, vy, ω), compute wheel velocities:
#
#   wheel_vel[i] = (1/r) * (-sin(α[i])*vx + cos(α[i])*vy + R*ω)
#
# Where:
#   r = 0.05 m (wheel radius)
#   R = 0.125 m (robot base radius)
#   α = [π/2, π/2 + 2π/3, π/2 + 4π/3] (wheel angles: left, right, back)

const LEKIWI_WHEEL_RADIUS = 0.05       # meters
const LEKIWI_BASE_RADIUS = 0.125       # meters
const LEKIWI_WHEEL_ANGLES = [π / 2, π / 2 + 2π / 3, π / 2 + 4π / 3]

"""
    body_to_wheel_velocities(vx, vy, omega) -> Vector{Float64}

Convert body velocities to wheel velocities for LeKiwi's 3-wheel omnidrive.

# Arguments
- `vx`: Forward/backward velocity in m/s (positive = forward)
- `vy`: Strafe velocity in m/s (positive = left)
- `omega`: Angular velocity in rad/s (positive = counter-clockwise)

# Returns
- Vector of 3 wheel velocities in rad/s: [left, right, back]
"""
function body_to_wheel_velocities(vx::Float64, vy::Float64, omega::Float64)
    wheel_vels = zeros(3)
    for i in 1:3
        α = LEKIWI_WHEEL_ANGLES[i]
        wheel_vels[i] = (1 / LEKIWI_WHEEL_RADIUS) * (
            -sin(α) * vx + cos(α) * vy + LEKIWI_BASE_RADIUS * omega
        )
    end
    return wheel_vels
end

# =============================================================================
# Command Senders
# =============================================================================

"""
    send_joints_command!(ws, joints::Dict)

Send joint state command. 
- Arm joints: values in degrees
- Wheel joints: values in rad/s (velocity control)
"""
function send_joints_command!(ws, joints::Dict)
    payload = Dict("command" => "set_joints_state", "joints" => joints)
    HTTP.WebSockets.send(ws, JSON.json(payload))
end

# =============================================================================
# Motion Patterns
# =============================================================================

"""
    compute_arm_sinewave(t::Float64) -> Dict

Compute arm joint positions based on sine waves at time t.
Returns joint angles in degrees.
"""
function compute_arm_sinewave(t::Float64)
    return Dict(
        "Rotation" => 30.0 * sin(0.5 * t),      # ±30° at 0.5 Hz
        "Pitch" => 20.0 * sin(0.7 * t),         # ±20° at 0.7 Hz
        "Elbow" => 15.0 * sin(0.6 * t),         # ±15° at 0.6 Hz
        "Wrist_Pitch" => 25.0 * sin(0.8 * t),   # ±25° at 0.8 Hz
        "Wrist_Roll" => 40.0 * sin(0.4 * t),    # ±40° at 0.4 Hz
        "Jaw" => 30.0 + 20.0 * sin(0.3 * t)     # 10-50° range at 0.3 Hz
    )
end

"""
    compute_circular_base_velocity() -> Tuple{Float64, Float64, Float64}

Compute base velocity for circular motion.
Returns (vx, vy, omega).
"""
function compute_circular_base_velocity()
    # Constant forward velocity + constant angular velocity = circular path
    vx = 0.05    # Slow forward (m/s)
    vy = 0.0     # No strafe
    omega = 0.3  # Turning rate (rad/s)
    return (vx, vy, omega)
end

"""
    compute_wheel_joints(vx, vy, omega) -> Dict

Compute wheel joint velocities from body velocities.
Returns Dict with wheel actuator names and velocities in rad/s.
"""
function compute_wheel_joints(vx::Float64, vy::Float64, omega::Float64)
    wheel_vels = body_to_wheel_velocities(vx, vy, omega)
    return Dict(
        "base_left_wheel" => wheel_vels[1],
        "base_right_wheel" => wheel_vels[2],
        "base_back_wheel" => wheel_vels[3]
    )
end

# =============================================================================
# Main Teleoperation Loop
# =============================================================================

"""
    teleoperate_loop(ws)

Main teleoperation loop. Runs until Ctrl+C.
"""
function teleoperate_loop(ws)
    println("Starting teleoperation...")
    println("  - Base: driving in circles (vx=0.05 m/s, omega=0.3 rad/s)")
    println("  - Arm: sine wave motion on all joints")
    println("  - Using set_joints_state for both arm and wheels")
    println()
    println("Press Ctrl+C to stop.")
    println()

    start_time = time()
    loop_count = 0
    update_rate = 30  # Hz

    try
        while true
            t = time() - start_time
            loop_count += 1

            # Compute arm positions
            arm_joints = compute_arm_sinewave(t)

            # Compute wheel velocities from body velocity
            vx, vy, omega = compute_circular_base_velocity()
            wheel_joints = compute_wheel_joints(vx, vy, omega)

            # Merge arm and wheel joints into single command
            all_joints = merge(arm_joints, wheel_joints)
            send_joints_command!(ws, all_joints)

            # Status update every 2 seconds
            if loop_count % (update_rate * 2) == 0
                elapsed = round(t, digits = 1)
                println("  Running... $(elapsed)s elapsed")
            end

            # Maintain update rate
            sleep(1.0 / update_rate)
        end
    catch e
        if e isa InterruptException
            println()
            println("Stopping teleoperation...")

            # Stop the base and return arm to neutral
            println("  Stopping base and returning arm to neutral...")
            stop_joints = Dict(
                # Arm neutral positions
                "Rotation" => 0.0,
                "Pitch" => 0.0,
                "Elbow" => 0.0,
                "Wrist_Pitch" => 0.0,
                "Wrist_Roll" => 0.0,
                "Jaw" => 30.0,
                # Wheel velocities to zero
                "base_left_wheel" => 0.0,
                "base_right_wheel" => 0.0,
                "base_back_wheel" => 0.0
            )
            send_joints_command!(ws, stop_joints)
            sleep(0.1)
        else
            rethrow(e)
        end
    end
end

# =============================================================================
# Entry Point
# =============================================================================

function main()
    println("LeKiwi Teleoperation Client")
    println("===========================")
    println()
    println("Connecting to $SERVER_URL...")

    HTTP.WebSockets.open(SERVER_URL) do ws
        # Ping test
        println("Sending ping...")
        HTTP.WebSockets.send(ws, JSON.json(Dict("command" => "ping")))
        msg = HTTP.WebSockets.receive(ws)
        response = JSON.parse(String(msg))
        if get(response, "event", "") == "pong"
            println("Connected successfully!")
        else
            println("Received: ", String(msg))
        end
        println()

        # Run teleoperation loop
        teleoperate_loop(ws)
    end

    println()
    println("Done.")
end

main()
