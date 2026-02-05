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

# =============================================================================
# Command Senders
# =============================================================================

"""
    send_arm_command!(ws, joints::Dict)

Send arm joint positions command. All values in degrees.
"""
function send_arm_command!(ws, joints::Dict)
    payload = Dict("command" => "set_joints_state", "joints" => joints)
    HTTP.WebSockets.send(ws, JSON.json(payload))
end

"""
    send_base_velocity!(ws, vx::Float64, vy::Float64, omega::Float64)

Send base velocity command.
- vx: forward/backward velocity (m/s)
- vy: strafe velocity (m/s)
- omega: angular velocity (rad/s)
"""
function send_base_velocity!(ws, vx::Float64, vy::Float64, omega::Float64)
    payload = Dict(
        "command" => "set_base_velocity",
        "vx" => vx,
        "vy" => vy,
        "omega" => omega
    )
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

            # Compute and send arm position
            arm_joints = compute_arm_sinewave(t)
            send_arm_command!(ws, arm_joints)

            # Compute and send base velocity
            vx, vy, omega = compute_circular_base_velocity()
            send_base_velocity!(ws, vx, vy, omega)

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

            # Stop the base
            println("  Stopping base...")
            send_base_velocity!(ws, 0.0, 0.0, 0.0)

            # Return arm to neutral position
            println("  Returning arm to neutral...")
            neutral_joints = Dict(
                "Rotation" => 0.0,
                "Pitch" => 0.0,
                "Elbow" => 0.0,
                "Wrist_Pitch" => 0.0,
                "Wrist_Roll" => 0.0,
                "Jaw" => 30.0
            )
            send_arm_command!(ws, neutral_joints)
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
