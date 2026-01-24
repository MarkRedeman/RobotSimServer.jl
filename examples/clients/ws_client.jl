# WebSocket Client Example for Robot Arm Control
#
# Moves the gripper in a sweeping pattern over the ground to interact with cubes.
# All joint values are in DEGREES.
#
# Usage:
#   julia --project=. examples/clients/ws_client.jl
#
# Make sure the simulation is running first:
#   julia --project=. -t 4 examples/so101/websocket_sim.jl

using HTTP
using JSON

# Joint limits (degrees)
const JOINT_LIMITS = Dict(
    "shoulder_pan" => (-110.0, 110.0),
    "shoulder_lift" => (-100.0, 100.0),
    "elbow_flex" => (-97.0, 97.0),
    "wrist_flex" => (-95.0, 95.0),
    "wrist_roll" => (-157.0, 163.0),
    "gripper" => (-10.0, 100.0)  # -10° = closed, 100° = open
)

"""
    send_joints!(ws, joints::Dict)

Send joint command and print status.
"""
function send_joints!(ws, joints::Dict)
    payload = Dict("command" => "set_joints_state", "joints" => joints)
    HTTP.WebSockets.send(ws, JSON.json(payload))
end

"""
    lerp_joints(from::Dict, to::Dict, t::Float64) -> Dict

Linear interpolation between two joint configurations.
"""
function lerp_joints(from::Dict, to::Dict, t::Float64)
    result = Dict{String, Float64}()
    for (key, val) in to
        from_val = get(from, key, val)
        result[key] = from_val + t * (val - from_val)
    end
    return result
end

"""
    move_to!(ws, target::Dict; steps=20, delay=0.05)

Smoothly move to target joint configuration.
"""
function move_to!(ws, current::Dict, target::Dict; steps = 20, delay = 0.05)
    for i in 1:steps
        t = i / steps
        joints = lerp_joints(current, target, t)
        send_joints!(ws, joints)
        sleep(delay)
    end
    return target
end

"""
    sweep_pattern(ws)

Move the gripper in a sweeping pattern over the ground.
All values in degrees.
"""
function sweep_pattern(ws)
    # Home position - arm pointing forward, gripper open
    home = Dict(
        "shoulder_pan" => 0.0,
        "shoulder_lift" => 0.0,
        "elbow_flex" => 0.0,
        "wrist_flex" => 0.0,
        "wrist_roll" => 0.0,
        "gripper" => 57.0  # Open (~1 rad)
    )

    # Low positions - gripper near ground level (values in degrees)
    low_forward = Dict(
        "shoulder_pan" => 0.0,
        "shoulder_lift" => 80.0,   # ~1.4 rad
        "elbow_flex" => -46.0,  # ~-0.8 rad
        "wrist_flex" => 0.0,
        "wrist_roll" => 0.0,
        "gripper" => 57.0
    )

    low_left = Dict(
        "shoulder_pan" => 69.0,   # ~1.2 rad
        "shoulder_lift" => 80.0,
        "elbow_flex" => -46.0,
        "wrist_flex" => 0.0,
        "wrist_roll" => 0.0,
        "gripper" => 57.0
    )

    low_right = Dict(
        "shoulder_pan" => -69.0,
        "shoulder_lift" => 80.0,
        "elbow_flex" => -46.0,
        "wrist_flex" => 0.0,
        "wrist_roll" => 0.0,
        "gripper" => 57.0
    )

    low_back_left = Dict(
        "shoulder_pan" => 86.0,   # ~1.5 rad
        "shoulder_lift" => 69.0,   # ~1.2 rad
        "elbow_flex" => -34.0,  # ~-0.6 rad
        "wrist_flex" => 0.0,
        "wrist_roll" => 0.0,
        "gripper" => 57.0
    )

    low_back_right = Dict(
        "shoulder_pan" => -86.0,
        "shoulder_lift" => 69.0,
        "elbow_flex" => -34.0,
        "wrist_flex" => 0.0,
        "wrist_roll" => 0.0,
        "gripper" => 57.0
    )

    println("Moving to home position...")
    current = move_to!(ws, home, home; steps = 30, delay = 0.03)
    sleep(0.5)

    println("Starting sweep pattern (Ctrl+C to stop)...")

    # Sweep pattern: home -> forward -> left -> right -> back_left -> back_right -> repeat
    waypoints = [
        low_forward, low_left, low_forward, low_right, low_back_left, low_back_right]
    waypoint_names = ["forward", "left", "forward", "right", "back-left", "back-right"]

    try
        while true
            for (wp, name) in zip(waypoints, waypoint_names)
                println("  Sweeping $name...")
                current = move_to!(ws, current, wp; steps = 40, delay = 0.02)
                sleep(0.2)
            end
        end
    catch e
        if e isa InterruptException
            println("\nReturning to home...")
            move_to!(ws, current, home; steps = 30, delay = 0.03)
        else
            rethrow(e)
        end
    end
end

"""
    push_cubes_pattern(ws)

Aggressive pushing pattern to scatter cubes.
All values in degrees.
"""
function push_cubes_pattern(ws)
    # Start with arm up
    home = Dict(
        "shoulder_pan" => 0.0,
        "shoulder_lift" => 0.0,
        "elbow_flex" => 0.0,
        "wrist_flex" => 0.0,
        "wrist_roll" => 0.0,
        "gripper" => 57.0  # Open (~1 rad)
    )

    println("Moving to start position...")
    current = move_to!(ws, home, home; steps = 30, delay = 0.03)
    sleep(0.5)

    println("Pushing cubes around (Ctrl+C to stop)...")

    try
        while true
            # Rotate around and push down (-86° to 86°, was -1.5 to 1.5 rad)
            for pan in range(-86.0, 86.0, length = 8)
                # Move to position above
                up_pos = Dict(
                    "shoulder_pan" => pan,
                    "shoulder_lift" => 46.0,   # ~0.8 rad
                    "elbow_flex" => -23.0,  # ~-0.4 rad
                    "wrist_flex" => 0.0,
                    "wrist_roll" => 0.0,
                    "gripper" => 57.0
                )
                current = move_to!(ws, current, up_pos; steps = 15, delay = 0.02)

                # Push down - gripper close to ground
                down_pos = Dict(
                    "shoulder_pan" => pan,
                    "shoulder_lift" => 86.0,   # ~1.5 rad
                    "elbow_flex" => -46.0,  # ~-0.8 rad
                    "wrist_flex" => 0.0,
                    "wrist_roll" => 0.0,
                    "gripper" => 57.0
                )
                current = move_to!(ws, current, down_pos; steps = 20, delay = 0.02)

                # Sweep sideways while low (+17° offset, was +0.3 rad)
                sweep_pos = Dict(
                    "shoulder_pan" => pan + 17.0,
                    "shoulder_lift" => 86.0,
                    "elbow_flex" => -46.0,
                    "wrist_flex" => 0.0,
                    "wrist_roll" => 0.0,
                    "gripper" => 57.0
                )
                current = move_to!(ws, current, sweep_pos; steps = 15, delay = 0.02)
            end
        end
    catch e
        if e isa InterruptException
            println("\nReturning to home...")
            move_to!(ws, current, home; steps = 30, delay = 0.03)
        else
            rethrow(e)
        end
    end
end

"""
    gripper_test_pattern(ws)

Test pattern that opens and closes the gripper while moving the arm.
Good for testing gripper-mounted cameras.
All values in degrees.
"""
function gripper_test_pattern(ws)
    # Home position - arm pointing forward
    home = Dict(
        "shoulder_pan" => 0.0,
        "shoulder_lift" => 0.0,
        "elbow_flex" => 0.0,
        "wrist_flex" => 0.0,
        "wrist_roll" => 0.0,
        "gripper" => 29.0  # Half open (~0.5 rad)
    )

    # Position with gripper visible and accessible
    view_pos = Dict(
        "shoulder_pan" => 0.0,
        "shoulder_lift" => 57.0,   # ~1.0 rad
        "elbow_flex" => -29.0,  # ~-0.5 rad
        "wrist_flex" => 0.0,
        "wrist_roll" => 0.0,
        "gripper" => 29.0
    )

    println("Moving to view position...")
    current = move_to!(ws, home, home; steps = 30, delay = 0.03)
    current = move_to!(ws, current, view_pos; steps = 30, delay = 0.03)
    sleep(0.5)

    println("Starting gripper open/close pattern (Ctrl+C to stop)...")

    try
        while true
            # Open gripper
            println("  Opening gripper...")
            open_pos = copy(current)
            open_pos["gripper"] = 86.0  # Fully open (~1.5 rad)
            current = move_to!(ws, current, open_pos; steps = 25, delay = 0.04)
            sleep(0.3)

            # Close gripper
            println("  Closing gripper...")
            closed_pos = copy(current)
            closed_pos["gripper"] = -6.0  # Fully closed (~-0.1 rad)
            current = move_to!(ws, current, closed_pos; steps = 25, delay = 0.04)
            sleep(0.3)

            # Rotate wrist while opening
            println("  Rotating wrist + opening...")
            rotate_open = copy(current)
            rotate_open["wrist_roll"] = 86.0   # ~1.5 rad
            rotate_open["gripper"] = 86.0
            current = move_to!(ws, current, rotate_open; steps = 30, delay = 0.03)
            sleep(0.2)

            # Rotate back while closing
            println("  Rotating back + closing...")
            rotate_close = copy(current)
            rotate_close["wrist_roll"] = -86.0  # ~-1.5 rad
            rotate_close["gripper"] = -6.0
            current = move_to!(ws, current, rotate_close; steps = 30, delay = 0.03)
            sleep(0.2)

            # Return to center
            println("  Centering...")
            center = copy(current)
            center["wrist_roll"] = 0.0
            center["gripper"] = 29.0
            current = move_to!(ws, current, center; steps = 20, delay = 0.03)
            sleep(0.3)

            # Move arm around while operating gripper
            println("  Moving arm + gripper cycle...")
            for pan in [-46.0, 46.0, 0.0]  # ~-0.8, 0.8, 0 rad
                target = copy(current)
                target["shoulder_pan"] = pan
                target["gripper"] = pan > 0 ? 86.0 : -6.0  # Open when left, close when right
                current = move_to!(ws, current, target; steps = 25, delay = 0.03)
                sleep(0.2)
            end
        end
    catch e
        if e isa InterruptException
            println("\nReturning to home...")
            move_to!(ws, current, home; steps = 30, delay = 0.03)
        else
            rethrow(e)
        end
    end
end

function main()
    println("SO101 Robot Arm Test Client")
    println("===========================")
    println()
    println("Connecting to ws://localhost:8081...")

    HTTP.WebSockets.open("ws://localhost:8081") do ws
        # Ping test
        println("Sending ping...")
        HTTP.WebSockets.send(ws, JSON.json(Dict("command" => "ping")))
        msg = HTTP.WebSockets.receive(ws)
        println("Received: ", String(msg))
        println()

        # Choose pattern
        println("Select pattern:")
        println("  1. Sweep pattern (gentle sweeping motion)")
        println("  2. Push cubes (aggressive pushing)")
        println("  3. Gripper test (open/close gripper)")
        print("Choice [1]: ")
        choice = readline()

        if choice == "2"
            push_cubes_pattern(ws)
        elseif choice == "3"
            gripper_test_pattern(ws)
        else
            sweep_pattern(ws)
        end
    end

    println("Done.")
end

main()
