using ZMQ
using JSON
using Dates

function main()
    ctx = Context()
    socket = Socket(ctx, REQ)
    ZMQ.connect(socket, "tcp://localhost:5555")
    println("Connected to ZMQ server at tcp://localhost:5555")

    # 1. Ping
    println("\nSending ping...")
    ZMQ.send(socket, JSON.json(Dict("command" => "ping")))
    msg = ZMQ.recv(socket)
    println("Received: ", unsafe_string(msg))

    # 2. Set joint states
    println("\nMoving arm (Press Ctrl+C to stop)...")
    commands = [
        Dict("shoulder_pan" => 1.0, "shoulder_lift" => 0.5, "elbow_flex" => -1.0),
        Dict("shoulder_pan" => -1.0, "shoulder_lift" => -0.5, "elbow_flex" => 1.0),
        Dict("shoulder_pan" => 0.0, "shoulder_lift" => 0.0, "elbow_flex" => 0.0)
    ]

    try
        while true
            for cmd in commands
                println("Sending set_joints_state: ", cmd)
                payload = Dict(
                    "command" => "set_joints_state",
                    "joints" => cmd
                )
                ZMQ.send(socket, JSON.json(payload))
                msg = ZMQ.recv(socket)
                # println("Received: ", unsafe_string(msg))
                sleep(1)
            end

            # 3. Read state occasionally
            ZMQ.send(socket, JSON.json(Dict("command" => "read_state")))
            msg = ZMQ.recv(socket)
            println("Current State: ", unsafe_string(msg))
        end
    catch e
        if e isa InterruptException
            println("\nStopping client...")
        else
            rethrow(e)
        end
    finally
        ZMQ.close(socket)
        ZMQ.term(ctx)
    end
end

main()
