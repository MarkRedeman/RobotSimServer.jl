using MuJoCo
using ZMQ
using JSON
using Dates

# Path to the SO101 model
xml_path = joinpath(
    @__DIR__, "..", "..", "robots", "SO-ARM100", "Simulation", "SO101", "scene.xml")

# Load the SO101 model and data
model = load_model(xml_path)
data = init_data(model)

# ZMQ Setup
ctx = Context()
socket = Socket(ctx, REP)
ZMQ.bind(socket, "tcp://*:5555")
println("ZMQ server listening on tcp://*:5555")

# Map actuator names to indices
actuator_names = [unsafe_string(mj_id2name(
                      model, Int32(MuJoCo.LibMuJoCo.mjOBJ_ACTUATOR), i - 1))
                  for i in 1:(model.nu)]
actuator_map = Dict(name => i for (i, name) in enumerate(actuator_names))

# Controller that checks for ZMQ messages
function ctrl!(model, data)
    # Check if there is a message waiting (non-blocking)
    # ZMQ.POLLIN indicates data is available to read
    if (ZMQ.get_events(socket) & ZMQ.POLLIN) != 0
        msg = ZMQ.recv(socket)
        raw_data = JSON.parse(unsafe_string(msg))

        response = Dict{String, Any}("timestamp" => datetime2unix(now()))

        if haskey(raw_data, "command")
            cmd = String(raw_data["command"])
            if cmd == "ping"
                response["event"] = "pong"
            elseif cmd == "set_joints_state"
                if haskey(raw_data, "joints")
                    joints = raw_data["joints"]
                    for (name, val) in joints
                        s_name = String(name)
                        if haskey(actuator_map, s_name)
                            idx = actuator_map[s_name]
                            data.ctrl[idx] = Float64(val)
                        end
                    end
                    response["event"] = "set_joints_state"
                end
            elseif cmd == "read_state"
                state = Dict{String, Float64}()
                for (name, idx) in actuator_map
                    state[name] = Float64(data.ctrl[idx])
                end
                response["event"] = "read_state"
                response["joints"] = state
            else
                response["event"] = "error"
                response["message"] = "Unknown command: $cmd"
            end
        end

        ZMQ.send(socket, JSON.json(response))
    end
end

# Run the visualiser
init_visualiser()

# The visualiser starts paused by default and doesn't expose an unpause flag.
# You can press SPACE in the window to start the simulation.

visualise!(model, data, controller = ctrl!)
