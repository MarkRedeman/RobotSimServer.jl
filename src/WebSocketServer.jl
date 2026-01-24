# WebSocket Control Server - Shared WebSocket logic for robot simulations
#
# This module provides a reusable WebSocket server for robot joint control.
# It handles client connections, command processing, and state broadcasting.
#
# Usage:
#   include("../../src/WebSocketServer.jl")
#
#   server = WebSocketControlServer(port=8081, fps=30.0)
#
#   # Define robot-specific callbacks
#   get_state = (model, data) -> Dict("joint1" => rad2deg(data.qpos[1]), ...)
#   apply_command! = (data, actuator_map, joints) -> begin
#       for (name, val) in joints
#           data.ctrl[actuator_map[name]] = deg2rad(val)
#       end
#   end
#
#   start_server!(server, get_state)
#
#   # In your control loop:
#   function ctrl!(m, d)
#       process_commands!(server, d, actuator_map, apply_command!)
#       maybe_broadcast!(server, m, d, get_state)
#   end

using HTTP
using JSON

"""
    WebSocketControlServer

Manages WebSocket connections for robot joint control with state broadcasting.

# Fields
- `port::Int`: WebSocket server port (default 8081)
- `control_channel::Channel{Dict{String, Any}}`: Channel for incoming commands
- `clients::Set{Any}`: Connected WebSocket clients
- `clients_lock::ReentrantLock`: Lock for thread-safe client access
- `last_broadcast_time::Ref{Float64}`: Time of last state broadcast
- `broadcast_interval::Float64`: Minimum time between broadcasts (1/fps)
- `prev_state::Dict{String, Float64}`: Previous state for change detection
- `state_change_threshold::Float64`: Minimum change to trigger broadcast (degrees)
- `model_ref::Ref{Any}`: Reference to MuJoCo model (for new client state sends)
- `data_ref::Ref{Any}`: Reference to MuJoCo data (for new client state sends)
"""
mutable struct WebSocketControlServer
    port::Int
    control_channel::Channel{Dict{String, Any}}
    clients::Set{Any}
    clients_lock::ReentrantLock
    last_broadcast_time::Ref{Float64}
    broadcast_interval::Float64
    prev_state::Dict{String, Float64}
    state_change_threshold::Float64
    model_ref::Ref{Any}
    data_ref::Ref{Any}
end

"""
    WebSocketControlServer(; port=8081, fps=30.0, state_change_threshold=0.01)

Create a new WebSocket control server.

# Arguments
- `port`: Port to listen on (default 8081)
- `fps`: Maximum state broadcast rate in Hz (default 30.0)
- `state_change_threshold`: Minimum joint change in degrees to trigger broadcast (default 0.01)
"""
function WebSocketControlServer(; port::Int = 8081, fps::Float64 = 30.0,
        state_change_threshold::Float64 = 0.01)
    return WebSocketControlServer(
        port,
        Channel{Dict{String, Any}}(10),
        Set{Any}(),
        ReentrantLock(),
        Ref(0.0),
        1.0 / fps,
        Dict{String, Float64}(),
        state_change_threshold,
        Ref{Any}(nothing),
        Ref{Any}(nothing)
    )
end

"""
    state_changed(current::Dict, previous::Dict, threshold::Float64) -> Bool

Check if state has changed beyond the threshold.
"""
function state_changed(current::Dict, previous::Dict, threshold::Float64)
    isempty(previous) && return true
    for (name, val) in current
        prev_val = get(previous, name, val + threshold * 2)
        if abs(val - prev_val) > threshold
            return true
        end
    end
    return false
end

"""
    build_state_message(state::Dict) -> String

Build JSON message for state broadcast.
"""
function build_state_message(state::Dict)
    return JSON.json(Dict(
        "event" => "state_was_updated",
        "timestamp" => time(),
        "state" => state,
        "is_controlled" => false
    ))
end

"""
    send_state_to_client!(server::WebSocketControlServer, ws, model, data, get_state::Function)

Send current state to a specific client (used for new connections).
"""
function send_state_to_client!(server::WebSocketControlServer, ws, model, data,
        get_state::Function)
    state = get_state(model, data)
    msg = build_state_message(state)
    try
        HTTP.WebSockets.send(ws, msg)
    catch e
        @warn "Failed to send state to client: $e"
    end
end

"""
    start_server!(server::WebSocketControlServer, get_state::Function)

Start the WebSocket server in a background task.

# Arguments
- `server`: The WebSocket control server
- `get_state`: Function (model, data) -> Dict{String, Float64} that returns current joint positions in degrees
"""
function start_server!(server::WebSocketControlServer, get_state::Function)
    @async begin
        HTTP.WebSockets.listen("0.0.0.0", server.port) do ws
            println("WebSocket client connected (control)")
            @lock server.clients_lock push!(server.clients, ws)

            # Send initial state to new client
            if server.model_ref[] !== nothing && server.data_ref[] !== nothing
                send_state_to_client!(server, ws, server.model_ref[], server.data_ref[],
                    get_state)
            end

            try
                for msg in ws
                    try
                        raw = JSON.parse(String(msg))
                        put!(server.control_channel, raw)

                        if get(raw, "command", "") == "ping"
                            HTTP.WebSockets.send(
                                ws, JSON.json(Dict(
                                    "event" => "pong",
                                    "timestamp" => time()
                                )))
                        end
                    catch e
                        @warn "WebSocket error: $e"
                    end
                end
            finally
                @lock server.clients_lock delete!(server.clients, ws)
                println("WebSocket client disconnected (control)")
            end
        end
    end
    println("WebSocket control server listening on ws://0.0.0.0:$(server.port)")
end

"""
    process_commands!(server::WebSocketControlServer, data, actuator_map::Dict, 
                      apply_command!::Function)

Process pending commands from the control channel.

# Arguments
- `server`: The WebSocket control server
- `data`: MuJoCo data object
- `actuator_map`: Dict mapping joint names to actuator indices
- `apply_command!`: Function (data, actuator_map, joints_dict) that applies joint commands
"""
function process_commands!(server::WebSocketControlServer, data, actuator_map::Dict,
        apply_command!::Function)
    while isready(server.control_channel)
        raw = take!(server.control_channel)
        cmd = get(raw, "command", "")

        if cmd == "set_joints_state"
            joints = get(raw, "joints", Dict())
            apply_command!(data, actuator_map, joints)
        end
    end
end

"""
    broadcast_state!(server::WebSocketControlServer, state::Dict)

Broadcast state to all connected clients.
"""
function broadcast_state!(server::WebSocketControlServer, state::Dict)
    msg = build_state_message(state)

    @lock server.clients_lock begin
        dead_clients = []
        for ws in server.clients
            try
                HTTP.WebSockets.send(ws, msg)
            catch e
                push!(dead_clients, ws)
                println("WebSocket client disconnected during broadcast")
            end
        end
        # Remove dead clients
        for ws in dead_clients
            delete!(server.clients, ws)
        end
    end
end

"""
    maybe_broadcast!(server::WebSocketControlServer, model, data, get_state::Function)

Broadcast state if enough time has passed and state has changed.

# Arguments
- `server`: The WebSocket control server
- `model`: MuJoCo model
- `data`: MuJoCo data
- `get_state`: Function (model, data) -> Dict{String, Float64} that returns current joint positions in degrees
"""
function maybe_broadcast!(server::WebSocketControlServer, model, data, get_state::Function)
    # Update refs for new client state sends
    if server.model_ref[] === nothing
        server.model_ref[] = model
        server.data_ref[] = data
    end

    # Throttle broadcasts
    current_time = time()
    if current_time - server.last_broadcast_time[] < server.broadcast_interval
        return
    end

    state = get_state(model, data)

    # Only broadcast if state has changed
    if !state_changed(state, server.prev_state, server.state_change_threshold)
        return
    end

    # Update previous state and broadcast
    merge!(server.prev_state, state)
    broadcast_state!(server, state)
    server.last_broadcast_time[] = current_time
end
