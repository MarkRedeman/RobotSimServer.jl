# MJPEGOutput backend - stream frames as multipart JPEG over HTTP

using HTTP

"""
    UnifiedMJPEGBackendState

State for MJPEG output using the unified server.
"""
mutable struct UnifiedMJPEGBackendState
    server::Any
    camera_name::String
end

const MJPEG_BOUNDARY = "frame"

"""
    MJPEGBackendState

State for MJPEG output backend.
"""
mutable struct MJPEGBackendState
    port::Int
    camera_name::String
    server_task::Union{Task, Nothing}
    clients::Vector{Any}
    clients_lock::ReentrantLock
    shutdown_condition::Condition
    running::Bool
end

"""
    init_backend(backend::MJPEGOutput, camera_name::String, width::Int, height::Int, fps::Float64)

Initialize MJPEG output backend.
"""
function init_backend(backend::MJPEGOutput, camera_name::String,
        width::Int, height::Int, fps::Float64)
    if backend.server !== nothing
        if !haskey(backend.server.mjpeg_cameras, camera_name)
            register_mjpeg_camera!(backend.server, camera_name)
        end

        println("MJPEGOutput($(camera_name)): using unified server")
        println("MJPEGOutput($(camera_name)): streaming on http://127.0.0.1:$(backend.server.port)/$(backend.server.robot)/cameras/$(camera_name)/stream")
        return UnifiedMJPEGBackendState(backend.server, camera_name)
    end

    state = MJPEGBackendState(
        backend.port,
        camera_name,
        nothing,
        Any[],
        ReentrantLock(),
        Condition(),
        true
    )

    state.server_task = @async begin
        try
            HTTP.listen("127.0.0.1", backend.port) do http
                if http.message.method != "GET"
                    HTTP.setstatus(http, 405)
                    HTTP.startwrite(http)
                    write(http, "Method not allowed")
                    return
                end

                if http.message.target != "/stream"
                    HTTP.setstatus(http, 404)
                    HTTP.startwrite(http)
                    write(http, "Not found")
                    return
                end

                add_mjpeg_headers!(http)
                HTTP.startwrite(http)

                client_count = @lock state.clients_lock begin
                    push!(state.clients, http)
                    length(state.clients)
                end
                println("MJPEGOutput($(camera_name)): client connected ($(client_count) total)")

                try
                    # Keep the multipart response open so frames can be pushed
                    # asynchronously until the server shuts down.
                    wait(state.shutdown_condition)
                catch e
                    if !(e isa EOFError)
                        @warn "MJPEG client error" exception = e
                    end
                finally
                    remaining = @lock state.clients_lock begin
                        filter!(c -> c !== http, state.clients)
                        length(state.clients)
                    end
                    println("MJPEGOutput($(camera_name)): client disconnected ($(remaining) remaining)")
                end
            end
        catch e
            if state.running
                @warn "MJPEG server error" exception = (e, catch_backtrace())
            end
        end
    end

    println("MJPEGOutput: streaming $(camera_name) on http://127.0.0.1:$(backend.port)/stream")
    return state
end

function add_mjpeg_headers!(http)
    HTTP.setheader(http, "Content-Type" => "multipart/x-mixed-replace; boundary=$MJPEG_BOUNDARY")
    HTTP.setheader(http, "Cache-Control" => "no-cache, no-store, must-revalidate")
    HTTP.setheader(http, "Pragma" => "no-cache")
    HTTP.setheader(http, "Connection" => "keep-alive")
end

"""
    mjpeg_part_header(jpeg_bytes::Vector{UInt8}) -> String

Build a multipart MJPEG frame header.
"""
function mjpeg_part_header(jpeg_bytes::Vector{UInt8})
    return "--$MJPEG_BOUNDARY\r\nContent-Type: image/jpeg\r\nContent-Length: $(length(jpeg_bytes))\r\n\r\n"
end

"""
    process_frame!(backend::MJPEGOutput, state::MJPEGBackendState, work::CaptureWork)

Encode frame and broadcast as multipart JPEG chunks.
"""
function process_frame!(backend::MJPEGOutput, state::MJPEGBackendState, work::CaptureWork)
    clients = @lock state.clients_lock copy(state.clients)
    if isempty(clients)
        return
    end

    jpeg_bytes = encode_jpeg_frame(work.rgb_data, work.width, work.height)
    header = mjpeg_part_header(jpeg_bytes)

    failed_clients = Any[]
    for client in clients
        try
            write(client, header)
            write(client, jpeg_bytes)
            write(client, "\r\n")
        catch e
            push!(failed_clients, client)
        end
    end

    if !isempty(failed_clients)
        @lock state.clients_lock begin
            filter!(c -> !(c in failed_clients), state.clients)
        end
        for client in failed_clients
            try
                close(client)
            catch
            end
        end
    end
end

"""
    process_frame!(backend::MJPEGOutput, state::UnifiedMJPEGBackendState, work::CaptureWork)

Encode frame and broadcast as multipart JPEG chunks via the unified server.
"""
function process_frame!(
        backend::MJPEGOutput, state::UnifiedMJPEGBackendState, work::CaptureWork)
    if get_mjpeg_client_count(state.server, state.camera_name) == 0
        return
    end

    jpeg_bytes = encode_jpeg_frame(work.rgb_data, work.width, work.height)
    broadcast_mjpeg_frame!(state.server, state.camera_name, jpeg_bytes)
end

"""
    cleanup_backend!(backend::MJPEGOutput, state::MJPEGBackendState)

Cleanup MJPEG backend.
"""
function cleanup_backend!(backend::MJPEGOutput, state::MJPEGBackendState)
    state.running = false

    clients = @lock state.clients_lock copy(state.clients)
    for client in clients
        try
            close(client)
        catch e
            @warn "MJPEGOutput($(state.camera_name)): error closing client" exception = e
        end
    end

    notify(state.shutdown_condition, all = true)

    println("MJPEGOutput($(state.camera_name)): shutdown")
end

"""
    cleanup_backend!(backend::MJPEGOutput, state::UnifiedMJPEGBackendState)

Cleanup MJPEG unified backend (no-op; unified server owns lifecycle).
"""
function cleanup_backend!(backend::MJPEGOutput, state::UnifiedMJPEGBackendState)
    println("MJPEGOutput($(state.camera_name)): detached from unified server")
end
