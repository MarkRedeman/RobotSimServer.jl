# Async I/O worker for processing captured frames

"""
    CaptureWorker

Manages an async task that processes captured frames.
Uses a bounded channel to prevent memory buildup if processing falls behind.
"""
mutable struct CaptureWorker
    channel::Channel{CaptureWork}
    task::Union{Task, Nothing}
    running::Bool
end

"""
    create_worker(; buffer_size=32) -> CaptureWorker

Create a new capture worker with a bounded channel.
"""
function create_worker(; buffer_size::Int = 32)
    return CaptureWorker(
        Channel{CaptureWork}(buffer_size),
        nothing,
        false
    )
end

"""
    start_worker!(worker::CaptureWorker)

Start the async worker task that processes frames.
"""
function start_worker!(worker::CaptureWorker)
    if worker.running
        @warn "Worker already running"
        return
    end

    worker.running = true
    worker.task = @async begin
        while worker.running || isready(worker.channel)
            try
                # Wait for work with timeout to allow graceful shutdown
                work = nothing
                while work === nothing && (worker.running || isready(worker.channel))
                    if isready(worker.channel)
                        work = take!(worker.channel)
                    else
                        sleep(0.001)  # 1ms poll interval
                    end
                end

                if work !== nothing
                    process_work!(work)
                end
            catch e
                if !(e isa InvalidStateException)  # Channel closed
                    @warn "Worker error processing frame" exception=(e, catch_backtrace())
                end
            end
        end
    end
end

"""
    stop_worker!(worker::CaptureWorker)

Stop the worker gracefully, processing remaining items.
"""
function stop_worker!(worker::CaptureWorker)
    worker.running = false

    if worker.task !== nothing
        # Wait for task to finish processing remaining items
        try
            wait(worker.task)
        catch e
            @warn "Error waiting for worker task" exception=e
        end
        worker.task = nothing
    end
end

"""
    submit_work!(worker::CaptureWorker, work::CaptureWork) -> Bool

Submit work to the worker. Returns false if channel is full (frame dropped).
"""
function submit_work!(worker::CaptureWorker, work::CaptureWork)
    if !worker.running
        return false
    end

    # Non-blocking push - drop frame if channel is full
    if isready(worker.channel) || !isfull(worker.channel)
        try
            put!(worker.channel, work)
            return true
        catch e
            @warn "Failed to submit work" exception=e
            return false
        end
    else
        @warn "Capture worker channel full, dropping frame $(work.frame_num) for $(work.camera_name)"
        return false
    end
end

# Helper to check if channel is full
function isfull(ch::Channel)
    return length(ch.data) >= ch.sz_max
end

"""
    process_work!(work::CaptureWork)

Process a single work item by dispatching to the appropriate backend.
"""
function process_work!(work::CaptureWork)
    try
        process_frame!(work.backend, work.backend_state, work)
    catch e
        @warn "Failed to process frame $(work.frame_num) for $(work.camera_name)" exception=(
            e, catch_backtrace())
    end
end
