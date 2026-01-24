# VideoOutput backend - pipe frames to FFMPEG for video encoding

"""
    VideoBackendState

State for video output backend.
Manages FFMPEG process and stdin pipe.
"""
mutable struct VideoBackendState
    path::String
    camera_name::String
    process::Union{Base.Process, Nothing}
    stdin_pipe::Union{IO, Nothing}
    width::Int
    height::Int
    running::Bool
end

"""
    init_backend(backend::VideoOutput, camera_name::String, width::Int, height::Int, fps::Float64)

Initialize video output backend. Spawns FFMPEG process with fragmented MP4 for crash safety.
"""
function init_backend(
        backend::VideoOutput, camera_name::String, width::Int, height::Int, fps::Float64)
    # Ensure output directory exists
    output_dir = dirname(backend.path)
    if !isempty(output_dir)
        mkpath(output_dir)
    end

    # Build FFMPEG command
    # -f rawvideo: input is raw video
    # -pix_fmt rgb24: 8-bit RGB
    # -s WxH: frame size
    # -r fps: frame rate
    # -i pipe:0: read from stdin
    # -c:v codec: video codec
    # -preset: encoding speed/quality tradeoff
    # -crf: quality (lower = better, 0-51 for x264)
    # -pix_fmt yuv420p: output pixel format (widely compatible)
    # -movflags frag_keyframe+empty_moov: fragmented MP4 for crash safety
    cmd = `ffmpeg -y -f rawvideo -pix_fmt rgb24 -s $(width)x$(height) -r $(fps) 
           -i pipe:0 
           -c:v $(backend.codec) -preset $(backend.preset) -crf $(backend.crf)
           -pix_fmt yuv420p 
           -movflags frag_keyframe+empty_moov 
           $(backend.path)`

    # Spawn FFMPEG process
    process = open(cmd, "w")

    state = VideoBackendState(
        backend.path,
        camera_name,
        process,
        process,  # stdin is the process itself for pipeline commands
        width,
        height,
        true
    )

    println("VideoOutput: recording $(camera_name) to $(backend.path) ($(width)x$(height) @ $(fps)fps, $(backend.codec) $(backend.preset) crf=$(backend.crf))")

    return state
end

"""
    process_frame!(backend::VideoOutput, state::VideoBackendState, work::CaptureWork)

Write raw RGB frame to FFMPEG stdin pipe.
"""
function process_frame!(backend::VideoOutput, state::VideoBackendState, work::CaptureWork)
    if !state.running || state.stdin_pipe === nothing
        return
    end

    try
        # Write raw RGB data directly - no encoding overhead!
        # Data is already flipped to top-to-bottom order
        write(state.stdin_pipe, work.rgb_data)
    catch e
        if state.running
            @warn "VideoOutput($(state.camera_name)): write error" exception=e
            state.running = false
        end
    end
end

"""
    cleanup_backend!(backend::VideoOutput, state::VideoBackendState)

Cleanup video backend. Closes pipe and waits for FFMPEG to finish.
"""
function cleanup_backend!(backend::VideoOutput, state::VideoBackendState)
    state.running = false

    if state.stdin_pipe !== nothing
        try
            close(state.stdin_pipe)
        catch e
            @warn "VideoOutput($(state.camera_name)): error closing pipe" exception=e
        end
        state.stdin_pipe = nothing
    end

    if state.process !== nothing
        try
            wait(state.process)
            println("VideoOutput($(state.camera_name)): finished writing $(state.path)")
        catch e
            @warn "VideoOutput($(state.camera_name)): error waiting for ffmpeg" exception=e
        end
        state.process = nothing
    end
end
