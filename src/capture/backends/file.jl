# FileOutput backend - save frames as JPEG/PNG to disk

using Images
using FileIO

"""
    FileBackendState

State for file output backend.
"""
mutable struct FileBackendState
    dir::String
    format::Symbol
end

"""
    init_backend(backend::FileOutput, camera_name::String, width::Int, height::Int, fps::Float64)

Initialize file output backend. Creates the output directory.
"""
function init_backend(
        backend::FileOutput, camera_name::String, width::Int, height::Int, fps::Float64)
    # Create output directory
    mkpath(backend.dir)
    println("FileOutput: saving $(backend.format) frames to $(backend.dir)")

    return FileBackendState(backend.dir, backend.format)
end

"""
    process_frame!(backend::FileOutput, state::FileBackendState, work::CaptureWork)

Encode and save a frame to disk.
"""
function process_frame!(backend::FileOutput, state::FileBackendState, work::CaptureWork)
    # Convert raw RGB buffer to image
    # Data is already flipped (bottom-to-top -> top-to-bottom)
    img = rgb_to_image(work.rgb_data, work.width, work.height)

    # Generate filename
    ext = state.format == :png ? "png" : "jpg"
    filename = joinpath(state.dir, "frame_$(lpad(work.frame_num, 6, '0')).$(ext)")

    # Save image
    save(filename, img)
end

"""
    cleanup_backend!(backend::FileOutput, state::FileBackendState)

Cleanup file backend. Nothing to do.
"""
function cleanup_backend!(backend::FileOutput, state::FileBackendState)
    # Nothing to cleanup
end

"""
    rgb_to_image(rgb_data::Vector{UInt8}, width::Int, height::Int)

Convert raw RGB buffer to Images.jl RGB image.
Assumes data is already vertically flipped (top-to-bottom order).
"""
function rgb_to_image(rgb_data::Vector{UInt8}, width::Int, height::Int)
    # Reshape: 3 channels x width x height
    rgb_reshaped = reshape(rgb_data, 3, width, height)

    # Convert to N0f8 (normalized 0-1 fixed point)
    rgb_n0f8 = reinterpret(N0f8, rgb_reshaped)

    # Build RGB image matrix
    img = Matrix{RGB{N0f8}}(undef, height, width)
    for j in 1:height
        for i in 1:width
            img[j, i] = RGB(rgb_n0f8[1, i, j], rgb_n0f8[2, i, j], rgb_n0f8[3, i, j])
        end
    end

    return img
end
