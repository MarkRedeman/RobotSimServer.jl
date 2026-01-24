# Type definitions for multi-camera capture system

"""
    OutputBackend

Abstract type for camera output backends.
Concrete implementations: FileOutput, WebSocketOutput, VideoOutput
"""
abstract type OutputBackend end

"""
    FileOutput

Save frames as JPEG or PNG files to disk.

# Fields
- `dir::String`: Output directory path
- `format::Symbol`: Image format, either :jpeg or :png (default: :jpeg)
"""
Base.@kwdef struct FileOutput <: OutputBackend
    dir::String
    format::Symbol = :jpeg
end

"""
    WebSocketOutput

Stream frames as raw JPEG bytes to WebSocket clients.
Multiple clients can connect to the same port (broadcast mode).

# Fields
- `port::Int`: WebSocket server port
"""
Base.@kwdef struct WebSocketOutput <: OutputBackend
    port::Int
end

"""
    VideoOutput

Pipe frames to FFMPEG for video encoding.
Uses fragmented MP4 for crash-safe recording.

# Fields
- `path::String`: Output video file path
- `codec::String`: Video codec (default: "libx264")
- `preset::String`: Encoding preset (default: "ultrafast")
- `crf::Int`: Constant rate factor, lower = better quality (default: 23)
"""
Base.@kwdef struct VideoOutput <: OutputBackend
    path::String
    codec::String = "libx264"
    preset::String = "ultrafast"
    crf::Int = 23
end

"""
    CameraSpec

Specification for a single camera view with its output backend.

Supports two modes:
- `:free` - Freely positioned camera (default). Uses lookat, distance, azimuth, elevation.
- `:fixed` - Model-defined camera attached to a body. Uses `model_camera` name.

# Fields
- `name::String`: Unique camera identifier
- `mode::Symbol`: Camera mode, either :free or :fixed (default: :free)
- `model_camera::Union{String, Nothing}`: Name of model-defined camera (for :fixed mode)
- `lookat::Vector{Float64}`: 3D point the camera looks at (for :free mode)
- `distance::Float64`: Distance from lookat point (for :free mode)
- `azimuth::Float64`: Horizontal angle in degrees (for :free mode)
- `elevation::Float64`: Vertical angle in degrees (for :free mode)
- `orbiting::Bool`: If true, azimuth changes over time (for :free mode)
- `orbit_speed::Float64`: Rotation speed in degrees per second (for :free mode)
- `output::OutputBackend`: Where to send captured frames
"""
Base.@kwdef struct CameraSpec
    name::String
    mode::Symbol = :free
    model_camera::Union{String, Nothing} = nothing
    lookat::Vector{Float64} = [0.0, 0.0, 0.2]
    distance::Float64 = 1.2
    azimuth::Float64 = 180.0
    elevation::Float64 = -20.0
    orbiting::Bool = false
    orbit_speed::Float64 = 0.0
    output::OutputBackend
end

"""
    CaptureConfig

Global configuration for the capture system.
Resolution and FPS are shared across all cameras.

# Fields
- `width::Int`: Frame width in pixels
- `height::Int`: Frame height in pixels
- `fps::Float64`: Capture rate in frames per second
- `cameras::Vector{CameraSpec}`: List of camera specifications
"""
Base.@kwdef struct CaptureConfig
    width::Int = 640
    height::Int = 480
    fps::Float64 = 10.0
    cameras::Vector{CameraSpec}
end

"""
    CaptureWork

A unit of work for the async I/O worker.
Contains a copy of the frame data and metadata.
"""
struct CaptureWork
    camera_name::String
    frame_num::Int
    rgb_data::Vector{UInt8}  # Copy of RGB buffer (already flipped)
    width::Int
    height::Int
    backend::OutputBackend
    backend_state::Any       # Backend-specific state (file handle, WS clients, etc.)
    timestamp::Float64
end
