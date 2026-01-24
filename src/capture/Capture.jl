# Multi-camera capture system for MuJoCo.jl
#
# Provides flexible per-camera output backends:
# - FileOutput: Save frames as JPEG/PNG to disk
# - WebSocketOutput: Stream frames to WebSocket clients
# - VideoOutput: Pipe to FFMPEG for crash-safe video recording
#
# Supports two camera modes:
# - :free - Freely positioned cameras (lookat, distance, azimuth, elevation)
# - :fixed - Model-defined cameras attached to bodies (body-mounted cameras)
#
# Usage:
#   include("capture/capture.jl")
#   
#   config = CaptureConfig(
#       width = 640,
#       height = 480,
#       fps = 10.0,
#       cameras = [
#           CameraSpec(name="front", azimuth=180.0, output=FileOutput("output/front")),
#           CameraSpec(name="orbit", orbiting=true, output=VideoOutput("output/orbit.mp4")),
#           CameraSpec(name="gripper", mode=:fixed, model_camera="gripper_cam", 
#                      output=VideoOutput("output/gripper.mp4")),
#       ]
#   )
#   
#   run_with_capture!(model, data, capture=config)

using MuJoCo
using MuJoCo.LibMuJoCo
using HTTP
using HTTP.WebSockets
using Images
using FileIO

# Include all components
include("types.jl")
include("worker.jl")
include("backends/file.jl")
include("backends/websocket.jl")
include("backends/video.jl")
include("manager.jl")

# Get access to VisualiserExt internals
function get_visualiser_ext()
    ext = Base.get_extension(MuJoCo, :VisualiserExt)
    if ext === nothing
        error("VisualiserExt not loaded. Call init_visualiser() first.")
    end
    return ext
end

"""
    run_with_capture!(model, data; controller=nothing, capture=CaptureConfig(...))

Run the MuJoCo visualizer with multi-camera capture.
This is a replacement for `visualise!()` that adds capture functionality.

# Arguments
- `model`: MuJoCo Model
- `data`: MuJoCo Data
- `controller`: Optional controller function called at each physics step
- `capture`: CaptureConfig specifying cameras and output backends
"""
function run_with_capture!(model::MuJoCo.Model, data::MuJoCo.Data;
        controller = nothing,
        capture::CaptureConfig)
    ext = get_visualiser_ext()

    # Get types and functions from extension
    Engine = ext.Engine
    Controller = ext.Controller
    PassiveDynamics = ext.PassiveDynamics
    prepare! = ext.prepare!
    render! = ext.render!
    runphysics! = ext.runphysics!
    default_windowsize = ext.default_windowsize
    GetRefreshRate = ext.GetRefreshRate
    RNDGAMMA = ext.RNDGAMMA
    GLFW = ext.GLFW

    # Build engine modes
    modes = []
    if controller !== nothing
        push!(modes, Controller(controller))
    end
    push!(modes, PassiveDynamics())

    # Create engine with default window size
    window_size = default_windowsize()
    engine = Engine(window_size, model, data, Tuple(modes))

    # Initialize capture manager (pass model for fixed camera lookups)
    manager = init_capture_manager(capture, model)

    # Render first frame
    prepare!(engine)
    engine.ui.refreshrate = GetRefreshRate()
    engine.ui.lastrender = time()
    GLFW.ShowWindow(engine.manager.state.window)

    # Run physics in separate thread
    modetask = Threads.@spawn runphysics!(engine)

    # Print info
    println(ext.ASCII)
    println("Press \"F1\" to show the help message.")
    println("Multi-camera capture active: $(length(capture.cameras)) cameras @ $(capture.fps) fps")

    # UI loop with capture hook
    shouldexit = false
    try
        while !shouldexit
            # Poll events and prepare visualization
            @lock engine.phys.lock begin
                GLFW.PollEvents()
                prepare!(engine)
            end

            # Render main window
            render!(engine)

            # Multi-camera capture
            @lock engine.phys.lock begin
                maybe_capture!(manager, engine)
            end

            trender = time()

            # Update refresh rate
            rt = 1 / (trender - engine.ui.lastrender)
            @lock engine.ui.lock begin
                engine.ui.refreshrate = RNDGAMMA * engine.ui.refreshrate +
                                        (1 - RNDGAMMA) * rt
                engine.ui.lastrender = trender
                shouldexit = engine.ui.shouldexit |
                             GLFW.WindowShouldClose(engine.manager.state.window)
            end

            yield()
        end
    finally
        # Cleanup
        @lock engine.ui.lock begin
            engine.ui.shouldexit = true
        end
        GLFW.DestroyWindow(engine.manager.state.window)

        # Cleanup capture manager
        cleanup_capture_manager!(manager)
    end

    wait(modetask)

    return nothing
end
