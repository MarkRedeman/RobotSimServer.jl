# CaptureManager - orchestrates multi-camera capture

using MuJoCo
using MuJoCo.LibMuJoCo

"""
    CaptureManager

Manages multi-camera capture with various output backends.
Orchestrates rendering, buffer management, and async I/O.
"""
mutable struct CaptureManager
    config::CaptureConfig
    worker::CaptureWorker

    # MuJoCo camera objects (one per camera spec)
    mj_cameras::Vector{Any}

    # Camera modes and fixed camera IDs
    camera_modes::Vector{Symbol}
    fixed_cam_ids::Vector{Int}

    # Backend states (one per camera)
    backend_states::Vector{Any}

    # Shared RGB buffer for rendering
    rgb_buffer::Vector{UInt8}
    rect::mjrRect

    # Timing
    last_capture_time::Float64
    start_time::Float64
    capture_interval::Float64

    # Frame counter
    frame_count::Int

    # Control
    enabled::Bool
end

"""
    init_capture_manager(config::CaptureConfig, model=nothing) -> CaptureManager

Initialize the capture manager with the given configuration.
Creates camera objects and initializes all backends.

# Arguments
- `config`: CaptureConfig specifying cameras and output backends
- `model`: MuJoCo Model (required for fixed cameras to lookup camera IDs)
"""
function init_capture_manager(config::CaptureConfig, model = nothing)
    width, height = config.width, config.height
    fps = config.fps

    # Create MuJoCo camera objects
    mj_cameras = []
    camera_modes = Symbol[]
    fixed_cam_ids = Int[]

    for spec in config.cameras
        cam = MuJoCo.VisualiserCamera()

        if spec.mode == :fixed
            # Fixed camera - attached to body, defined in model
            if model === nothing
                error("Model required for fixed camera '$(spec.name)' (model_camera='$(spec.model_camera)')")
            end
            if spec.model_camera === nothing
                error("model_camera must be specified for fixed camera '$(spec.name)'")
            end

            # Look up camera ID in model
            cam_id = mj_name2id(model, Int32(LibMuJoCo.mjOBJ_CAMERA), spec.model_camera)
            if cam_id < 0
                error("Camera not found in model: $(spec.model_camera)")
            end

            # Set up as fixed camera
            cam.type = Int32(LibMuJoCo.mjCAMERA_FIXED)
            cam.fixedcamid = cam_id

            push!(camera_modes, :fixed)
            push!(fixed_cam_ids, cam_id)
        else
            # Free camera - positioned in world space
            cam.lookat .= spec.lookat
            cam.distance = spec.distance
            cam.azimuth = spec.azimuth
            cam.elevation = spec.elevation
            cam.type = Int32(LibMuJoCo.mjCAMERA_FREE)

            push!(camera_modes, :free)
            push!(fixed_cam_ids, -1)
        end

        push!(mj_cameras, cam)
    end

    # Initialize backends
    backend_states = []
    for spec in config.cameras
        state = init_backend(spec.output, spec.name, width, height, fps)
        push!(backend_states, state)
    end

    # Create shared RGB buffer
    rgb_buffer = zeros(UInt8, 3 * width * height)
    rect = mjrRect(Cint(0), Cint(0), Cint(width), Cint(height))

    # Create and start worker
    worker = create_worker()
    start_worker!(worker)

    manager = CaptureManager(
        config,
        worker,
        mj_cameras,
        camera_modes,
        fixed_cam_ids,
        backend_states,
        rgb_buffer,
        rect,
        0.0,          # last_capture_time
        time(),       # start_time
        1.0 / fps,    # capture_interval
        0,            # frame_count
        true          # enabled
    )

    num_free = count(==(:free), camera_modes)
    num_fixed = count(==(:fixed), camera_modes)
    println("CaptureManager: initialized $(length(config.cameras)) cameras ($(num_free) free, $(num_fixed) fixed) @ $(width)x$(height) $(fps)fps")

    return manager
end

"""
    flip_buffer!(src::Vector{UInt8}, width::Int, height::Int) -> Vector{UInt8}

Create a vertically flipped copy of the RGB buffer.
OpenGL renders bottom-to-top, but we need top-to-bottom.
"""
function flip_buffer(src::Vector{UInt8}, width::Int, height::Int)
    dst = Vector{UInt8}(undef, length(src))
    row_bytes = 3 * width

    for j in 1:height
        src_offset = (j - 1) * row_bytes + 1
        dst_offset = (height - j) * row_bytes + 1
        dst[dst_offset:(dst_offset + row_bytes - 1)] = src[src_offset:(src_offset + row_bytes - 1)]
    end

    return dst
end

"""
    capture_camera!(manager::CaptureManager, engine, cam_idx::Int, elapsed_time::Float64)

Render a single camera view and submit to async worker.
Must be called from the UI thread.
"""
function capture_camera!(
        manager::CaptureManager, engine, cam_idx::Int, elapsed_time::Float64)
    spec = manager.config.cameras[cam_idx]
    mj_cam = manager.mj_cameras[cam_idx]

    # Update orbiting camera azimuth (only for free cameras)
    if manager.camera_modes[cam_idx] == :free && spec.orbiting
        mj_cam.azimuth = mod(spec.azimuth + spec.orbit_speed * elapsed_time, 360.0)
    end

    # Update scene with this camera's view
    LibMuJoCo.mjv_updateScene(
        engine.phys.model,
        engine.phys.data,
        engine.ui.vopt,
        engine.phys.pert,
        mj_cam,
        Int32(LibMuJoCo.mjCAT_ALL),
        engine.ui.scn
    )

    # Render to offscreen buffer
    LibMuJoCo.mjr_setBuffer(Int32(LibMuJoCo.mjFB_OFFSCREEN), engine.ui.con)
    LibMuJoCo.mjr_render(manager.rect, engine.ui.scn, engine.ui.con)

    # Read pixels into our buffer
    LibMuJoCo.mjr_readPixels(manager.rgb_buffer, C_NULL, manager.rect, engine.ui.con)

    # Create flipped copy for async processing
    flipped_rgb = flip_buffer(
        manager.rgb_buffer, manager.config.width, manager.config.height)

    # Submit work to async worker
    work = CaptureWork(
        spec.name,
        manager.frame_count,
        flipped_rgb,
        manager.config.width,
        manager.config.height,
        spec.output,
        manager.backend_states[cam_idx],
        time()
    )

    submit_work!(manager.worker, work)
end

"""
    maybe_capture!(manager::CaptureManager, engine)

Check if it's time to capture and capture all cameras if so.
Called from the UI thread render loop.
"""
function maybe_capture!(manager::CaptureManager, engine)
    if !manager.enabled
        return
    end

    # Wall clock timing (works even when sim is paused)
    current_time = time()

    if current_time < manager.last_capture_time + manager.capture_interval
        return
    end

    # Calculate elapsed time for orbiting cameras
    elapsed_time = current_time - manager.start_time

    # Increment frame counter
    manager.frame_count += 1

    # Capture from all cameras
    for i in eachindex(manager.mj_cameras)
        capture_camera!(manager, engine, i, elapsed_time)
    end

    manager.last_capture_time = current_time

    # Restore main UI camera view for display window
    LibMuJoCo.mjv_updateScene(
        engine.phys.model,
        engine.phys.data,
        engine.ui.vopt,
        engine.phys.pert,
        engine.ui.cam,
        Int32(LibMuJoCo.mjCAT_ALL),
        engine.ui.scn
    )

    # Switch back to window buffer
    LibMuJoCo.mjr_setBuffer(Int32(LibMuJoCo.mjFB_WINDOW), engine.ui.con)
end

"""
    cleanup_capture_manager!(manager::CaptureManager)

Gracefully shutdown the capture manager.
Stops worker and cleans up all backends.
"""
function cleanup_capture_manager!(manager::CaptureManager)
    manager.enabled = false

    # Stop the worker (processes remaining items)
    stop_worker!(manager.worker)

    # Cleanup all backends
    for (i, spec) in enumerate(manager.config.cameras)
        cleanup_backend!(spec.output, manager.backend_states[i])
    end

    println("CaptureManager: shutdown complete ($(manager.frame_count) frames captured)")
end
