# HeadlessRenderer.jl - Offscreen OpenGL rendering for headless MuJoCo simulations
#
# Provides headless (no GUI window) rendering capability for MuJoCo simulations.
# Uses a hidden GLFW window to create an OpenGL context for offscreen rendering.
#
# Usage:
#   include("src/HeadlessRenderer.jl")
#
#   renderer = HeadlessRenderer(model, 640, 480)
#   rgb_data = render_camera!(renderer, data, camera)
#   cleanup!(renderer)

using MuJoCo
using MuJoCo.LibMuJoCo
using GLFW

# =============================================================================
# Types
# =============================================================================

"""
    HeadlessRenderer

Manages offscreen OpenGL rendering for MuJoCo without a visible window.

Uses a hidden GLFW window to provide an OpenGL context for MuJoCo's
offscreen rendering. Each HeadlessRenderer is independent and can be
used in parallel (each needs its own thread/task due to OpenGL context).

# Fields
- `window::GLFW.Window`: Hidden GLFW window for GL context
- `context::MuJoCo.RendererContext`: MuJoCo rendering context
- `scene::MuJoCo.VisualiserScene`: MuJoCo scene for rendering
- `vopt::MuJoCo.VisualiserOption`: Visualization options
- `width::Int`: Framebuffer width in pixels
- `height::Int`: Framebuffer height in pixels
- `rgb_buffer::Vector{UInt8}`: Preallocated RGB buffer for readback
- `rect::mjrRect`: Rendering viewport rectangle
- `initialized::Bool`: Whether renderer is ready for use
"""
mutable struct HeadlessRenderer
    window::GLFW.Window
    context::MuJoCo.RendererContext
    scene::MuJoCo.VisualiserScene
    vopt::MuJoCo.VisualiserOption
    width::Int
    height::Int
    rgb_buffer::Vector{UInt8}
    rect::mjrRect
    initialized::Bool
end

# =============================================================================
# Initialization
# =============================================================================

"""
    HeadlessRenderer(model::MuJoCo.Model, width::Int=640, height::Int=480) -> HeadlessRenderer

Create a new headless renderer for the given MuJoCo model.

Initializes GLFW (if needed), creates a hidden window for the OpenGL context,
and sets up all MuJoCo rendering structures.

# Arguments
- `model`: MuJoCo Model to render
- `width`: Framebuffer width in pixels (default: 640)
- `height`: Framebuffer height in pixels (default: 480)

# Returns
- `HeadlessRenderer`: Ready-to-use renderer instance

# Example
```julia
model = load_model("robot.xml")
renderer = HeadlessRenderer(model, 640, 480)
```

# Notes
- Each renderer gets its own OpenGL context
- Multiple renderers can exist but each should be used from a single thread
- Call `cleanup!(renderer)` when done to free resources
"""
function HeadlessRenderer(model::MuJoCo.Model, width::Int = 640, height::Int = 480)
    # Initialize GLFW if needed
    if !GLFW.Init()
        error("Failed to initialize GLFW")
    end

    # Create hidden window for GL context
    # Note: Don't set explicit GL version hints - MuJoCo's mjr_makeContext fails with
    # "ARB_framebuffer_object required" when version hints are set, even on GL 4.6 systems.
    # Let GLFW pick the best available version instead.
    GLFW.DefaultWindowHints()
    GLFW.WindowHint(GLFW.VISIBLE, 0)
    window = GLFW.CreateWindow(width, height, "HeadlessRenderer")
    if window == C_NULL
        error("Failed to create GLFW window")
    end
    GLFW.MakeContextCurrent(window)

    # Create MuJoCo rendering context
    context = MuJoCo.RendererContext()
    mjr_makeContext(model.internal_pointer, context.internal_pointer, Int32(mjFONTSCALE_150))

    # Create scene
    scene = MuJoCo.VisualiserScene()
    mjv_defaultScene(scene.internal_pointer)
    mjv_makeScene(model.internal_pointer, scene.internal_pointer, 1000)

    # Create visualization options
    vopt = MuJoCo.VisualiserOption()

    # Preallocate RGB buffer
    rgb_buffer = zeros(UInt8, 3 * width * height)
    rect = mjrRect(Cint(0), Cint(0), Cint(width), Cint(height))

    renderer = HeadlessRenderer(
        window,
        context,
        scene,
        vopt,
        width,
        height,
        rgb_buffer,
        rect,
        true
    )

    println("HeadlessRenderer initialized: $(width)x$(height)")
    return renderer
end

# =============================================================================
# Rendering
# =============================================================================

"""
    render_camera!(renderer::HeadlessRenderer, model::MuJoCo.Model, data::MuJoCo.Data,
                   camera::MuJoCo.VisualiserCamera) -> Vector{UInt8}

Render a single frame from the given camera viewpoint.

Updates the scene with current simulation state, renders to the offscreen
buffer, and returns the RGB pixel data.

# Arguments
- `renderer`: HeadlessRenderer instance
- `model`: MuJoCo Model
- `data`: MuJoCo Data with current simulation state
- `camera`: Camera defining the viewpoint

# Returns
- `Vector{UInt8}`: RGB pixel data (height * width * 3 bytes), bottom-to-top row order

# Example
```julia
camera = MuJoCo.VisualiserCamera()
camera.distance = 1.5
camera.lookat .= [0, 0, 0.2]

rgb_data = render_camera!(renderer, model, data, camera)
```
"""
function render_camera!(renderer::HeadlessRenderer, model::MuJoCo.Model,
        data::MuJoCo.Data, camera::MuJoCo.VisualiserCamera)
    if !renderer.initialized
        error("HeadlessRenderer not initialized")
    end

    # Make our context current (important for multi-renderer scenarios)
    GLFW.MakeContextCurrent(renderer.window)

    # Update scene with current simulation state
    mjv_updateScene(
        model.internal_pointer,
        data.internal_pointer,
        renderer.vopt.internal_pointer,
        C_NULL,  # No perturbations
        camera.internal_pointer,
        Int32(mjCAT_ALL),
        renderer.scene.internal_pointer
    )

    # Render to offscreen buffer
    mjr_setBuffer(Int32(mjFB_OFFSCREEN), renderer.context.internal_pointer)
    mjr_render(renderer.rect, renderer.scene.internal_pointer, renderer.context.internal_pointer)

    # Read pixels into buffer
    mjr_readPixels(renderer.rgb_buffer, C_NULL, renderer.rect, renderer.context.internal_pointer)

    # Return copy of buffer (caller may process asynchronously)
    return copy(renderer.rgb_buffer)
end

"""
    render_camera_flipped!(renderer::HeadlessRenderer, model::MuJoCo.Model,
                           data::MuJoCo.Data, camera::MuJoCo.VisualiserCamera) -> Vector{UInt8}

Render a frame and return with rows flipped (top-to-bottom order).

OpenGL renders bottom-to-top, but image formats typically use top-to-bottom.
This function handles the conversion.

# Arguments
- `renderer`: HeadlessRenderer instance
- `model`: MuJoCo Model
- `data`: MuJoCo Data with current simulation state
- `camera`: Camera defining the viewpoint

# Returns
- `Vector{UInt8}`: RGB pixel data in top-to-bottom row order
"""
function render_camera_flipped!(renderer::HeadlessRenderer, model::MuJoCo.Model,
        data::MuJoCo.Data, camera::MuJoCo.VisualiserCamera)
    rgb = render_camera!(renderer, model, data, camera)
    return flip_rgb_buffer(rgb, renderer.width, renderer.height)
end

"""
    flip_rgb_buffer(src::Vector{UInt8}, width::Int, height::Int) -> Vector{UInt8}

Flip an RGB buffer vertically (convert between top-to-bottom and bottom-to-top).

# Arguments
- `src`: Source RGB buffer
- `width`: Image width in pixels
- `height`: Image height in pixels

# Returns
- `Vector{UInt8}`: Vertically flipped RGB buffer
"""
function flip_rgb_buffer(src::Vector{UInt8}, width::Int, height::Int)
    dst = Vector{UInt8}(undef, length(src))
    row_bytes = 3 * width

    for j in 1:height
        src_offset = (j - 1) * row_bytes + 1
        dst_offset = (height - j) * row_bytes + 1
        dst[dst_offset:(dst_offset + row_bytes - 1)] = src[src_offset:(src_offset + row_bytes - 1)]
    end

    return dst
end

# =============================================================================
# Camera Helpers
# =============================================================================

"""
    create_free_camera(; lookat=[0,0,0.2], distance=1.5, azimuth=180.0,
                         elevation=-20.0) -> MuJoCo.VisualiserCamera

Create a free camera with the specified parameters.

# Arguments
- `lookat`: 3D point the camera looks at
- `distance`: Distance from lookat point
- `azimuth`: Horizontal angle in degrees
- `elevation`: Vertical angle in degrees

# Returns
- `MuJoCo.VisualiserCamera`: Configured camera
"""
function create_free_camera(;
        lookat::Vector{<:Real} = [0.0, 0.0, 0.2],
        distance::Real = 1.5,
        azimuth::Real = 180.0,
        elevation::Real = -20.0)
    camera = MuJoCo.VisualiserCamera()
    camera.type = Int32(mjCAMERA_FREE)
    camera.lookat .= lookat
    camera.distance = distance
    camera.azimuth = azimuth
    camera.elevation = elevation
    return camera
end

"""
    create_fixed_camera(model::MuJoCo.Model, camera_name::String) -> MuJoCo.VisualiserCamera

Create a fixed camera that uses a camera defined in the MuJoCo model.

# Arguments
- `model`: MuJoCo Model containing the camera definition
- `camera_name`: Name of the camera in the model

# Returns
- `MuJoCo.VisualiserCamera`: Camera configured to use the model camera

# Throws
- `ArgumentError`: If camera name not found in model
"""
function create_fixed_camera(model::MuJoCo.Model, camera_name::String)
    cam_id = mj_name2id(model, Int32(mjOBJ_CAMERA), camera_name)
    if cam_id < 0
        throw(ArgumentError("Camera not found in model: $camera_name"))
    end

    camera = MuJoCo.VisualiserCamera()
    camera.type = Int32(mjCAMERA_FIXED)
    camera.fixedcamid = cam_id
    return camera
end

# =============================================================================
# Cleanup
# =============================================================================

"""
    cleanup!(renderer::HeadlessRenderer)

Free all resources associated with the renderer.

Should be called when the renderer is no longer needed to release
OpenGL resources and close the hidden GLFW window.

# Arguments
- `renderer`: HeadlessRenderer to clean up
"""
function cleanup!(renderer::HeadlessRenderer)
    if !renderer.initialized
        return
    end

    renderer.initialized = false

    # Free MuJoCo scene
    try
        mjv_freeScene(renderer.scene.internal_pointer)
    catch
    end

    # Free MuJoCo context
    try
        mjr_freeContext(renderer.context.internal_pointer)
    catch
    end

    # Destroy GLFW window
    try
        GLFW.DestroyWindow(renderer.window)
    catch
    end

    println("HeadlessRenderer cleaned up")
end

# =============================================================================
# Utility Functions
# =============================================================================

"""
    get_framebuffer_size(renderer::HeadlessRenderer) -> Tuple{Int, Int}

Get the framebuffer dimensions.

# Returns
- `(width, height)`: Framebuffer size in pixels
"""
function get_framebuffer_size(renderer::HeadlessRenderer)
    return (renderer.width, renderer.height)
end

"""
    is_initialized(renderer::HeadlessRenderer) -> Bool

Check if the renderer is initialized and ready for use.
"""
function is_initialized(renderer::HeadlessRenderer)
    return renderer.initialized
end
