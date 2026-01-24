using MuJoCo
using MuJoCo.LibMuJoCo
using HTTP
using JSON
using Dates
using Sockets
using Images
using FileIO
using GLFW

# Path to the SO101 model
xml_path = joinpath(
    @__DIR__, "..", "..", "robots", "SO-ARM100", "Simulation", "SO101", "scene.xml")

# Create output directory
mkpath(joinpath(@__DIR__, "..", "..", "output"))

# Load the SO101 model and data
model = load_model(xml_path)
data = init_data(model)

# --- Offscreen Rendering Setup ---
width, height = 640, 480
vopt = MuJoCo.VisualiserOption()
scn = MuJoCo.VisualiserScene()
mjv_defaultScene(scn.internal_pointer)
mjv_makeScene(model.internal_pointer, scn.internal_pointer, 1000)

cam = MuJoCo.VisualiserCamera()
cam.type = Int32(mjCAMERA_FREE)
cam.distance = 1.5
cam.lookat .= [0.0, 0.0, 0.2]

# Map actuator names to indices
actuator_names = [unsafe_string(mj_id2name(
                      model, Int32(MuJoCo.LibMuJoCo.mjOBJ_ACTUATOR), i - 1))
                  for i in 1:(model.nu)]
actuator_map = Dict(name => i for (i, name) in enumerate(actuator_names))

# Global state
control_channel = Channel{Dict{String, Any}}(10)
rgb_buffer = zeros(UInt8, 3 * width * height)
rect = mjrRect(Cint(0), Cint(0), Cint(width), Cint(height))

# OpenGL Setup
GLFW.Init()
GLFW.WindowHint(GLFW.VISIBLE, 0)
window = GLFW.CreateWindow(width, height, "Hidden")
GLFW.MakeContextCurrent(window)

con = MuJoCo.RendererContext()
mjr_makeContext(model.internal_pointer, con.internal_pointer, Int32(mjFONTSCALE_150))

# WebSocket server
@async begin
    HTTP.WebSockets.listen("0.0.0.0", 8081) do ws
        println("WebSocket client connected")
        for msg in ws
            try
                raw_data = JSON.parse(String(msg))
                put!(control_channel, raw_data)
            catch e
                @warn "WebSocket error: $e"
            end
        end
    end
end

function capture_and_save(t)
    mjv_updateScene(model.internal_pointer, data.internal_pointer, vopt.internal_pointer,
        C_NULL, cam.internal_pointer, Int32(mjCAT_ALL), scn.internal_pointer)

    mjr_setBuffer(Int32(mjFB_OFFSCREEN), con.internal_pointer)
    mjr_render(rect, scn.internal_pointer, con.internal_pointer)
    mjr_readPixels(rgb_buffer, C_NULL, rect, con.internal_pointer)

    img_raw = reshape(rgb_buffer, 3, width, height)
    img_n0f8 = reinterpret(N0f8, img_raw)
    img_final = Matrix{RGB{N0f8}}(undef, height, width)
    for j in 1:height
        for i in 1:width
            img_final[height - j + 1, i] = RGB(
                img_n0f8[1, i, j], img_n0f8[2, i, j], img_n0f8[3, i, j])
        end
    end

    filename = joinpath(@__DIR__, "..", "..", "output",
        "headless_$(Dates.format(now(), "HH-MM-SS-sss")).jpg")
    save(filename, img_final)
end

println("Starting headless simulation loop...")
last_save_time = -0.2
save_interval = 0.2

try
    while true
        # Process control commands
        while isready(control_channel)
            raw_data = take!(control_channel)
            if haskey(raw_data, "command") && raw_data["command"] == "set_joints_state"
                joints = raw_data["joints"]
                for (name, val) in joints
                    if haskey(actuator_map, String(name))
                        data.ctrl[actuator_map[String(name)]] = Float64(val)
                    end
                end
            end
        end

        # Step physics
        step!(model, data)

        # Periodic capture
        if data.time >= last_save_time + save_interval
            capture_and_save(data.time)
            global last_save_time = data.time
            print(".") # Progress indicator
        end

        # Real-time sleep (approximate)
        sleep(0.001)
    end
finally
    GLFW.DestroyWindow(window)
end
