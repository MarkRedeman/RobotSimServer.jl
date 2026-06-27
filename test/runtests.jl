using Test

include(joinpath(@__DIR__, "..", "src", "capture", "Capture.jl"))

const TEST_PORT = 8099
const FIRST_FRAME = 1

mutable struct FakeMJPEGClient <: IO
    data::Vector{UInt8}
    open::Bool
end

FakeMJPEGClient() = FakeMJPEGClient(UInt8[], true)

Base.isopen(client::FakeMJPEGClient) = client.open

function Base.write(client::FakeMJPEGClient, data::Union{SubString{String}, String})
    append!(client.data, codeunits(data))
    return ncodeunits(data)
end

function Base.write(client::FakeMJPEGClient, data::Vector{UInt8})
    append!(client.data, data)
    return length(data)
end

function Base.close(client::FakeMJPEGClient)
    client.open = false
    return nothing
end

takebytes(client::FakeMJPEGClient) = copy(client.data)

@testset "MJPEG backend" begin
    rgb = UInt8[255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255]
    jpeg = encode_jpeg_frame(rgb, 2, 2)
    @test !isempty(jpeg)

    client = FakeMJPEGClient()
    state = MJPEGBackendState(
        TEST_PORT,          # port
        "camera",          # camera_name
        nothing,            # server_task
        Any[client],        # clients
        ReentrantLock(),    # clients_lock
        Condition(),        # shutdown_condition
        true                # running
    )
    work = CaptureWork(
        "camera", FIRST_FRAME, rgb, 2, 2, MJPEGOutput(port = TEST_PORT), state, time())

    process_frame!(MJPEGOutput(port = TEST_PORT), state, work)

    payload = takebytes(client)
    @test !isempty(payload)
    prefix = collect(codeunits(mjpeg_part_header(jpeg)))
    @test payload[1:length(prefix)] == prefix
    @test length(payload) > length(prefix)

    cleanup_backend!(MJPEGOutput(port = TEST_PORT), state)
    @test !client.open
end
