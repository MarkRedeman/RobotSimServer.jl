using Test

include(joinpath(@__DIR__, "..", "src", "UnifiedWebSocketServer.jl"))
include(joinpath(@__DIR__, "..", "src", "capture", "Capture.jl"))

const TEST_PORT = 8099
const TEST_FRAME_NUMBER = 1

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
    state = init_backend(MJPEGOutput(port = TEST_PORT), "camera", 2, 2, 30.0)
    @lock state.clients_lock push!(state.clients, client)
    work = CaptureWork(
        "camera", TEST_FRAME_NUMBER, rgb, 2, 2, MJPEGOutput(port = TEST_PORT), state,
        time())

    process_frame!(MJPEGOutput(port = TEST_PORT), state, work)

    payload = takebytes(client)
    @test !isempty(payload)
    prefix = collect(codeunits(mjpeg_part_header(jpeg)))
    @test payload[1:length(prefix)] == prefix
    @test length(payload) > length(prefix)

    cleanup_backend!(MJPEGOutput(port = TEST_PORT), state)
    @test !client.open
end

@testset "Unified MJPEG backend" begin
    rgb = UInt8[255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255]
    jpeg = encode_jpeg_frame(rgb, 2, 2)
    server = UnifiedServer(port = TEST_PORT, robot = "robot")
    state = init_backend(MJPEGOutput(server = server), "camera", 2, 2, 30.0)

    client = FakeMJPEGClient()
    endpoint = server.mjpeg_cameras["camera"]
    @lock endpoint.clients_lock push!(endpoint.clients, client)

    work = CaptureWork(
        "camera", TEST_FRAME_NUMBER, rgb, 2, 2, MJPEGOutput(server = server), state,
        time())

    process_frame!(MJPEGOutput(server = server), state, work)

    payload = takebytes(client)
    @test !isempty(payload)
    prefix = collect(codeunits(mjpeg_part_header(jpeg)))
    @test payload[1:length(prefix)] == prefix
    @test length(payload) > length(prefix)

    cleanup_backend!(MJPEGOutput(server = server), state)
    stop!(server)
end
