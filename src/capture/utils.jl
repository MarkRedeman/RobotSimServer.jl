# Shared helpers for capture backends

"""
    encode_jpeg_frame(rgb_data::Vector{UInt8}, width::Int, height::Int) -> Vector{UInt8}

Convert a flipped RGB buffer into JPEG bytes for streaming backends using
`rgb_to_image` from the shared capture helpers.
"""
function encode_jpeg_frame(rgb_data::Vector{UInt8}, width::Int, height::Int)
    img = rgb_to_image(rgb_data, width, height)
    io = IOBuffer()
    save(Stream{format"JPEG"}(io), img)
    return take!(io)
end
