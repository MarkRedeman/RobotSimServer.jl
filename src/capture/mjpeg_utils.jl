# Shared MJPEG helpers for capture backends and unified streaming

const MJPEG_BOUNDARY = "frame"

function add_mjpeg_headers!(http)
    HTTP.setheader(http, "Content-Type" => "multipart/x-mixed-replace; boundary=$MJPEG_BOUNDARY")
    HTTP.setheader(http, "Cache-Control" => "no-cache, no-store, must-revalidate")
    HTTP.setheader(http, "Pragma" => "no-cache")
    HTTP.setheader(http, "Connection" => "keep-alive")
end

function mjpeg_part_header(jpeg_bytes::Vector{UInt8})
    return "--$MJPEG_BOUNDARY\r\nContent-Type: image/jpeg\r\nContent-Length: $(length(jpeg_bytes))\r\n\r\n"
end
