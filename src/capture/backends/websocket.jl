# WebSocketOutput backend - stream frames to WebSocket clients

using HTTP
using HTTP.WebSockets

# =============================================================================
# Unified Server Mode (Recommended)
# =============================================================================

"""
    UnifiedWebSocketBackendState

State for WebSocket output using unified server.
"""
mutable struct UnifiedWebSocketBackendState
    server::Any  # UnifiedServer
    camera_name::String
end

"""
    init_backend(backend::WebSocketOutput, camera_name::String, width::Int, height::Int, fps::Float64)

Initialize WebSocket output backend.
"""
function init_backend(backend::WebSocketOutput, camera_name::String,
        width::Int, height::Int, fps::Float64)

    # New unified server mode
    if backend.server !== nothing
        # Camera should already be registered via register_camera!
        # but we can check and warn if not
        if !haskey(backend.server.cameras, camera_name)
            @warn "Camera '$camera_name' not registered with server. Call register_camera!(server, \"$camera_name\") before starting."
        end
        println("WebSocketOutput($(camera_name)): using unified server")
        return UnifiedWebSocketBackendState(backend.server, camera_name)
    end

    # =========================================================================
    # LEGACY: Port-based mode - remove this block when all simulations migrated
    # =========================================================================
    return init_legacy_backend(backend, camera_name, width, height, fps)
    # =========================================================================
end

"""
    process_frame!(backend::WebSocketOutput, state::UnifiedWebSocketBackendState, work::CaptureWork)

Encode frame and broadcast via unified server.
"""
function process_frame!(
        backend::WebSocketOutput, state::UnifiedWebSocketBackendState, work::CaptureWork)

    # Skip encoding if no clients connected
    if get_camera_client_count(state.server, state.camera_name) == 0
        return
    end

    # Convert to image and encode as JPEG
    img = rgb_to_image(work.rgb_data, work.width, work.height)
    io = IOBuffer()
    save(Stream{format"JPEG"}(io), img)
    jpeg_bytes = take!(io)

    # Broadcast via unified server
    broadcast_frame!(state.server, state.camera_name, jpeg_bytes)
end

"""
    cleanup_backend!(backend::WebSocketOutput, state::UnifiedWebSocketBackendState)

Cleanup unified WebSocket backend (no-op, server handles cleanup).
"""
function cleanup_backend!(backend::WebSocketOutput, state::UnifiedWebSocketBackendState)
    println("WebSocketOutput($(state.camera_name)): detached from unified server")
    # Server cleanup is handled by stop!(server)
end

# =============================================================================
# LEGACY: Port-based mode - remove everything below when all simulations migrated
# =============================================================================

"""
    WebSocketBackendState

State for WebSocket output backend (legacy port-based mode).
Manages server and connected clients.
"""
mutable struct WebSocketBackendState
    port::Int
    camera_name::String
    server_task::Union{Task, Nothing}
    clients::Vector{HTTP.WebSockets.WebSocket}
    clients_lock::ReentrantLock
    running::Bool
end

"""
    init_legacy_backend(backend::WebSocketOutput, camera_name::String, width::Int, height::Int, fps::Float64)

Initialize legacy port-based WebSocket backend. Starts a WS server on the specified port.
"""
function init_legacy_backend(backend::WebSocketOutput, camera_name::String,
        width::Int, height::Int, fps::Float64)
    state = WebSocketBackendState(
        backend.port,
        camera_name,
        nothing,
        HTTP.WebSockets.WebSocket[],
        ReentrantLock(),
        true
    )

    # Start WebSocket server in background
    state.server_task = @async begin
        try
            HTTP.WebSockets.listen("0.0.0.0", backend.port) do ws
                # Add client to list
                @lock state.clients_lock begin
                    push!(state.clients, ws)
                end
                println("WebSocketOutput($(camera_name)): client connected ($(length(state.clients)) total)")

                # Keep connection alive, handle incoming messages (ignored)
                try
                    for msg in ws
                        # We don't expect messages from clients, but consume them
                    end
                catch e
                    if !(e isa HTTP.WebSockets.WebSocketError || e isa EOFError)
                        @warn "WebSocket client error" exception=e
                    end
                finally
                    # Remove client from list
                    @lock state.clients_lock begin
                        filter!(c -> c !== ws, state.clients)
                    end
                    println("WebSocketOutput($(camera_name)): client disconnected ($(length(state.clients)) remaining)")
                end
            end
        catch e
            if state.running
                @warn "WebSocket server error" exception=(e, catch_backtrace())
            end
        end
    end

    println("WebSocketOutput: streaming $(camera_name) on ws://0.0.0.0:$(backend.port)")

    return state
end

"""
    process_frame!(backend::WebSocketOutput, state::WebSocketBackendState, work::CaptureWork)

Encode frame as JPEG and broadcast to all connected clients (legacy mode).
"""
function process_frame!(
        backend::WebSocketOutput, state::WebSocketBackendState, work::CaptureWork)
    # Get current clients snapshot
    clients = @lock state.clients_lock copy(state.clients)

    if isempty(clients)
        return  # No clients connected, skip encoding
    end

    # Convert to image and encode as JPEG
    img = rgb_to_image(work.rgb_data, work.width, work.height)

    # Encode to JPEG bytes in memory
    io = IOBuffer()
    save(Stream{format"JPEG"}(io), img)
    jpeg_bytes = take!(io)

    # Broadcast to all clients
    failed_clients = HTTP.WebSockets.WebSocket[]

    for client in clients
        try
            send(client, jpeg_bytes)
        catch e
            push!(failed_clients, client)
        end
    end

    # Remove failed clients
    if !isempty(failed_clients)
        @lock state.clients_lock begin
            filter!(c -> !(c in failed_clients), state.clients)
        end
    end
end

"""
    cleanup_backend!(backend::WebSocketOutput, state::WebSocketBackendState)

Cleanup WebSocket backend (legacy mode). Closes all connections and stops server.
"""
function cleanup_backend!(backend::WebSocketOutput, state::WebSocketBackendState)
    state.running = false

    # Close all client connections
    clients = @lock state.clients_lock copy(state.clients)
    for client in clients
        try
            close(client)
        catch e
            # Ignore close errors
        end
    end

    # Note: HTTP.WebSockets.listen doesn't have a clean shutdown mechanism
    # The server task will terminate when all clients disconnect or on next error

    println("WebSocketOutput($(state.camera_name)): shutdown")
end

# =============================================================================
# END LEGACY CODE
# =============================================================================
