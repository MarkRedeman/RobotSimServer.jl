# AssetServer - HTTP static file serving for robot assets
#
# Provides HTTP endpoints for serving robot URDF files, mesh files, and textures
# to web clients (React apps) for 3D visualization.
#
# Features:
#   - URDF serving with proper Content-Type headers
#   - Mesh file serving (STL, OBJ, DAE) with path traversal protection
#   - Texture serving (PNG, JPEG)
#   - CORS support for cross-origin requests
#
# Usage:
#   include("src/AssetServer.jl")
#
#   # In HTTP handler:
#   function handle_http_request!(http, robot_config)
#       path = http.message.target
#       
#       if endswith(path, "/urdf")
#           serve_urdf!(http, robot_config.urdf_path, robot_config.robot_id)
#       elseif contains(path, "/meshes/")
#           mesh_path = extract_mesh_path(path)
#           serve_mesh!(http, robot_config.assets_dir, mesh_path)
#       end
#   end

using HTTP

# =============================================================================
# Content Type Mappings
# =============================================================================

"""
    CONTENT_TYPES::Dict{String, String}

Mapping of file extensions to MIME content types for robot assets.
"""
const CONTENT_TYPES = Dict{String, String}(
    ".stl" => "model/stl",
    ".obj" => "model/obj",
    ".dae" => "model/vnd.collada+xml",
    ".png" => "image/png",
    ".jpg" => "image/jpeg",
    ".jpeg" => "image/jpeg",
    ".xml" => "application/xml",
    ".urdf" => "application/xml"
)

"""
    get_content_type(filepath::AbstractString) -> String

Determine the Content-Type for a file based on its extension.

Returns "application/octet-stream" for unknown file types.

# Arguments
- `filepath`: Path to the file (only extension is used)

# Returns
- MIME content type string
"""
function get_content_type(filepath::AbstractString)
    ext = lowercase(splitext(filepath)[2])
    return get(CONTENT_TYPES, ext, "application/octet-stream")
end

# =============================================================================
# CORS Support
# =============================================================================

"""
    add_cors_headers!(http)

Add CORS headers to allow cross-origin requests from web clients.

Adds:
- Access-Control-Allow-Origin: *
- Access-Control-Allow-Methods: GET, OPTIONS
- Access-Control-Allow-Headers: Content-Type

# Arguments
- `http`: HTTP.Stream object
"""
function add_cors_headers!(http)
    HTTP.setheader(http, "Access-Control-Allow-Origin" => "*")
    HTTP.setheader(http, "Access-Control-Allow-Methods" => "GET, OPTIONS")
    HTTP.setheader(http, "Access-Control-Allow-Headers" => "Content-Type")
end

"""
    handle_options_preflight!(http) -> Bool

Handle OPTIONS preflight requests for CORS.

Returns true if the request was an OPTIONS request and was handled.

# Arguments
- `http`: HTTP.Stream object

# Returns
- `true` if OPTIONS request was handled, `false` otherwise
"""
function handle_options_preflight!(http)
    if http.message.method == "OPTIONS"
        add_cors_headers!(http)
        HTTP.setstatus(http, 204)  # No Content
        HTTP.startwrite(http)
        return true
    end
    return false
end

# =============================================================================
# Path Security
# =============================================================================

"""
    is_path_safe(base_dir::AbstractString, requested_path::AbstractString) -> Bool

Check if a requested path is safely contained within the base directory.

Uses realpath() to resolve symlinks and normalize paths before checking
containment. This prevents path traversal attacks using ".." or symlinks.

# Arguments
- `base_dir`: The base directory that should contain the file
- `requested_path`: The path to validate

# Returns
- `true` if the path is safely within base_dir, `false` otherwise
"""
function is_path_safe(base_dir::AbstractString, requested_path::AbstractString)
    try
        # Resolve to absolute paths, following symlinks
        resolved_base = realpath(base_dir)
        resolved_path = realpath(requested_path)

        # Check if resolved path starts with resolved base
        # Add trailing separator to prevent matching partial directory names
        # e.g., /foo/bar should not match /foo/barbaz
        base_with_sep = resolved_base * (endswith(resolved_base, '/') ? "" : "/")
        return startswith(resolved_path, base_with_sep) ||
               resolved_path == resolved_base
    catch e
        # If realpath fails (file doesn't exist, permission denied, etc.)
        # treat as unsafe
        return false
    end
end

"""
    normalize_subpath(subpath::AbstractString) -> String

Normalize a subpath by removing leading slashes and collapsing path components.

# Arguments
- `subpath`: The subpath to normalize

# Returns
- Normalized subpath string
"""
function normalize_subpath(subpath::AbstractString)
    # Remove leading slashes
    result = lstrip(subpath, '/')
    # URL decode if necessary (basic %XX decoding)
    result = HTTP.URIs.unescapeuri(result)
    return result
end

# =============================================================================
# Response Helpers
# =============================================================================

"""
    serve_content!(http, content::Union{String, Vector{UInt8}}, content_type::AbstractString)

Serve content with the specified content type and CORS headers.

# Arguments
- `http`: HTTP.Stream object
- `content`: Content to serve (string or bytes)
- `content_type`: MIME content type
"""
function serve_content!(http, content::Union{String, Vector{UInt8}}, content_type::AbstractString)
    add_cors_headers!(http)
    HTTP.setheader(http, "Content-Type" => content_type)
    HTTP.setheader(http, "Content-Length" => string(sizeof(content)))
    HTTP.setstatus(http, 200)
    HTTP.startwrite(http)
    write(http, content)
end

"""
    serve_file!(http, filepath::AbstractString, content_type::AbstractString)

Serve a file from disk with the specified content type.

Reads the file and serves it with appropriate headers. Returns false if
the file cannot be read.

# Arguments
- `http`: HTTP.Stream object
- `filepath`: Path to the file to serve
- `content_type`: MIME content type

# Returns
- `true` if file was served successfully, `false` otherwise
"""
function serve_file!(http, filepath::AbstractString, content_type::AbstractString)
    try
        content = read(filepath)
        serve_content!(http, content, content_type)
        return true
    catch e
        @warn "Failed to read file" filepath exception = e
        return false
    end
end

"""
    serve_404!(http, message::String = "Not Found")

Send a 404 Not Found response with CORS headers.

# Arguments
- `http`: HTTP.Stream object
- `message`: Optional message to include in the response body
"""
function serve_404!(http, message::String = "Not Found")
    add_cors_headers!(http)
    HTTP.setheader(http, "Content-Type" => "text/plain")
    HTTP.setstatus(http, 404)
    HTTP.startwrite(http)
    write(http, message)
end

"""
    serve_403!(http, message::String = "Forbidden")

Send a 403 Forbidden response with CORS headers.

# Arguments
- `http`: HTTP.Stream object
- `message`: Optional message to include in the response body
"""
function serve_403!(http, message::String = "Forbidden")
    add_cors_headers!(http)
    HTTP.setheader(http, "Content-Type" => "text/plain")
    HTTP.setstatus(http, 403)
    HTTP.startwrite(http)
    write(http, message)
end

"""
    serve_500!(http, message::String = "Internal Server Error")

Send a 500 Internal Server Error response with CORS headers.

# Arguments
- `http`: HTTP.Stream object
- `message`: Optional message to include in the response body
"""
function serve_500!(http, message::String = "Internal Server Error")
    add_cors_headers!(http)
    HTTP.setheader(http, "Content-Type" => "text/plain")
    HTTP.setstatus(http, 500)
    HTTP.startwrite(http)
    write(http, message)
end

# =============================================================================
# Asset Serving Functions
# =============================================================================

"""
    serve_urdf!(http, urdf_path::AbstractString, robot_id::AbstractString)

Serve a URDF file for a robot.

# Arguments
- `http`: HTTP.Stream object
- `urdf_path`: Absolute path to the URDF file
- `robot_id`: Robot identifier (used for error messages)

# Returns
- `true` if URDF was served successfully, `false` otherwise

# Example
```julia
serve_urdf!(http, "/path/to/robot.urdf", "so101")
```
"""
function serve_urdf!(http, urdf_path::AbstractString, robot_id::AbstractString)
    # Handle OPTIONS preflight
    if handle_options_preflight!(http)
        return true
    end

    # Check if file exists
    if !isfile(urdf_path)
        serve_404!(http, "URDF not found for robot: $robot_id")
        return false
    end

    # Serve the URDF file
    if !serve_file!(http, urdf_path, "application/xml")
        serve_500!(http, "Failed to read URDF file for robot: $robot_id")
        return false
    end

    return true
end

"""
    serve_mesh!(http, assets_dir::AbstractString, mesh_subpath::AbstractString)

Serve a mesh file from the assets directory with path traversal protection.

Validates that the requested mesh path doesn't escape the assets directory
using realpath() resolution. Supports STL, OBJ, DAE mesh formats and
PNG/JPEG textures.

# Arguments
- `http`: HTTP.Stream object
- `assets_dir`: Base directory containing mesh assets
- `mesh_subpath`: Relative path to the mesh file within assets_dir

# Returns
- `true` if mesh was served successfully, `false` otherwise

# Security
This function validates that the resolved path is within assets_dir to prevent
path traversal attacks. It uses realpath() to resolve symlinks before checking.

# Example
```julia
serve_mesh!(http, "/path/to/assets", "meshes/arm/link1.stl")
```
"""
function serve_mesh!(http, assets_dir::AbstractString, mesh_subpath::AbstractString)
    # Handle OPTIONS preflight
    if handle_options_preflight!(http)
        return true
    end

    # Normalize the subpath
    normalized_subpath = normalize_subpath(mesh_subpath)

    # Build full path
    full_path = joinpath(assets_dir, normalized_subpath)

    # First check if file exists (before realpath, which requires existence)
    if !isfile(full_path)
        serve_404!(http, "Mesh file not found: $normalized_subpath")
        return false
    end

    # Security check: ensure path is within assets directory
    if !is_path_safe(assets_dir, full_path)
        @warn "Path traversal attempt blocked" requested=mesh_subpath resolved=full_path
        serve_403!(http, "Access denied: path outside assets directory")
        return false
    end

    # Determine content type
    content_type = get_content_type(full_path)

    # Serve the file
    if !serve_file!(http, full_path, content_type)
        serve_500!(http, "Failed to read mesh file: $normalized_subpath")
        return false
    end

    return true
end

"""
    serve_texture!(http, assets_dir::AbstractString, texture_subpath::AbstractString)

Serve a texture file from the assets directory with path traversal protection.

This is an alias for serve_mesh! since textures follow the same serving pattern.

# Arguments
- `http`: HTTP.Stream object
- `assets_dir`: Base directory containing texture assets
- `texture_subpath`: Relative path to the texture file within assets_dir

# Returns
- `true` if texture was served successfully, `false` otherwise
"""
function serve_texture!(http, assets_dir::AbstractString, texture_subpath::AbstractString)
    return serve_mesh!(http, assets_dir, texture_subpath)
end

"""
    serve_asset!(http, assets_dir::AbstractString, asset_subpath::AbstractString)

Generic asset serving function that handles any file type.

Determines content type automatically and applies path traversal protection.

# Arguments
- `http`: HTTP.Stream object
- `assets_dir`: Base directory containing assets
- `asset_subpath`: Relative path to the asset within assets_dir

# Returns
- `true` if asset was served successfully, `false` otherwise
"""
function serve_asset!(http, assets_dir::AbstractString, asset_subpath::AbstractString)
    return serve_mesh!(http, assets_dir, asset_subpath)
end
