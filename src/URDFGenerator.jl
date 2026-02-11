# URDFGenerator - Convert MuJoCo XML (MJCF) to URDF format
#
# This module provides MJCF to URDF conversion for serving robot descriptions
# to web clients that require URDF format (e.g., for 3D visualization).
#
# MuJoCo has many features that URDF doesn't support (tendons, actuators,
# contact parameters, etc.). This converter focuses on the kinematic structure:
# links, joints, and visual/collision geometry.
#
# Usage:
#   include("src/URDFGenerator.jl")
#   
#   # Direct conversion
#   urdf_content = generate_urdf_from_mjcf("robots/trossen/scene.xml")
#   
#   # With caching
#   urdf_content = get_or_generate_urdf("robots/trossen/scene.xml", ".cache/urdf")

using EzXML

"""
    MJCFParseError

Exception type for MJCF parsing errors.
"""
struct MJCFParseError <: Exception
    msg::String
end

Base.showerror(io::IO, e::MJCFParseError) = print(io, "MJCFParseError: ", e.msg)

# =============================================================================
# MJCF Joint Type Mapping
# =============================================================================

const MJCF_TO_URDF_JOINT_TYPE = Dict{String, String}(
    "hinge" => "revolute",
    "slide" => "prismatic",
    "free" => "floating",
    "ball" => "floating"  # URDF doesn't have ball joints, approximate with floating
)

"""
    mjcf_joint_type_to_urdf(mjcf_type::String) -> String

Convert MuJoCo joint type to URDF joint type.

# Mapping
- `hinge` → `revolute`
- `slide` → `prismatic`
- `free` → `floating`
- `ball` → `floating` (approximation)
- default → `revolute` (MuJoCo default is hinge)
"""
function mjcf_joint_type_to_urdf(mjcf_type::String)
    return get(MJCF_TO_URDF_JOINT_TYPE, lowercase(mjcf_type), "revolute")
end

# =============================================================================
# XML Utilities
# =============================================================================

"""
    get_attr(node::EzXML.Node, name::String, default::String="") -> String

Safely get an XML attribute value, returning default if not present.
"""
function get_attr(node::EzXML.Node, name::String, default::String = "")
    return haskey(node, name) ? node[name] : default
end

"""
    parse_vec3(s::String, default::Vector{Float64}=[0.0, 0.0, 0.0]) -> Vector{Float64}

Parse a space-separated string of 3 floats into a vector.
"""
function parse_vec3(s::String, default::Vector{Float64} = [0.0, 0.0, 0.0])
    if isempty(s)
        return default
    end
    parts = split(strip(s))
    if length(parts) >= 3
        try
            return [parse(Float64, parts[1]), parse(Float64, parts[2]),
                parse(Float64, parts[3])]
        catch
            return default
        end
    end
    return default
end

"""
    parse_quat(s::String) -> Vector{Float64}

Parse a quaternion string. MuJoCo uses [w, x, y, z] format.
Returns [w, x, y, z] or identity quaternion if parsing fails.
"""
function parse_quat(s::String)
    default = [1.0, 0.0, 0.0, 0.0]
    if isempty(s)
        return default
    end
    parts = split(strip(s))
    if length(parts) >= 4
        try
            return [parse(Float64, parts[1]), parse(Float64, parts[2]),
                parse(Float64, parts[3]), parse(Float64, parts[4])]
        catch
            return default
        end
    end
    return default
end

"""
    quat_to_rpy(quat::Vector{Float64}) -> Vector{Float64}

Convert quaternion [w, x, y, z] to roll-pitch-yaw [r, p, y] in radians.
"""
function quat_to_rpy(quat::Vector{Float64})
    w, x, y, z = quat
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = atan(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0
        pitch = copysign(π / 2, sinp)  # Use 90 degrees if out of range
    else
        pitch = asin(sinp)
    end

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan(siny_cosp, cosy_cosp)

    return [roll, pitch, yaw]
end

# Rotation matrix constructors for euler angle conversion
function _rotx(a::Float64)
    c, s = cos(a), sin(a)
    return [1.0 0.0 0.0; 0.0 c -s; 0.0 s c]
end

function _roty(a::Float64)
    c, s = cos(a), sin(a)
    return [c 0.0 s; 0.0 1.0 0.0; -s 0.0 c]
end

function _rotz(a::Float64)
    c, s = cos(a), sin(a)
    return [c -s 0.0; s c 0.0; 0.0 0.0 1.0]
end

"""
    intrinsic_xyz_to_matrix(a::Float64, b::Float64, c::Float64) -> Matrix{Float64}

Build rotation matrix from intrinsic XYZ euler angles.
Intrinsic XYZ means: rotate around X, then around the NEW Y, then around the NEW Z.
"""
function intrinsic_xyz_to_matrix(a::Float64, b::Float64, c::Float64)
    return _rotx(a) * _roty(b) * _rotz(c)
end

"""
    intrinsic_zyx_to_matrix(a::Float64, b::Float64, c::Float64) -> Matrix{Float64}

Build rotation matrix from intrinsic ZYX euler angles.
"""
function intrinsic_zyx_to_matrix(a::Float64, b::Float64, c::Float64)
    return _rotz(a) * _roty(b) * _rotx(c)
end

"""
    matrix_to_extrinsic_xyz(R::Matrix{Float64}) -> Vector{Float64}

Extract extrinsic XYZ euler angles (URDF rpy) from a rotation matrix.
URDF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
"""
function matrix_to_extrinsic_xyz(R::Matrix{Float64})
    # Extract roll, pitch, yaw from R = Rz(yaw) * Ry(pitch) * Rx(roll)
    sy = -R[3, 1]
    if abs(sy) >= 1.0
        # Gimbal lock
        pitch = copysign(π / 2, sy)
        roll = 0.0
        yaw = atan(R[1, 2], R[1, 3])
    else
        pitch = asin(sy)
        roll = atan(R[3, 2], R[3, 3])
        yaw = atan(R[2, 1], R[1, 1])
    end
    return [roll, pitch, yaw]
end

"""
    convert_mjcf_euler_to_urdf_rpy(euler::Vector{Float64}, eulerseq::String) -> Vector{Float64}

Convert MJCF euler angles to URDF roll-pitch-yaw (extrinsic XYZ).

MJCF default eulerseq is "xyz" (lowercase = intrinsic/body-fixed rotations).
URDF rpy is extrinsic XYZ (fixed-axis rotations).

The conversion builds a rotation matrix from the MJCF euler angles, then extracts
the equivalent extrinsic XYZ angles for URDF.
"""
function convert_mjcf_euler_to_urdf_rpy(euler::Vector{Float64}, eulerseq::String)
    seq = lowercase(eulerseq)

    if seq == "xyz"
        # Intrinsic xyz -> build rotation matrix -> extract extrinsic XYZ
        R = intrinsic_xyz_to_matrix(euler[1], euler[2], euler[3])
        return matrix_to_extrinsic_xyz(R)
    elseif seq == "zyx"
        # Intrinsic zyx -> build rotation matrix -> extract extrinsic XYZ
        R = intrinsic_zyx_to_matrix(euler[1], euler[2], euler[3])
        return matrix_to_extrinsic_xyz(R)
    else
        # For uppercase (extrinsic) sequences, handle common cases
        if eulerseq == "XYZ"
            # Already extrinsic XYZ, same as URDF rpy
            return euler
        elseif eulerseq == "ZYX"
            # Extrinsic ZYX -> build matrix -> extract extrinsic XYZ
            R = _rotx(euler[3]) * _roty(euler[2]) * _rotz(euler[1])
            return matrix_to_extrinsic_xyz(R)
        else
            @warn "Unsupported eulerseq '$eulerseq', using euler values directly"
            return euler
        end
    end
end

"""
    format_vec3(v::Vector{Float64}) -> String

Format a 3-element vector as a space-separated string.
"""
function format_vec3(v::Vector{Float64})
    return "$(v[1]) $(v[2]) $(v[3])"
end

# =============================================================================
# MJCF Include Resolution
# =============================================================================

"""
    resolve_mjcf_includes(doc::EzXML.Document, base_dir::String) -> EzXML.Document

Recursively resolve <include file="..."/> directives in MJCF.
Returns a new document with all includes inlined.
"""
function resolve_mjcf_includes(doc::EzXML.Document, base_dir::String)
    root = EzXML.root(doc)
    _resolve_includes_recursive!(root, base_dir)
    return doc
end

function _resolve_includes_recursive!(node::EzXML.Node, base_dir::String)
    # Find all include elements
    includes_to_process = EzXML.Node[]
    for child in EzXML.eachelement(node)
        if EzXML.nodename(child) == "include"
            push!(includes_to_process, child)
        end
    end

    # Process includes
    for include_node in includes_to_process
        file_attr = get_attr(include_node, "file", "")
        if isempty(file_attr)
            @warn "Include element missing 'file' attribute"
            continue
        end

        include_path = joinpath(base_dir, file_attr)
        if !isfile(include_path)
            @warn "Include file not found: $include_path"
            continue
        end

        # Parse included file
        included_doc = EzXML.readxml(include_path)
        included_root = EzXML.root(included_doc)
        included_base_dir = dirname(include_path)

        # Recursively resolve includes in the included document
        _resolve_includes_recursive!(included_root, included_base_dir)

        # Insert all children of the included mujoco root before the include element
        parent = EzXML.parentnode(include_node)
        for child in EzXML.eachelement(included_root)
            # Clone with path rewriting relative to included file's directory
            cloned = _deep_clone_with_base_dir(child, included_base_dir)
            EzXML.linkprev!(include_node, cloned)
        end

        # Remove the include element
        EzXML.unlink!(include_node)
    end

    # Recursively process children
    for child in EzXML.eachelement(node)
        _resolve_includes_recursive!(child, base_dir)
    end
end

"""
    _deep_clone_with_base_dir(node::EzXML.Node, base_dir::String) -> EzXML.Node

Create a deep clone of an XML node, converting relative file paths to absolute paths.
This is used when inlining included files to preserve correct path resolution.
"""
function _deep_clone_with_base_dir(node::EzXML.Node, base_dir::String)
    # Create new element with same name
    new_node = EzXML.ElementNode(EzXML.nodename(node))

    # Copy attributes, making file paths absolute
    for attr in EzXML.attributes(node)
        key = EzXML.nodename(attr)
        value = EzXML.nodecontent(attr)

        # Convert relative file paths to absolute
        if key == "file" && !isempty(value) && !isabspath(value)
            value = normpath(joinpath(base_dir, value))
        end

        new_node[key] = value
    end

    # Recursively clone children
    for child in EzXML.eachelement(node)
        cloned_child = _deep_clone_with_base_dir(child, base_dir)
        EzXML.link!(new_node, cloned_child)
    end

    return new_node
end

# =============================================================================
# Model Asset Resolution (for <attach> directive support)
# =============================================================================

"""
    ModelAsset

Information about a model asset defined in MJCF <asset> section.
"""
struct ModelAsset
    name::String
    file::String           # Original file path
    resolved_path::String  # Absolute path to the model file
    base_dir::String       # Directory containing the model file
end

"""
    extract_model_assets(doc::EzXML.Document, base_dir::String) -> Dict{String, ModelAsset}

Extract model asset definitions from MJCF document.
Returns a dictionary mapping model names to their file paths.
"""
function extract_model_assets(doc::EzXML.Document, base_dir::String)
    models = Dict{String, ModelAsset}()
    root = EzXML.root(doc)

    # Find all asset sections
    for asset_node in EzXML.findall("//asset", root)
        for child in EzXML.eachelement(asset_node)
            if EzXML.nodename(child) == "model"
                name = get_attr(child, "name", "")
                file = get_attr(child, "file", "")

                if !isempty(name) && !isempty(file)
                    # File path may already be absolute if it came from an included file
                    if isabspath(file)
                        resolved_path = normpath(file)
                    else
                        resolved_path = normpath(joinpath(base_dir, file))
                    end
                    model_base_dir = dirname(resolved_path)
                    models[name] = ModelAsset(name, file, resolved_path, model_base_dir)
                end
            end
        end
    end

    return models
end

"""
    resolve_attach_directives!(doc::EzXML.Document, base_dir::String)

Resolve <attach model="..." body="..." prefix="..."/> directives in MJCF.
Replaces attach elements with the actual body hierarchy from the referenced model.
Also merges <default> sections from attached models to ensure proper property inheritance.
"""
function resolve_attach_directives!(doc::EzXML.Document, base_dir::String)
    root = EzXML.root(doc)

    # First, extract model assets
    models = extract_model_assets(doc, base_dir)

    if isempty(models)
        return  # No models to attach
    end

    # Find and process all attach elements, passing doc for defaults merging
    _resolve_attach_recursive!(doc, root, models)
end

function _resolve_attach_recursive!(
        doc::EzXML.Document, node::EzXML.Node, models::Dict{String, ModelAsset})
    # Find all attach elements in this node
    attach_nodes = EzXML.Node[]
    for child in EzXML.eachelement(node)
        if EzXML.nodename(child) == "attach"
            push!(attach_nodes, child)
        end
    end

    # Process each attach element
    for attach_node in attach_nodes
        model_name = get_attr(attach_node, "model", "")
        body_name = get_attr(attach_node, "body", "")
        prefix = get_attr(attach_node, "prefix", "")

        if isempty(model_name)
            @warn "Attach element missing 'model' attribute"
            EzXML.unlink!(attach_node)
            continue
        end

        if !haskey(models, model_name)
            @warn "Model not found for attach: $model_name"
            EzXML.unlink!(attach_node)
            continue
        end

        model_asset = models[model_name]

        if !isfile(model_asset.resolved_path)
            @warn "Model file not found: $(model_asset.resolved_path)"
            EzXML.unlink!(attach_node)
            continue
        end

        # Parse the model file
        model_doc = EzXML.readxml(model_asset.resolved_path)
        model_root = EzXML.root(model_doc)

        # Recursively resolve includes and attach in the model
        _resolve_includes_recursive!(model_root, model_asset.base_dir)

        # Also resolve nested models in the attached model
        nested_models = extract_model_assets(model_doc, model_asset.base_dir)
        if !isempty(nested_models)
            _resolve_attach_recursive!(model_doc, model_root, nested_models)
        end

        # Merge defaults from the attached model into the main document
        # This ensures joint/geom class defaults are available for URDF generation
        _merge_defaults_from_model!(doc, model_root, model_asset.base_dir)

        # Find the body to attach (or use worldbody children if body is empty)
        bodies_to_insert = EzXML.Node[]

        if isempty(body_name)
            # Attach all worldbody children
            for wb in EzXML.findall("//worldbody", model_root)
                for child in EzXML.eachelement(wb)
                    if EzXML.nodename(child) == "body"
                        push!(bodies_to_insert, child)
                    end
                end
            end
        else
            # Find the specific body by name
            for wb in EzXML.findall("//worldbody", model_root)
                body = _find_body_by_name(wb, body_name)
                if body !== nothing
                    push!(bodies_to_insert, body)
                    break
                end
            end
        end

        if isempty(bodies_to_insert)
            @warn "Body '$body_name' not found in model '$model_name'"
            EzXML.unlink!(attach_node)
            continue
        end

        # Get parent of attach node (should be a body)
        parent_node = EzXML.parentnode(attach_node)

        # Clone and insert bodies, applying prefix to names if specified
        for body in bodies_to_insert
            cloned = _deep_clone_with_prefix(body, prefix, model_asset.base_dir)
            EzXML.linkprev!(attach_node, cloned)
        end

        # Remove the attach element
        EzXML.unlink!(attach_node)
    end

    # Recursively process children (for nested bodies)
    for child in EzXML.eachelement(node)
        _resolve_attach_recursive!(doc, child, models)
    end
end

"""
    _merge_defaults_from_model!(main_doc::EzXML.Document, model_root::EzXML.Node, model_base_dir::String)

Merge <default> sections from an attached model into the main document.
This ensures that joint and geom defaults from the attached model are available
for property resolution during URDF generation.
"""
function _merge_defaults_from_model!(
        main_doc::EzXML.Document, model_root::EzXML.Node, model_base_dir::String)
    main_root = EzXML.root(main_doc)

    # Find default sections in the attached model
    for model_default in EzXML.findall("default", model_root)
        # Clone the default section
        cloned_default = _deep_clone(model_default)

        # Find or create the default section in the main document
        main_defaults = EzXML.findall("default", main_root)

        if isempty(main_defaults)
            # No default section in main doc, create one and add the cloned defaults as a child
            main_default = EzXML.ElementNode("default")
            EzXML.link!(cloned_default, main_default)

            # Insert after compiler or at beginning of document
            first_child = nothing
            for child in EzXML.eachelement(main_root)
                first_child = child
                break
            end

            if first_child !== nothing
                EzXML.linkprev!(first_child, cloned_default)
            else
                EzXML.link!(main_root, cloned_default)
            end
        else
            # Add as a nested default inside the first default section
            # This preserves the class hierarchy
            EzXML.link!(main_defaults[1], cloned_default)
        end
    end
end

"""
    _find_body_by_name(node::EzXML.Node, name::String) -> Union{EzXML.Node, Nothing}

Find a body element by name in the node tree.
"""
function _find_body_by_name(node::EzXML.Node, name::String)
    for child in EzXML.eachelement(node)
        if EzXML.nodename(child) == "body"
            if get_attr(child, "name", "") == name
                return child
            end
            # Search recursively
            result = _find_body_by_name(child, name)
            if result !== nothing
                return result
            end
        end
    end
    return nothing
end

"""
    _deep_clone_with_prefix(node::EzXML.Node, prefix::String, model_base_dir::String) -> EzXML.Node

Create a deep clone of an XML node, applying prefix to name attributes
and adjusting mesh file paths relative to the model's base directory.
"""
function _deep_clone_with_prefix(node::EzXML.Node, prefix::String, model_base_dir::String)
    # Create new element with same name
    new_node = EzXML.ElementNode(EzXML.nodename(node))

    # Copy attributes, applying prefix to 'name' attribute
    for attr in EzXML.attributes(node)
        key = EzXML.nodename(attr)
        value = EzXML.nodecontent(attr)

        if key == "name" && !isempty(prefix)
            value = prefix * value
        end

        new_node[key] = value
    end

    # Recursively clone children
    for child in EzXML.eachelement(node)
        cloned_child = _deep_clone_with_prefix(child, prefix, model_base_dir)
        EzXML.link!(new_node, cloned_child)
    end

    return new_node
end

"""
    _deep_clone(node::EzXML.Node) -> EzXML.Node

Create a deep clone of an XML node.
"""
function _deep_clone(node::EzXML.Node)
    # Create new element with same name
    new_node = EzXML.ElementNode(EzXML.nodename(node))

    # Copy attributes (EzXML.attributes returns Vector{Node})
    for attr in EzXML.attributes(node)
        key = EzXML.nodename(attr)
        value = EzXML.nodecontent(attr)
        new_node[key] = value
    end

    # Recursively clone children
    for child in EzXML.eachelement(node)
        cloned_child = _deep_clone(child)
        EzXML.link!(new_node, cloned_child)
    end

    return new_node
end

# =============================================================================
# Mesh Path Handling
# =============================================================================

"""
    MeshInfo

Information about a mesh referenced in MJCF.
"""
struct MeshInfo
    name::String
    file::String           # Original file path from MJCF
    resolved_file::String  # Resolved path relative to project root (for URL generation)
    scale::Vector{Float64}
end

"""
    resolve_mesh_path(mesh_file::String, base_dir::String, project_root::String) -> String

Resolve a mesh file path to be relative to the project root.

Given a mesh path from MJCF (which may be relative to the MJCF file's directory),
resolve it to an absolute path and then make it relative to the project root.

# Arguments
- `mesh_file`: The mesh file path as specified in MJCF
- `base_dir`: The directory containing the MJCF file
- `project_root`: The project root directory

# Returns
- Path relative to project root, suitable for URL generation
"""
function resolve_mesh_path(mesh_file::String, base_dir::String, project_root::String)
    # Resolve the mesh path relative to the MJCF's base directory
    abs_mesh_path = normpath(joinpath(base_dir, mesh_file))

    # Make it relative to project root
    abs_project_root = normpath(abspath(project_root))

    # Check if the mesh path is within the project root
    if startswith(abs_mesh_path, abs_project_root)
        # Remove project root prefix and leading separator
        rel_path = abs_mesh_path[(length(abs_project_root) + 1):end]
        rel_path = lstrip(rel_path, ['/', '\\'])
        return rel_path
    else
        # Mesh is outside project root - use original path
        # This shouldn't happen in practice, but handle gracefully
        @warn "Mesh file outside project root" mesh_file abs_mesh_path project_root
        return mesh_file
    end
end

"""
    extract_mesh_assets(doc::EzXML.Document, base_dir::String, project_root::String) -> Dict{String, MeshInfo}

Extract mesh asset definitions from MJCF document.
Returns a dictionary mapping mesh names to their file paths and scale.

# Arguments
- `doc`: Parsed MJCF document
- `base_dir`: Directory containing the MJCF file
- `project_root`: Project root directory for resolving relative paths
"""
function extract_mesh_assets(
        doc::EzXML.Document, base_dir::String, project_root::String = base_dir)
    meshes = Dict{String, MeshInfo}()
    root = EzXML.root(doc)

    # Find all asset sections
    for asset_node in EzXML.findall("//asset", root)
        for mesh_node in EzXML.eachelement(asset_node)
            if EzXML.nodename(mesh_node) == "mesh"
                name = get_attr(mesh_node, "name", "")
                file = get_attr(mesh_node, "file", "")
                scale_str = get_attr(mesh_node, "scale", "1 1 1")

                if !isempty(name) && !isempty(file)
                    scale = parse_vec3(scale_str, [1.0, 1.0, 1.0])
                    # Resolve the path relative to project root
                    resolved = resolve_mesh_path(file, base_dir, project_root)
                    meshes[name] = MeshInfo(name, file, resolved, scale)
                end
            end
        end
    end

    return meshes
end

# =============================================================================
# Material Extraction
# =============================================================================

"""
    MaterialInfo

Information about a material defined in MJCF.
Only color-based materials are supported; texture-based materials are skipped.
"""
struct MaterialInfo
    name::String
    rgba::String  # "R G B A" color values
end

"""
    extract_materials(doc::EzXML.Document) -> Dict{String, MaterialInfo}

Extract material definitions from MJCF document.
Returns a dictionary mapping material names to their RGBA colors.

Only extracts color-based materials (those with rgba attribute).
Texture-based materials are skipped as URDF texture support varies by viewer.
"""
function extract_materials(doc::EzXML.Document)
    materials = Dict{String, MaterialInfo}()
    root = EzXML.root(doc)

    # Find all asset sections
    for asset_node in EzXML.findall("//asset", root)
        for mat_node in EzXML.eachelement(asset_node)
            if EzXML.nodename(mat_node) == "material"
                name = get_attr(mat_node, "name", "")
                rgba = get_attr(mat_node, "rgba", "")
                texture = get_attr(mat_node, "texture", "")

                # Only include color-based materials (skip texture-based)
                if !isempty(name) && !isempty(rgba) && isempty(texture)
                    materials[name] = MaterialInfo(name, rgba)
                end
            end
        end
    end

    return materials
end

# =============================================================================
# Default Class Handling
# =============================================================================

"""
    GeomDefaults

Default properties for a geom class, extracted from MJCF <default> elements.
"""
struct GeomDefaults
    type::String
    mesh::String
    size::String
    pos::String
    euler::String
    quat::String
    rgba::String
    material::String  # Material name reference
    contype::String
    conaffinity::String
end

function GeomDefaults()
    return GeomDefaults("sphere", "", "", "0 0 0", "", "1 0 0 0", "", "", "1", "1")
end

"""
    JointDefaults

Default properties for a joint class, extracted from MJCF <default> elements.
"""
struct JointDefaults
    type::String   # Joint type: hinge, slide, ball, free
    axis::String   # Joint axis as "x y z" string
    range::String  # Joint limits as "lower upper" string
    armature::String
    frictionloss::String
    damping::String
end

function JointDefaults()
    return JointDefaults("hinge", "0 0 1", "", "", "", "")
end

"""
    extract_default_classes(doc::EzXML.Document) -> Tuple{Dict{String, GeomDefaults}, Dict{String, JointDefaults}}

Extract default class definitions from MJCF document.
Returns a tuple of two dictionaries:
- Geom defaults: mapping class names to their default geom properties
- Joint defaults: mapping class names to their default joint properties
"""
function extract_default_classes(doc::EzXML.Document)
    geom_defaults = Dict{String, GeomDefaults}()
    joint_defaults = Dict{String, JointDefaults}()
    root = EzXML.root(doc)

    # Find all default sections and process recursively
    for default_section in EzXML.findall("//default", root)
        _extract_defaults_recursive!(
            geom_defaults, joint_defaults, default_section, GeomDefaults(), JointDefaults())
    end

    return (geom_defaults, joint_defaults)
end

function _extract_defaults_recursive!(
        geom_defaults::Dict{String, GeomDefaults},
        joint_defaults::Dict{String, JointDefaults},
        node::EzXML.Node,
        parent_geom_defaults::GeomDefaults,
        parent_joint_defaults::JointDefaults
)
    # Check if this default element has a class attribute
    class_name = get_attr(node, "class", "")

    # Look for geom and joint children to get defaults
    current_geom_defaults = parent_geom_defaults
    current_joint_defaults = parent_joint_defaults

    for child in EzXML.eachelement(node)
        if EzXML.nodename(child) == "geom"
            # Merge geom attributes with parent defaults
            current_geom_defaults = GeomDefaults(
                _get_or_default(child, "type", parent_geom_defaults.type),
                _get_or_default(child, "mesh", parent_geom_defaults.mesh),
                _get_or_default(child, "size", parent_geom_defaults.size),
                _get_or_default(child, "pos", parent_geom_defaults.pos),
                _get_or_default(child, "euler", parent_geom_defaults.euler),
                _get_or_default(child, "quat", parent_geom_defaults.quat),
                _get_or_default(child, "rgba", parent_geom_defaults.rgba),
                _get_or_default(child, "material", parent_geom_defaults.material),
                _get_or_default(child, "contype", parent_geom_defaults.contype),
                _get_or_default(child, "conaffinity", parent_geom_defaults.conaffinity)
            )
        elseif EzXML.nodename(child) == "joint"
            # Merge joint attributes with parent defaults
            current_joint_defaults = JointDefaults(
                _get_or_default(child, "type", parent_joint_defaults.type),
                _get_or_default(child, "axis", parent_joint_defaults.axis),
                _get_or_default(child, "range", parent_joint_defaults.range),
                _get_or_default(child, "armature", parent_joint_defaults.armature),
                _get_or_default(child, "frictionloss", parent_joint_defaults.frictionloss),
                _get_or_default(child, "damping", parent_joint_defaults.damping)
            )
        end
    end

    # Store defaults if this has a class name
    if !isempty(class_name)
        geom_defaults[class_name] = current_geom_defaults
        joint_defaults[class_name] = current_joint_defaults
    end

    # Process nested default elements
    for child in EzXML.eachelement(node)
        if EzXML.nodename(child) == "default"
            _extract_defaults_recursive!(
                geom_defaults, joint_defaults, child,
                current_geom_defaults, current_joint_defaults)
        end
    end
end

function _get_or_default(node::EzXML.Node, attr::String, default::String)
    val = get_attr(node, attr, "")
    return isempty(val) ? default : val
end

"""
    apply_geom_defaults(geom::EzXML.Node, defaults::Dict{String, GeomDefaults}) -> NamedTuple

Get effective geom properties by merging with class defaults.
Returns a named tuple with all relevant properties.
"""
function apply_geom_defaults(geom::EzXML.Node, defaults::Dict{String, GeomDefaults})
    class_name = get_attr(geom, "class", "")

    # Start with base defaults
    base = GeomDefaults()
    if haskey(defaults, class_name)
        base = defaults[class_name]
    end

    # Override with explicit attributes on the geom
    return (
        type = _get_or_default(geom, "type", base.type),
        mesh = _get_or_default(geom, "mesh", base.mesh),
        size = _get_or_default(geom, "size", base.size),
        pos = _get_or_default(geom, "pos", base.pos),
        euler = _get_or_default(geom, "euler", base.euler),
        quat = _get_or_default(geom, "quat", base.quat),
        rgba = _get_or_default(geom, "rgba", base.rgba),
        material = _get_or_default(geom, "material", base.material),
        contype = _get_or_default(geom, "contype", base.contype),
        conaffinity = _get_or_default(geom, "conaffinity", base.conaffinity)
    )
end

"""
    apply_joint_defaults(joint::EzXML.Node, defaults::Dict{String, JointDefaults}) -> NamedTuple

Get effective joint properties by merging with class defaults.
Returns a named tuple with all relevant properties.
"""
function apply_joint_defaults(joint::EzXML.Node, defaults::Dict{String, JointDefaults})
    class_name = get_attr(joint, "class", "")

    # Start with base defaults
    base = JointDefaults()
    if haskey(defaults, class_name)
        base = defaults[class_name]
    end

    # Override with explicit attributes on the joint
    return (
        type = _get_or_default(joint, "type", base.type),
        axis = _get_or_default(joint, "axis", base.axis),
        range = _get_or_default(joint, "range", base.range),
        armature = _get_or_default(joint, "armature", base.armature),
        frictionloss = _get_or_default(joint, "frictionloss", base.frictionloss),
        damping = _get_or_default(joint, "damping", base.damping)
    )
end

# =============================================================================
# URDF Generation
# =============================================================================

"""
    URDFBuilder

Accumulator for building URDF XML content.
"""
mutable struct URDFBuilder
    io::IOBuffer
    indent_level::Int
    robot_name::String
    link_names::Set{String}
    joint_names::Set{String}
    meshes::Dict{String, MeshInfo}
    materials::Dict{String, MaterialInfo}
    geom_defaults::Dict{String, GeomDefaults}
    joint_defaults::Dict{String, JointDefaults}
    base_dir::String
    warnings::Vector{String}
    eulerseq::String  # Euler angle sequence from MJCF compiler (default: "xyz")
end

function URDFBuilder(
        robot_name::String,
        meshes::Dict{String, MeshInfo},
        materials::Dict{String, MaterialInfo},
        geom_defaults::Dict{String, GeomDefaults},
        joint_defaults::Dict{String, JointDefaults},
        base_dir::String,
        eulerseq::String = "xyz"
)
    return URDFBuilder(
        IOBuffer(),
        0,
        robot_name,
        Set{String}(),
        Set{String}(),
        meshes,
        materials,
        geom_defaults,
        joint_defaults,
        base_dir,
        String[],
        eulerseq
    )
end

function add_warning!(builder::URDFBuilder, msg::String)
    push!(builder.warnings, msg)
    @warn msg
end

function emit!(builder::URDFBuilder, s::String)
    indent = repeat("  ", builder.indent_level)
    print(builder.io, indent, s, "\n")
end

function emit_raw!(builder::URDFBuilder, s::String)
    print(builder.io, s)
end

function get_urdf(builder::URDFBuilder)
    return String(take!(builder.io))
end

"""
    generate_link_urdf!(builder::URDFBuilder, name::String, geoms::Vector, 
                        inertial::Union{Nothing, EzXML.Node})

Generate URDF <link> element from MJCF body information.
"""
function generate_link_urdf!(
        builder::URDFBuilder,
        name::String,
        geoms::Vector,
        inertial::Union{Nothing, EzXML.Node}
)
    # Ensure unique link name
    link_name = name
    counter = 1
    while link_name in builder.link_names
        link_name = "$(name)_$(counter)"
        counter += 1
    end
    push!(builder.link_names, link_name)

    emit!(builder, "<link name=\"$(link_name)\">")
    builder.indent_level += 1

    # Add inertial if present
    if inertial !== nothing
        generate_inertial_urdf!(builder, inertial)
    end

    # Add visual and collision for each geom
    visual_count = 0
    for geom in geoms
        if generate_geometry_urdf!(builder, geom, "visual", visual_count)
            visual_count += 1
        end
    end

    collision_count = 0
    for geom in geoms
        if generate_geometry_urdf!(builder, geom, "collision", collision_count)
            collision_count += 1
        end
    end

    builder.indent_level -= 1
    emit!(builder, "</link>")

    return link_name
end

"""
    generate_inertial_urdf!(builder::URDFBuilder, inertial::EzXML.Node)

Generate URDF <inertial> element from MJCF inertial.
"""
function generate_inertial_urdf!(builder::URDFBuilder, inertial::EzXML.Node)
    emit!(builder, "<inertial>")
    builder.indent_level += 1

    # Origin
    pos = parse_vec3(get_attr(inertial, "pos", "0 0 0"))
    quat = parse_quat(get_attr(inertial, "quat", "1 0 0 0"))
    rpy = quat_to_rpy(quat)
    emit!(builder, "<origin xyz=\"$(format_vec3(pos))\" rpy=\"$(format_vec3(rpy))\"/>")

    # Mass
    mass_str = get_attr(inertial, "mass", "0.001")
    mass = tryparse(Float64, mass_str)
    if mass === nothing
        mass = 0.001
    end
    emit!(builder, "<mass value=\"$(mass)\"/>")

    # Inertia - MJCF uses diaginertia, URDF uses full inertia tensor
    diaginertia_str = get_attr(inertial, "diaginertia", "0.001 0.001 0.001")
    diag = parse_vec3(diaginertia_str, [0.001, 0.001, 0.001])
    # URDF inertia: ixx, ixy, ixz, iyy, iyz, izz
    # For diagonal inertia, off-diagonal terms are 0
    emit!(builder,
        "<inertia ixx=\"$(diag[1])\" ixy=\"0\" ixz=\"0\" iyy=\"$(diag[2])\" iyz=\"0\" izz=\"$(diag[3])\"/>")

    builder.indent_level -= 1
    emit!(builder, "</inertial>")
end

"""
    generate_geometry_urdf!(builder::URDFBuilder, geom::EzXML.Node, 
                            geom_type::String, index::Int) -> Bool

Generate URDF <visual> or <collision> element from MJCF geom.
Returns true if geometry was generated.
"""
function generate_geometry_urdf!(
        builder::URDFBuilder,
        geom::EzXML.Node,
        geom_type::String,
        index::Int
)
    # Apply defaults from class if present
    props = apply_geom_defaults(geom, builder.geom_defaults)
    mjcf_type = props.type

    # Skip geoms with contype="0" conaffinity="0" for collision
    # (these are visual-only in MuJoCo)
    if geom_type == "collision"
        if props.contype == "0" && props.conaffinity == "0"
            return false
        end
    end

    # Skip collision-only geoms for visual processing
    if geom_type == "visual"
        geom_class = get_attr(geom, "class", "")
        # For visual type, skip if class name contains "collision" but not "visual"
        if occursin("collision", lowercase(geom_class)) &&
           !occursin("visual", lowercase(geom_class))
            return false
        end
    end

    geom_name = get_attr(geom, "name", "")
    suffix = index > 0 ? "_$(index)" : ""
    element_name = isempty(geom_name) ? "" : " name=\"$(geom_name)$(suffix)\""

    emit!(builder, "<$(geom_type)$(element_name)>")
    builder.indent_level += 1

    # Origin - prefer euler if present, otherwise use quat
    pos = parse_vec3(props.pos, [0.0, 0.0, 0.0])
    if !isempty(props.euler)
        # MJCF euler is in radians
        euler = parse_vec3(props.euler, [0.0, 0.0, 0.0])
        emit!(builder, "<origin xyz=\"$(format_vec3(pos))\" rpy=\"$(format_vec3(euler))\"/>")
    else
        quat = parse_quat(props.quat)
        rpy = quat_to_rpy(quat)
        emit!(builder, "<origin xyz=\"$(format_vec3(pos))\" rpy=\"$(format_vec3(rpy))\"/>")
    end

    # Geometry
    emit!(builder, "<geometry>")
    builder.indent_level += 1

    if mjcf_type == "mesh"
        mesh_name = props.mesh
        if haskey(builder.meshes, mesh_name)
            mesh_info = builder.meshes[mesh_name]
            # Use meshes/ prefix for URL routing via the asset server
            # The resolved_file is relative to project root
            mesh_path = "meshes/" * mesh_info.resolved_file
            # Include scale if it's not identity (1, 1, 1)
            scale = mesh_info.scale
            if scale ≈ [1.0, 1.0, 1.0]
                emit!(builder, "<mesh filename=\"$(mesh_path)\"/>")
            else
                emit!(builder,
                    "<mesh filename=\"$(mesh_path)\" scale=\"$(format_vec3(scale))\"/>")
            end
        else
            add_warning!(builder, "Mesh not found: $(mesh_name)")
            emit!(builder, "<sphere radius=\"0.01\"/>")
        end
    elseif mjcf_type == "box"
        size_str = !isempty(props.size) ? props.size : "0.01 0.01 0.01"
        size = parse_vec3(size_str, [0.01, 0.01, 0.01])
        # MJCF size is half-extents, URDF is full size
        full_size = size .* 2.0
        emit!(builder, "<box size=\"$(format_vec3(full_size))\"/>")
    elseif mjcf_type == "sphere"
        size_str = !isempty(props.size) ? props.size : "0.01"
        parts = split(strip(size_str))
        radius = length(parts) >= 1 ? tryparse(Float64, parts[1]) : 0.01
        if radius === nothing
            radius = 0.01
        end
        emit!(builder, "<sphere radius=\"$(radius)\"/>")
    elseif mjcf_type == "cylinder"
        size_str = !isempty(props.size) ? props.size : "0.01 0.01"
        parts = split(strip(size_str))
        radius = length(parts) >= 1 ? tryparse(Float64, parts[1]) : 0.01
        half_length = length(parts) >= 2 ? tryparse(Float64, parts[2]) : 0.01
        if radius === nothing
            radius = 0.01
        end
        if half_length === nothing
            half_length = 0.01
        end
        emit!(builder, "<cylinder radius=\"$(radius)\" length=\"$(half_length * 2)\"/>")
    elseif mjcf_type == "capsule"
        # URDF doesn't have capsule, approximate with cylinder
        size_str = !isempty(props.size) ? props.size : "0.01 0.01"
        parts = split(strip(size_str))
        radius = length(parts) >= 1 ? tryparse(Float64, parts[1]) : 0.01
        half_length = length(parts) >= 2 ? tryparse(Float64, parts[2]) : 0.01
        if radius === nothing
            radius = 0.01
        end
        if half_length === nothing
            half_length = 0.01
        end
        add_warning!(builder, "Capsule geometry approximated as cylinder")
        emit!(builder, "<cylinder radius=\"$(radius)\" length=\"$(half_length * 2)\"/>")
    elseif mjcf_type == "plane"
        # Approximate plane as a large thin box
        add_warning!(builder, "Plane geometry approximated as thin box")
        emit!(builder, "<box size=\"10 10 0.001\"/>")
    else
        add_warning!(builder, "Unsupported geometry type: $(mjcf_type)")
        emit!(builder, "<sphere radius=\"0.01\"/>")
    end

    builder.indent_level -= 1
    emit!(builder, "</geometry>")

    # Material (only for visual)
    if geom_type == "visual"
        # Try to get rgba color: first from explicit rgba, then from material reference
        rgba_str = ""

        if !isempty(props.rgba)
            # Direct rgba attribute on geom
            rgba_str = props.rgba
        elseif !isempty(props.material)
            # Look up material by name
            if haskey(builder.materials, props.material)
                rgba_str = builder.materials[props.material].rgba
            end
        end

        if !isempty(rgba_str)
            parts = split(strip(rgba_str))
            if length(parts) >= 4
                emit!(builder, "<material name=\"material_$(index)\">")
                builder.indent_level += 1
                emit!(builder, "<color rgba=\"$(rgba_str)\"/>")
                builder.indent_level -= 1
                emit!(builder, "</material>")
            end
        end
    end

    builder.indent_level -= 1
    emit!(builder, "</$(geom_type)>")

    return true
end

"""
    generate_joint_urdf!(builder::URDFBuilder, joint::EzXML.Node, 
                         parent_link::String, child_link::String,
                         body_pos::Vector{Float64}, body_rpy::Vector{Float64})

Generate URDF <joint> element from MJCF joint.
"""
function generate_joint_urdf!(
        builder::URDFBuilder,
        joint::EzXML.Node,
        parent_link::String,
        child_link::String,
        body_pos::Vector{Float64},
        body_rpy::Vector{Float64}
)
    # Apply joint defaults from class if present
    props = apply_joint_defaults(joint, builder.joint_defaults)

    joint_name = get_attr(joint, "name", "joint_$(length(builder.joint_names))")

    # Ensure unique joint name
    base_name = joint_name
    counter = 1
    while joint_name in builder.joint_names
        joint_name = "$(base_name)_$(counter)"
        counter += 1
    end
    push!(builder.joint_names, joint_name)

    urdf_type = mjcf_joint_type_to_urdf(props.type)

    emit!(builder, "<joint name=\"$(joint_name)\" type=\"$(urdf_type)\">")
    builder.indent_level += 1

    # Parent and child
    emit!(builder, "<parent link=\"$(parent_link)\"/>")
    emit!(builder, "<child link=\"$(child_link)\"/>")

    # Origin (from body pos/rpy, as joint is defined within body)
    emit!(builder, "<origin xyz=\"$(format_vec3(body_pos))\" rpy=\"$(format_vec3(body_rpy))\"/>")

    # Axis - use defaults if not explicitly specified
    axis = parse_vec3(props.axis, [0.0, 0.0, 1.0])
    emit!(builder, "<axis xyz=\"$(format_vec3(axis))\"/>")

    # Limits (for revolute and prismatic)
    if urdf_type in ["revolute", "prismatic"]
        if !isempty(props.range)
            parts = split(strip(props.range))
            if length(parts) >= 2
                lower = tryparse(Float64, parts[1])
                upper = tryparse(Float64, parts[2])
                if lower !== nothing && upper !== nothing
                    # MJCF angles may be in radians or degrees depending on compiler
                    # We assume radians here (common for programmatic MJCF)
                    effort = 100.0  # Default effort limit
                    velocity = 10.0  # Default velocity limit
                    emit!(builder,
                        "<limit lower=\"$(lower)\" upper=\"$(upper)\" effort=\"$(effort)\" velocity=\"$(velocity)\"/>")
                end
            end
        else
            # No range specified, use defaults
            if urdf_type == "revolute"
                emit!(builder,
                    "<limit lower=\"-3.14159\" upper=\"3.14159\" effort=\"100\" velocity=\"10\"/>")
            else
                emit!(builder,
                    "<limit lower=\"-1\" upper=\"1\" effort=\"100\" velocity=\"1\"/>")
            end
        end
    end

    builder.indent_level -= 1
    emit!(builder, "</joint>")
end

"""
    generate_fixed_joint_urdf!(builder::URDFBuilder, parent_link::String, 
                                child_link::String, pos::Vector{Float64}, 
                                rpy::Vector{Float64})

Generate a fixed joint between parent and child links.
Used when a body has no explicit joint.
"""
function generate_fixed_joint_urdf!(
        builder::URDFBuilder,
        parent_link::String,
        child_link::String,
        pos::Vector{Float64},
        rpy::Vector{Float64}
)
    joint_name = "$(child_link)_fixed_joint"

    # Ensure unique joint name
    base_name = joint_name
    counter = 1
    while joint_name in builder.joint_names
        joint_name = "$(base_name)_$(counter)"
        counter += 1
    end
    push!(builder.joint_names, joint_name)

    emit!(builder, "<joint name=\"$(joint_name)\" type=\"fixed\">")
    builder.indent_level += 1

    emit!(builder, "<parent link=\"$(parent_link)\"/>")
    emit!(builder, "<child link=\"$(child_link)\"/>")

    emit!(builder, "<origin xyz=\"$(format_vec3(pos))\" rpy=\"$(format_vec3(rpy))\"/>")

    builder.indent_level -= 1
    emit!(builder, "</joint>")
end

"""
    process_body_recursive!(builder::URDFBuilder, body::EzXML.Node, 
                            parent_link::String)

Recursively process MJCF body elements, generating URDF links and joints.
"""
function process_body_recursive!(
        builder::URDFBuilder,
        body::EzXML.Node,
        parent_link::String
)
    body_name = get_attr(body, "name", "link_$(length(builder.link_names))")
    body_pos = parse_vec3(get_attr(body, "pos", "0 0 0"))

    # Check for euler first (more common in MJCF), then fall back to quat
    # Convert MJCF euler angles to URDF rpy based on the eulerseq convention
    body_euler_str = get_attr(body, "euler", "")
    if !isempty(body_euler_str)
        euler_angles = parse_vec3(body_euler_str, [0.0, 0.0, 0.0])
        body_rpy = convert_mjcf_euler_to_urdf_rpy(euler_angles, builder.eulerseq)
    else
        body_quat = parse_quat(get_attr(body, "quat", "1 0 0 0"))
        body_rpy = quat_to_rpy(body_quat)
    end

    # Collect geoms and inertial
    geoms = EzXML.Node[]
    inertial = nothing
    joints = EzXML.Node[]
    child_bodies = EzXML.Node[]

    for child in EzXML.eachelement(body)
        name = EzXML.nodename(child)
        if name == "geom"
            push!(geoms, child)
        elseif name == "inertial"
            inertial = child
        elseif name == "joint"
            push!(joints, child)
        elseif name == "body"
            push!(child_bodies, child)
        end
        # Skip: camera, site, light, freejoint, etc. (URDF doesn't support these)
    end

    # Generate link
    link_name = generate_link_urdf!(builder, body_name, geoms, inertial)

    # Generate joint(s)
    if isempty(joints)
        # No explicit joint - create fixed joint
        generate_fixed_joint_urdf!(builder, parent_link, link_name, body_pos, body_rpy)
    else
        # Generate a joint for the first joint definition
        # URDF only supports one joint per link pair
        if length(joints) > 1
            add_warning!(builder,
                "Body '$(body_name)' has $(length(joints)) joints, only first will be used")
        end
        generate_joint_urdf!(
            builder, joints[1], parent_link, link_name, body_pos, body_rpy)
    end

    # Process child bodies recursively
    for child_body in child_bodies
        process_body_recursive!(builder, child_body, link_name)
    end
end

"""
    generate_urdf_from_mjcf(mjcf_path::String) -> String

Convert a MuJoCo XML (MJCF) file to URDF format.

# Arguments
- `mjcf_path`: Path to the MuJoCo XML file

# Returns
- URDF content as a string

# Notes
- Resolves <include> directives recursively
- Resolves default class attributes for geoms
- Converts body hierarchy to link/joint structure
- Maps MuJoCo joint types to URDF equivalents
- Mesh paths are converted to URL-friendly paths with meshes/ prefix
- Skips unsupported MuJoCo features (tendons, actuators, sensors, etc.)

# Example
```julia
urdf = generate_urdf_from_mjcf("robots/trossen/scene.xml", ".")
write("robot.urdf", urdf)
```
"""
function generate_urdf_from_mjcf(mjcf_path::String, project_root::String = ".")
    if !isfile(mjcf_path)
        throw(MJCFParseError("File not found: $mjcf_path"))
    end

    base_dir = dirname(abspath(mjcf_path))
    abs_project_root = abspath(project_root)

    # Parse MJCF
    doc = EzXML.readxml(mjcf_path)

    # Resolve includes first
    resolve_mjcf_includes(doc, base_dir)

    # Resolve <attach> directives (for model references)
    resolve_attach_directives!(doc, base_dir)

    root = EzXML.root(doc)

    # Get robot name from mujoco model attribute
    robot_name = get_attr(root, "model", "robot")

    # Extract mesh assets with project root for path resolution
    meshes = extract_mesh_assets(doc, base_dir, abs_project_root)

    # Extract material definitions for color support
    materials = extract_materials(doc)

    # Extract default classes for geom and joint property inheritance
    (geom_defaults, joint_defaults) = extract_default_classes(doc)

    # Extract compiler settings for euler angle convention
    # MJCF default eulerseq is "xyz" (intrinsic rotations)
    compiler_nodes = EzXML.findall("//compiler", root)
    eulerseq = "xyz"  # MJCF default
    for compiler in compiler_nodes
        seq = get_attr(compiler, "eulerseq", "")
        if !isempty(seq)
            eulerseq = seq
            break
        end
    end

    # Create builder
    builder = URDFBuilder(
        robot_name, meshes, materials, geom_defaults, joint_defaults, base_dir, eulerseq)

    # Start URDF document
    emit!(builder, "<?xml version=\"1.0\"?>")
    emit!(builder, "<robot name=\"$(robot_name)\">")
    builder.indent_level += 1

    # Find worldbody
    worldbody_nodes = EzXML.findall("//worldbody", root)
    if isempty(worldbody_nodes)
        throw(MJCFParseError("No <worldbody> element found in MJCF"))
    end

    # Create a base link for the world
    emit!(builder, "<link name=\"world\"/>")
    push!(builder.link_names, "world")

    # Process all top-level bodies in worldbody
    for worldbody in worldbody_nodes
        for child in EzXML.eachelement(worldbody)
            if EzXML.nodename(child) == "body"
                process_body_recursive!(builder, child, "world")
            elseif EzXML.nodename(child) == "geom"
                # World-level geoms (like floor) - skip for URDF
                add_warning!(builder, "Skipping world-level geom (not supported in URDF)")
            end
        end
    end

    # Close robot element
    builder.indent_level -= 1
    emit!(builder, "</robot>")

    # Report warnings
    if !isempty(builder.warnings)
        @info "MJCF to URDF conversion completed with $(length(builder.warnings)) warnings"
    end

    return get_urdf(builder)
end

"""
    get_or_generate_urdf(mjcf_path::String, cache_dir::String, project_root::String=".") -> String

Get cached URDF or generate from MJCF if cache is stale.

Uses file modification time for cache invalidation.

# Arguments
- `mjcf_path`: Path to the MuJoCo XML file
- `cache_dir`: Directory to store cached URDF files
- `project_root`: Project root directory for resolving mesh paths (default: ".")

# Returns
- URDF content as a string

# Example
```julia
urdf = get_or_generate_urdf("robots/trossen/scene.xml", ".cache/urdf", ".")
```
"""
function get_or_generate_urdf(
        mjcf_path::String, cache_dir::String, project_root::String = ".")
    if !isfile(mjcf_path)
        throw(MJCFParseError("File not found: $mjcf_path"))
    end

    # Create cache directory if it doesn't exist
    if !isdir(cache_dir)
        mkpath(cache_dir)
    end

    # Generate cache file path based on MJCF path AND project root (since paths depend on it)
    cache_input = mjcf_path * "|" * abspath(project_root)
    cache_key = bytes2hex(sha256(cache_input))
    cache_file = joinpath(cache_dir, "$(cache_key).urdf")
    cache_meta = joinpath(cache_dir, "$(cache_key).meta")

    # Check if cache is valid
    mjcf_mtime = mtime(mjcf_path)
    cache_valid = false

    if isfile(cache_file) && isfile(cache_meta)
        try
            meta_content = read(cache_meta, String)
            cached_mtime = parse(Float64, strip(meta_content))
            if cached_mtime >= mjcf_mtime
                cache_valid = true
            end
        catch
            cache_valid = false
        end
    end

    if cache_valid
        @info "Using cached URDF: $cache_file"
        return read(cache_file, String)
    end

    # Generate URDF
    @info "Generating URDF from: $mjcf_path"
    urdf_content = generate_urdf_from_mjcf(mjcf_path, project_root)

    # Write to cache
    write(cache_file, urdf_content)
    write(cache_meta, string(mjcf_mtime))

    return urdf_content
end

# Simple SHA256 implementation for cache keys (using stdlib)
using SHA

# Export public functions
export generate_urdf_from_mjcf, get_or_generate_urdf, MJCFParseError
