#!/usr/bin/env julia
#
# Convert ROS-Industrial Fanuc URDF/xacro robot descriptions to MuJoCo XML format.
#
# This script parses the xacro macro files from the fanuc-industrial repository
# and generates MuJoCo-compatible XML with proper mesh paths, colors, and actuators.
#
# Usage:
#     # Generate all default variants (one per family)
#     julia --project=. scripts/convert_fanuc_industrial.jl
#
#     # Generate specific variant(s)
#     julia --project=. scripts/convert_fanuc_industrial.jl m10ia7l crx10ial
#
#     # List available variants
#     julia --project=. scripts/convert_fanuc_industrial.jl --list

using EzXML
using Printf

# --- Constants ---

# Fanuc colors from common_colours.xacro
# These are the RGBA values used for visual geoms
const FANUC_COLORS = Dict{String, NTuple{4, Float64}}(
    "material_fanuc_yellow" => (0.96, 0.76, 0.13, 1.0),
    "material_fanuc_white" => (0.86, 0.85, 0.81, 1.0),
    "material_fanuc_grey" => (0.34, 0.35, 0.36, 1.0),
    "material_fanuc_gray24" => (0.239, 0.239, 0.239, 1.0),
    "material_fanuc_gray28" => (0.278, 0.278, 0.278, 1.0),
    "material_fanuc_gray31" => (0.310, 0.310, 0.310, 1.0),
    "material_fanuc_gray40" => (0.400, 0.400, 0.400, 1.0),
    "material_fanuc_yellow1" => (1.0, 1.0, 0.0, 1.0),
    "material_fanuc_BAB0B0" => (0.729, 0.690, 0.690, 1.0),
    "material_fanuc_E6E6E6" => (0.902, 0.902, 0.902, 1.0),
    "material_fanuc_00FF73" => (0.0, 1.0, 0.451, 1.0),
    "material_fanuc_black" => (0.15, 0.15, 0.15, 1.0),
    "material_fanuc_greyish" => (0.75, 0.75, 0.75, 1.0),
    "material_fanuc_green" => (0.0, 0.608, 0.275, 1.0)
)

# Default color for links without material specification
const DEFAULT_COLOR = (0.7, 0.7, 0.7, 1.0)

# Fixed mass and inertia for all links (keeping it simple)
const LINK_MASS = 1.0
const LINK_INERTIA = "0.01 0.01 0.01"

# Default variants for each family (one per family for generation)
const DEFAULT_VARIANTS = Dict{String, String}(
    "cr35ia" => "cr35ia",
    "cr7ia" => "cr7ia",
    "crx10ia" => "crx10ial",
    "lrmate200i" => "lrmate200i",
    "lrmate200ib" => "lrmate200ib",
    "lrmate200ic" => "lrmate200ic",
    "lrmate200id" => "lrmate200id",
    "m10ia" => "m10ia",
    "m16ib" => "m16ib20",
    "m20ia" => "m20ia",
    "m20ib" => "m20ib25",
    "m430ia" => "m430ia2f",
    "m6ib" => "m6ib",
    "m710ic" => "m710ic50",
    "m900ia" => "m900ia260l",
    "m900ib" => "m900ib700",
    "r1000ia" => "r1000ia80f",
    "r2000ib" => "r2000ib210f",
    "r2000ic" => "r2000ic165f"
)

# --- Data Types ---

Base.@kwdef struct LinkInfo
    name::String
    visual_mesh::String = ""
    collision_mesh::String = ""
    material::String = ""
end

Base.@kwdef struct JointInfo
    name::String
    type::String = "revolute"
    parent::String = ""
    child::String = ""
    origin_xyz::String = "0 0 0"
    origin_rpy::String = "0 0 0"
    axis::String = "0 0 1"
    lower_limit::Float64 = 0.0
    upper_limit::Float64 = 0.0
end

Base.@kwdef struct RobotConfig
    family::String
    variant::String
    package_dir::String
    xacro_file::String
end

# --- Helper Functions ---

"""
    evaluate_limit(expr::String) -> Float64

Evaluate joint limit expression, handling radians() function and pi references.
"""
function evaluate_limit(expr::String)
    expr = strip(expr)
    isempty(expr) && return 0.0

    # Handle \${radians(...)} or radians(...)
    m = match(r"radians\s*\(\s*(-?[\d.]+)\s*\)", expr)
    if m !== nothing
        return deg2rad(parse(Float64, m.captures[1]))
    end

    # Handle \${pi}, \${-pi}, etc.
    if occursin("pi", lowercase(expr))
        expr = replace(expr, r"\$\{" => "")
        expr = replace(expr, r"\}" => "")
        expr = replace(expr, r"pi"i => string(π))
        try
            return Float64(eval(Meta.parse(expr)))
        catch
            return 0.0
        end
    end

    # Try direct parse
    result = tryparse(Float64, expr)
    return isnothing(result) ? 0.0 : result
end

"""
    rpy_to_quat(rpy_str::String) -> String

Convert roll-pitch-yaw string to quaternion string for MuJoCo.
"""
function rpy_to_quat(rpy_str::String)
    try
        parts = split(strip(rpy_str))
        length(parts) != 3 && return "1 0 0 0"

        roll, pitch, yaw = parse.(Float64, parts)

        # Convert RPY to quaternion (ZYX convention)
        cr, sr = cos(roll / 2), sin(roll / 2)
        cp, sp = cos(pitch / 2), sin(pitch / 2)
        cy, sy = cos(yaw / 2), sin(yaw / 2)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        # MuJoCo uses w, x, y, z order
        return @sprintf("%.6f %.6f %.6f %.6f", w, x, y, z)
    catch
        return "1 0 0 0"
    end
end

"""
    get_color_rgba(material::String) -> String

Get RGBA string for a material name.
"""
function get_color_rgba(material::String)
    color = get(FANUC_COLORS, material, DEFAULT_COLOR)
    return @sprintf("%.3f %.3f %.3f %.3f", color[1], color[2], color[3], color[4])
end

"""
    clean_prefix(s::String) -> String

Remove \${prefix} from a string.
"""
clean_prefix(s::String) = replace(s, "\${prefix}" => "")

"""
    get_attr(node::EzXML.Node, name::String, default::String="") -> String

Get attribute value with default.
"""
function get_attr(node::EzXML.Node, name::String, default::String = "")
    return haskey(node, name) ? node[name] : default
end

# --- Robot Discovery ---

"""
    find_all_robots(fanuc_dir::String) -> Dict{String, RobotConfig}

Discover all robot configurations in the fanuc-industrial directory.
"""
function find_all_robots(fanuc_dir::String)
    robots = Dict{String, RobotConfig}()

    !isdir(fanuc_dir) && return robots

    for entry in readdir(fanuc_dir)
        support_dir = joinpath(fanuc_dir, entry)

        # Check if it's a fanuc_*_support directory
        !isdir(support_dir) && continue
        !startswith(entry, "fanuc_") && continue
        !endswith(entry, "_support") && continue

        # Extract family name
        family = replace(entry, "fanuc_" => "", "_support" => "")

        urdf_dir = joinpath(support_dir, "urdf")
        !isdir(urdf_dir) && continue

        # Find all xacro files
        for xacro_file in readdir(urdf_dir)
            !endswith(xacro_file, "_macro.xacro") && continue

            # Extract variant name
            variant = replace(xacro_file, "_macro.xacro" => "")

            # Verify meshes exist
            mesh_dir = joinpath(support_dir, "meshes", variant)
            !isdir(mesh_dir) && continue

            robots[variant] = RobotConfig(
                family = family,
                variant = variant,
                package_dir = support_dir,
                xacro_file = joinpath(urdf_dir, xacro_file)
            )
        end
    end

    return robots
end

# --- Xacro Parsing ---

"""
    find_macro_element(root::EzXML.Node) -> EzXML.Node

Find the main macro element with the most links (the actual robot definition).
"""
function find_macro_element(root::EzXML.Node)
    # Collect all macro elements
    macros = EzXML.Node[]

    function find_macros(node::EzXML.Node)
        name = nodename(node)
        # Check for xacro:macro or macro with xacro namespace
        if name == "macro" || endswith(name, ":macro")
            push!(macros, node)
        end
        for child in eachelement(node)
            find_macros(child)
        end
    end

    find_macros(root)

    isempty(macros) && error("No xacro:macro found")

    # Pick the macro with the most link elements
    function count_links(macro_node::EzXML.Node)
        count = 0
        function traverse(n::EzXML.Node)
            if nodename(n) == "link"
                count += 1
            end
            for child in eachelement(n)
                traverse(child)
            end
        end
        traverse(macro_node)
        return count
    end

    return argmax(m -> count_links(m), macros)
end

"""
    find_elements(parent::EzXML.Node, tag::String) -> Vector{EzXML.Node}

Find all descendant elements with the given tag name.
"""
function find_elements(parent::EzXML.Node, tag::String)
    results = EzXML.Node[]

    function traverse(node::EzXML.Node)
        if nodename(node) == tag
            push!(results, node)
        end
        for child in eachelement(node)
            traverse(child)
        end
    end

    traverse(parent)
    return results
end

"""
    find_element(parent::EzXML.Node, path::String) -> Union{EzXML.Node, Nothing}

Find first child element matching a simple path like "geometry/mesh".
"""
function find_element(parent::EzXML.Node, path::String)
    parts = split(path, "/")
    current = parent

    for part in parts
        found = nothing
        for child in eachelement(current)
            if nodename(child) == part
                found = child
                break
            end
        end
        found === nothing && return nothing
        current = found
    end

    return current
end

"""
    parse_xacro(xacro_path::String) -> Tuple{Vector{LinkInfo}, Vector{JointInfo}}

Parse a xacro macro file and extract link/joint information.
"""
function parse_xacro(xacro_path::String)
    doc = readxml(xacro_path)
    root_elem = root(doc)

    # Find the main macro element
    macro_elem = find_macro_element(root_elem)

    links = LinkInfo[]
    joints = JointInfo[]

    # Parse links
    for link in find_elements(macro_elem, "link")
        name = clean_prefix(get_attr(link, "name"))

        # Skip helper links
        name in ("base", "flange", "tool0", "") && continue

        visual_mesh = ""
        collision_mesh = ""
        material = ""

        # Find visual element
        for child in eachelement(link)
            if nodename(child) == "visual"
                mesh = find_element(child, "geometry/mesh")
                if mesh !== nothing
                    visual_mesh = get_attr(mesh, "filename")
                end

                # Find material macro (xacro:material_fanuc_*)
                for vchild in eachelement(child)
                    cname = nodename(vchild)
                    if startswith(cname, "material_fanuc")
                        material = cname
                    elseif occursin(":", cname)
                        # Handle namespaced elements like xacro:material_fanuc_yellow
                        local_name = split(cname, ":")[end]
                        if startswith(local_name, "material_fanuc")
                            material = local_name
                        end
                    end
                end
            elseif nodename(child) == "collision"
                mesh = find_element(child, "geometry/mesh")
                if mesh !== nothing
                    collision_mesh = get_attr(mesh, "filename")
                end
            end
        end

        push!(links,
            LinkInfo(
                name = name,
                visual_mesh = visual_mesh,
                collision_mesh = collision_mesh,
                material = material
            ))
    end

    # Parse joints
    for joint in find_elements(macro_elem, "joint")
        name = clean_prefix(get_attr(joint, "name"))
        jtype = get_attr(joint, "type", "fixed")

        # Skip fixed joints
        jtype == "fixed" && continue

        parent_link = ""
        child_link = ""
        origin_xyz = "0 0 0"
        origin_rpy = "0 0 0"
        axis_xyz = "0 0 1"
        lower = 0.0
        upper = 0.0

        for child in eachelement(joint)
            cname = nodename(child)
            if cname == "parent"
                parent_link = clean_prefix(get_attr(child, "link"))
            elseif cname == "child"
                child_link = clean_prefix(get_attr(child, "link"))
            elseif cname == "origin"
                origin_xyz = get_attr(child, "xyz", "0 0 0")
                origin_rpy = get_attr(child, "rpy", "0 0 0")
            elseif cname == "axis"
                axis_xyz = get_attr(child, "xyz", "0 0 1")
            elseif cname == "limit"
                lower = evaluate_limit(get_attr(child, "lower", "0"))
                upper = evaluate_limit(get_attr(child, "upper", "0"))
            end
        end

        push!(joints,
            JointInfo(
                name = name,
                type = jtype,
                parent = parent_link,
                child = child_link,
                origin_xyz = origin_xyz,
                origin_rpy = origin_rpy,
                axis = axis_xyz,
                lower_limit = lower,
                upper_limit = upper
            ))
    end

    return links, joints
end

# --- MuJoCo XML Generation ---

"""
    generate_mujoco_xml(robot::RobotConfig, links::Vector{LinkInfo}, 
                        joints::Vector{JointInfo}, output_dir::String, 
                        fanuc_dir::String)

Generate MuJoCo XML files for a robot.
"""
function generate_mujoco_xml(robot::RobotConfig, links::Vector{LinkInfo},
        joints::Vector{JointInfo}, output_dir::String, fanuc_dir::String)
    robot_dir = joinpath(output_dir, robot.variant)
    mkpath(robot_dir)

    # Calculate relative path to mesh directories
    visual_mesh_dir = joinpath(robot.package_dir, "meshes", robot.variant, "visual")
    collision_mesh_dir = joinpath(robot.package_dir, "meshes", robot.variant, "collision")

    # Relative path from output dir to mesh dir
    rel_visual_path = joinpath("..", "..", relpath(visual_mesh_dir, dirname(fanuc_dir)))
    rel_collision_path = joinpath(
        "..", "..", relpath(collision_mesh_dir, dirname(fanuc_dir)))

    # Build link and joint maps
    link_map = Dict(l.name => l for l in links)
    joint_map = Dict{String, JointInfo}()  # child -> joint
    children_map = Dict{String, Vector{String}}()  # parent -> [children]

    for j in joints
        joint_map[j.child] = j
        if !haskey(children_map, j.parent)
            children_map[j.parent] = String[]
        end
        push!(children_map[j.parent], j.child)
    end

    # Find root link (not a child of any joint)
    child_links = Set(j.child for j in joints)
    root_link = "base_link"
    for link in links
        if link.name ∉ child_links
            root_link = link.name
            break
        end
    end

    # Build XML using IOBuffer
    io = IOBuffer()

    println(io, "<?xml version=\"1.0\" ?>")
    println(io, "<!-- MuJoCo model for FANUC $(uppercase(robot.variant)) -->")
    println(
        io, "<!-- Auto-generated from fanuc-industrial/$(basename(robot.package_dir)) -->")
    println(io, "<mujoco model=\"$(robot.variant)\">")
    println(io, "  <compiler angle=\"radian\" autolimits=\"true\"/>")
    println(io)
    println(io, "  <default>")
    println(io, "    <default class=\"$(robot.variant)\">")
    println(io, "      <joint damping=\"100\" frictionloss=\"10\" armature=\"1\"/>")
    println(io, "      <position kp=\"1000\" kv=\"100\"/>")
    println(io, "      <default class=\"visual\">")
    println(io, "        <geom type=\"mesh\" contype=\"0\" conaffinity=\"0\" group=\"2\"/>")
    println(io, "      </default>")
    println(io, "      <default class=\"collision\">")
    println(io, "        <geom type=\"mesh\" group=\"3\"/>")
    println(io, "      </default>")
    println(io, "    </default>")
    println(io, "  </default>")
    println(io)
    println(io, "  <asset>")

    # Add mesh assets
    for link in links
        if !isempty(link.visual_mesh)
            mesh_file = basename(link.visual_mesh)
            println(io,
                "    <mesh name=\"$(link.name)_visual\" file=\"$(rel_visual_path)/$(mesh_file)\"/>")
        end
        if !isempty(link.collision_mesh)
            mesh_file = basename(link.collision_mesh)
            println(io,
                "    <mesh name=\"$(link.name)_collision\" file=\"$(rel_collision_path)/$(mesh_file)\"/>")
        end
    end

    println(io, "  </asset>")
    println(io)
    println(io, "  <worldbody>")

    # Recursive function to write nested bodies
    function write_nested_body(link_name::String, indent::Int)
        !haskey(link_map, link_name) && return
        !haskey(joint_map, link_name) && return

        link = link_map[link_name]
        joint = joint_map[link_name]
        pad = "  "^indent

        quat = rpy_to_quat(joint.origin_rpy)
        quat_attr = quat == "1.000000 0.000000 0.000000 0.000000" ? "" : " quat=\"$(quat)\""

        println(io,
            "$(pad)<body name=\"$(link_name)\" pos=\"$(joint.origin_xyz)\"$(quat_attr) childclass=\"$(robot.variant)\">")

        # Add joint
        println(io,
            "$(pad)  <joint name=\"$(joint.name)\" type=\"hinge\" axis=\"$(joint.axis)\" range=\"$(@sprintf("%.4f", joint.lower_limit)) $(@sprintf("%.4f", joint.upper_limit))\"/>")

        # Add inertial
        println(io,
            "$(pad)  <inertial pos=\"0 0 0\" mass=\"$(LINK_MASS)\" diaginertia=\"$(LINK_INERTIA)\"/>")

        # Add visual geom with color
        if !isempty(link.visual_mesh)
            rgba = get_color_rgba(link.material)
            println(io,
                "$(pad)  <geom class=\"visual\" mesh=\"$(link_name)_visual\" rgba=\"$(rgba)\"/>")
        end

        # Add collision geom
        if !isempty(link.collision_mesh)
            println(
                io, "$(pad)  <geom class=\"collision\" mesh=\"$(link_name)_collision\"/>")
        end

        # Recursively add children
        if haskey(children_map, link_name)
            for child in children_map[link_name]
                write_nested_body(child, indent + 1)
            end
        end

        println(io, "$(pad)</body>")
    end

    # Write the root body
    function write_body(link_name::String, indent::Int)
        !haskey(link_map, link_name) && return

        link = link_map[link_name]
        pad = "  "^indent

        println(io, "$(pad)<body name=\"$(link_name)\" childclass=\"$(robot.variant)\">")

        # Add inertial
        println(io,
            "$(pad)  <inertial pos=\"0 0 0\" mass=\"$(LINK_MASS)\" diaginertia=\"$(LINK_INERTIA)\"/>")

        # Add visual geom with color
        if !isempty(link.visual_mesh)
            rgba = get_color_rgba(link.material)
            println(io,
                "$(pad)  <geom class=\"visual\" mesh=\"$(link_name)_visual\" rgba=\"$(rgba)\"/>")
        end

        # Add collision geom
        if !isempty(link.collision_mesh)
            println(
                io, "$(pad)  <geom class=\"collision\" mesh=\"$(link_name)_collision\"/>")
        end

        # Add child bodies
        if haskey(children_map, link_name)
            for child_name in children_map[link_name]
                write_nested_body(child_name, indent + 1)
            end
        end

        println(io, "$(pad)</body>")
    end

    write_body(root_link, 2)

    println(io, "  </worldbody>")
    println(io)
    println(io, "  <contact>")
    println(
        io, "    <!-- Exclude collisions between adjacent links (parent-child pairs) -->")

    # Add contact exclusions for adjacent body pairs
    for j in joints
        println(io, "    <exclude body1=\"$(j.parent)\" body2=\"$(j.child)\"/>")
    end

    println(io, "  </contact>")
    println(io)
    println(io, "  <actuator>")

    # Add position actuators for each joint (sorted by name)
    for j in sort(joints, by = j -> j.name)
        println(io,
            "    <position name=\"$(j.name)_actuator\" joint=\"$(j.name)\" class=\"$(robot.variant)\"/>")
    end

    println(io, "  </actuator>")
    println(io, "</mujoco>")

    # Write robot XML
    robot_xml_path = joinpath(robot_dir, "$(robot.variant).xml")
    write(robot_xml_path, String(take!(io)))
    println("  Created $(robot_xml_path)")

    # Create scene XML
    scene_xml = """<?xml version="1.0" ?>
<!-- Scene file for FANUC $(uppercase(robot.variant)) -->
<mujoco model="scene">
  <include file="$(robot.variant).xml"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="120" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
             rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8"
             width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true"
              texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <worldbody>
    <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
"""

    scene_xml_path = joinpath(robot_dir, "scene.xml")
    write(scene_xml_path, scene_xml)
    println("  Created $(scene_xml_path)")
end

# --- Main ---

function main(args = ARGS)
    # Parse arguments
    list_mode = "--list" in args || "-l" in args
    variants_to_convert = filter(a -> !startswith(a, "-"), args)

    # Find project root
    script_dir = @__DIR__
    project_root = dirname(script_dir)
    fanuc_dir = joinpath(project_root, "robots", "fanuc-industrial")

    if !isdir(fanuc_dir)
        println("Error: fanuc-industrial directory not found at $(fanuc_dir)")
        return 1
    end

    # Discover all robots
    all_robots = find_all_robots(fanuc_dir)

    if isempty(all_robots)
        println("No robots found!")
        return 1
    end

    if list_mode
        println("Available variants ($(length(all_robots))):\n")

        # Group by family
        by_family = Dict{String, Vector{String}}()
        for (variant, config) in all_robots
            if !haskey(by_family, config.family)
                by_family[config.family] = String[]
            end
            push!(by_family[config.family], variant)
        end

        for family in sort(collect(keys(by_family)))
            default = get(DEFAULT_VARIANTS, family, "")
            variants = by_family[family]
            variant_strs = [v == default ? "$(v)*" : v for v in sort(variants)]
            println("  $(family): $(join(variant_strs, ", "))")
        end

        println("\n  * = default variant (generated with no arguments)")
        return 0
    end

    # Determine which variants to convert
    if !isempty(variants_to_convert)
        valid_variants = String[]
        for v in variants_to_convert
            if haskey(all_robots, v)
                push!(valid_variants, v)
            else
                println("Warning: Unknown variant '$(v)', skipping")
            end
        end
        variants_to_convert = valid_variants
    else
        # Convert all default variants
        variants_to_convert = collect(values(DEFAULT_VARIANTS))
    end

    output_dir = joinpath(project_root, "robots", "fanuc_mujoco")
    mkpath(output_dir)

    println("Converting $(length(variants_to_convert)) variant(s) to $(output_dir)...\n")

    success_count = 0
    for variant in sort(variants_to_convert)
        if !haskey(all_robots, variant)
            println("Skipping $(variant): not found")
            continue
        end

        robot = all_robots[variant]
        println("Processing $(variant) ($(robot.family))...")

        try
            links, joints = parse_xacro(robot.xacro_file)
            println("  Parsed $(length(links)) links, $(length(joints)) joints")

            generate_mujoco_xml(robot, links, joints, output_dir, fanuc_dir)
            success_count += 1
        catch e
            println("  Error: $(e)")
            showerror(stdout, e, catch_backtrace())
            println()
        end
    end

    println("\nDone! Successfully converted $(success_count)/$(length(variants_to_convert)) robots.")
    return success_count == length(variants_to_convert) ? 0 : 1
end

# Run if executed directly
if abspath(PROGRAM_FILE) == @__FILE__
    exit(main())
end
