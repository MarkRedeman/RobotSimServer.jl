# Scene Builder - Programmatically add objects to MuJoCo scenes
#
# This module allows adding graspable objects (cubes, etc.) and body-attached
# cameras to a MuJoCo scene without modifying the original XML files.
#
# Usage:
#   include("scene_builder.jl")
#   
#   cubes = [
#       CubeSpec(name="red_cube", pos=[0.15, 0.0, 0.025], color=[0.9, 0.2, 0.2, 1.0]),
#       CubeSpec(name="green_cube", pos=[0.15, 0.08, 0.025], color=[0.2, 0.9, 0.2, 1.0]),
#   ]
#   
#   cameras = [
#       BodyCamera(name="gripper_cam", body="gripper", pos=[0, 0, -0.05], 
#                  quat=[0.0, 0.707, 0.707, 0.0], fovy=60.0),
#   ]
#   
#   model, data = build_scene("path/to/scene.xml", cubes, cameras=cameras)

using MuJoCo
using MuJoCo.LibMuJoCo

"""
    CubeSpec

Specification for a graspable cube in the scene.

# Fields
- `name::String`: Unique identifier for the cube (used for body/joint/geom names)
- `pos::Vector{Float64}`: [x, y, z] position in meters
- `size::Float64`: Half-extent in meters (default 0.015 = 3cm cube)
- `color::Vector{Float64}`: RGBA color [r, g, b, a] (default red)
- `mass::Float64`: Mass in kg (default 0.015 = 15g)
- `friction::Float64`: Sliding friction coefficient (default 5.0)
"""
Base.@kwdef struct CubeSpec
    name::String
    pos::Vector{Float64}
    size::Float64 = 0.015                         # Half-extent (0.015 = 3cm cube)
    color::Vector{Float64} = [0.9, 0.2, 0.2, 1.0] # RGBA
    mass::Float64 = 0.015                         # kg
    friction::Float64 = 5.0                       # Sliding friction (very high)
end

"""
    BodyCamera

Specification for a camera attached to a body (moves with the body).

# Fields
- `name::String`: Unique camera name (used to reference in capture config)
- `body::String`: Name of the body to attach to (e.g., "gripper")
- `pos::Vector{Float64}`: Position relative to body frame [x, y, z]
- `quat::Vector{Float64}`: Orientation quaternion [w, x, y, z] - camera looks along -Z
- `fovy::Float64`: Vertical field of view in degrees (default 60.0)
"""
Base.@kwdef struct BodyCamera
    name::String
    body::String
    pos::Vector{Float64} = [0.0, 0.0, 0.0]
    quat::Vector{Float64} = [1.0, 0.0, 0.0, 0.0]  # Identity (looks along -Z of body)
    fovy::Float64 = 60.0
end

"""
    CollisionPrimitive

Specification for a collision geometry primitive to add to a body.
Used to add simple collision shapes (boxes, capsules) for more reliable
collision detection than complex mesh geometry.

# Fields
- `name::String`: Unique geom name
- `body::String`: Name of the body to attach to
- `geom_type::Symbol`: Type of geometry (:box, :capsule, :sphere, :cylinder)
- `pos::Vector{Float64}`: Position relative to body frame [x, y, z]
- `size::Vector{Float64}`: Size parameters (meaning depends on geom_type):
    - :box - [half_x, half_y, half_z]
    - :capsule - [radius, half_length]
    - :sphere - [radius]
    - :cylinder - [radius, half_length]
- `quat::Vector{Float64}`: Orientation quaternion [w, x, y, z]
- `rgba::Vector{Float64}`: Color [r, g, b, a] - use alpha < 1 for debugging visibility
- `friction::Vector{Float64}`: Friction coefficients [sliding, torsional, rolling]
- `solref::Vector{Float64}`: Contact solver reference [timeconst, dampratio] for contact stiffness
- `solimp::Vector{Float64}`: Contact solver impedance [dmin, dmax, width] for penetration control
- `condim::Int`: Contact dimensionality (1=frictionless, 3=pyramidal, 4=pyramidal+torsion, 6=elliptic)
"""
Base.@kwdef struct CollisionPrimitive
    name::String
    body::String
    geom_type::Symbol = :box
    pos::Vector{Float64} = [0.0, 0.0, 0.0]
    size::Vector{Float64} = [0.01, 0.01, 0.01]
    quat::Vector{Float64} = [1.0, 0.0, 0.0, 0.0]
    rgba::Vector{Float64} = [1.0, 0.0, 0.0, 0.3]  # Semi-transparent red for debugging
    friction::Vector{Float64} = [1.0, 0.005, 0.0001]
    solref::Vector{Float64} = [0.002, 1.0]        # Stiff, critically damped contact
    solimp::Vector{Float64} = [0.95, 0.99, 0.001] # Hard contact with minimal penetration
    condim::Int = 4                                # Pyramidal friction with torsion
end

"""
    resolve_includes(xml::String, base_dir::String) -> String

Recursively resolve <include file="..."/> directives in MuJoCo XML.
Inlines the content of included files into the main XML.
"""
function resolve_includes(xml::String, base_dir::String)
    result = xml

    # Match <include file="filename" /> or <include file="filename"/>
    include_pattern = r"<include\s+file=\"([^\"]+)\"\s*/>"

    while occursin(include_pattern, result)
        match_obj = match(include_pattern, result)
        include_path = joinpath(base_dir, match_obj.captures[1])

        if !isfile(include_path)
            error("Include file not found: $include_path")
        end

        # Read the included file (strip XML declaration and mujoco wrapper if present)
        included_content = read(include_path, String)

        # Remove XML declaration if present
        included_content = replace(included_content, r"<\?xml[^>]*\?>\s*" => "")

        # Extract content between <mujoco...> and </mujoco> if present
        mujoco_match = match(r"<mujoco[^>]*>(.*)</mujoco>"s, included_content)
        if mujoco_match !== nothing
            included_content = mujoco_match.captures[1]
        end

        # Replace the include directive with the file content
        result = replace(result, match_obj.match => included_content; count = 1)
    end

    return result
end

"""
    generate_camera_xml(camera::BodyCamera) -> String

Generate MuJoCo XML string for a body-attached camera.
"""
function generate_camera_xml(camera::BodyCamera)
    pos_str = join(camera.pos, " ")
    quat_str = join(camera.quat, " ")

    return """<camera name="$(camera.name)" pos="$(pos_str)" quat="$(quat_str)" fovy="$(camera.fovy)"/>"""
end

"""
    inject_camera_into_body(xml::String, camera::BodyCamera) -> String

Inject a camera element into its parent body in the XML.
Finds the body by name and inserts the camera element before the closing </body> tag.
"""
function inject_camera_into_body(xml::String, camera::BodyCamera)
    # Find the body element with matching name
    # Pattern: <body name="gripper" ... > ... </body>
    # We need to find the correct closing </body> tag for this specific body

    # First, find the opening tag of the target body
    body_pattern = Regex("<body\\s+name=\"$(camera.body)\"[^>]*>")
    body_match = match(body_pattern, xml)

    if body_match === nothing
        error("Body not found in XML: $(camera.body)")
    end

    # Start searching from after the opening tag
    start_pos = body_match.offset + length(body_match.match)

    # Count nested <body> tags to find the correct closing </body>
    depth = 1
    pos = start_pos
    xml_bytes = codeunits(xml)

    while depth > 0 && pos <= length(xml)
        # Look for next <body or </body>
        remaining = xml[pos:end]

        open_match = match(r"<body\s", remaining)
        close_match = match(r"</body>", remaining)

        if close_match === nothing
            error("Malformed XML: missing closing </body> tag for body '$(camera.body)'")
        end

        # Determine which comes first
        open_pos = open_match === nothing ? typemax(Int) : open_match.offset
        close_pos = close_match.offset

        if open_pos < close_pos
            # Found a nested <body> opening tag
            depth += 1
            pos = pos + open_pos + 5  # Move past "<body"
        else
            # Found a </body> closing tag
            depth -= 1
            if depth == 0
                # This is our target closing tag
                insert_pos = pos + close_pos - 1
                camera_xml = "\n        " * generate_camera_xml(camera)
                return xml[1:(insert_pos - 1)] * camera_xml * xml[insert_pos:end]
            else
                pos = pos + close_pos + 6  # Move past "</body>"
            end
        end
    end

    error("Could not find closing </body> tag for body '$(camera.body)'")
end

"""
    inject_cameras(xml::String, cameras::Vector{BodyCamera}) -> String

Inject multiple body-attached cameras into the XML.
"""
function inject_cameras(xml::String, cameras::Vector{BodyCamera})
    result = xml
    for camera in cameras
        result = inject_camera_into_body(result, camera)
    end
    return result
end

"""
    generate_collision_geom_xml(prim::CollisionPrimitive) -> String

Generate MuJoCo XML string for a collision primitive.
Includes friction, contact solver parameters, and condim for robust grasping.
"""
function generate_collision_geom_xml(prim::CollisionPrimitive)
    pos_str = join(prim.pos, " ")
    size_str = join(prim.size, " ")
    quat_str = join(prim.quat, " ")
    rgba_str = join(prim.rgba, " ")
    friction_str = join(prim.friction, " ")
    solref_str = join(prim.solref, " ")
    solimp_str = join(prim.solimp, " ")

    return """<geom name="$(prim.name)" type="$(prim.geom_type)" pos="$(pos_str)" quat="$(quat_str)" size="$(size_str)" rgba="$(rgba_str)" contype="1" conaffinity="1" condim="$(prim.condim)" friction="$(friction_str)" solref="$(solref_str)" solimp="$(solimp_str)" group="3"/>"""
end

"""
    inject_collision_into_body(xml::String, prim::CollisionPrimitive) -> String

Inject a collision primitive geom into its parent body in the XML.
Uses the same body-finding logic as camera injection.
"""
function inject_collision_into_body(xml::String, prim::CollisionPrimitive)
    # Find the body element with matching name
    body_pattern = Regex("<body\\s+name=\"$(prim.body)\"[^>]*>")
    body_match = match(body_pattern, xml)

    if body_match === nothing
        error("Body not found in XML: $(prim.body)")
    end

    # Start searching from after the opening tag
    start_pos = body_match.offset + length(body_match.match)

    # Count nested <body> tags to find the correct closing </body>
    depth = 1
    pos = start_pos

    while depth > 0 && pos <= length(xml)
        remaining = xml[pos:end]

        open_match = match(r"<body\s", remaining)
        close_match = match(r"</body>", remaining)

        if close_match === nothing
            error("Malformed XML: missing closing </body> tag for body '$(prim.body)'")
        end

        open_pos = open_match === nothing ? typemax(Int) : open_match.offset
        close_pos = close_match.offset

        if open_pos < close_pos
            depth += 1
            pos = pos + open_pos + 5
        else
            depth -= 1
            if depth == 0
                insert_pos = pos + close_pos - 1
                geom_xml = "\n        " * generate_collision_geom_xml(prim)
                return xml[1:(insert_pos - 1)] * geom_xml * xml[insert_pos:end]
            else
                pos = pos + close_pos + 6
            end
        end
    end

    error("Could not find closing </body> tag for body '$(prim.body)'")
end

"""
    inject_collision_primitives(xml::String, primitives::Vector{CollisionPrimitive}) -> String

Inject multiple collision primitives into the XML.
"""
function inject_collision_primitives(xml::String, primitives::Vector{CollisionPrimitive})
    result = xml
    for prim in primitives
        result = inject_collision_into_body(result, prim)
    end
    return result
end

"""
    remove_keyframes(xml::String) -> String

Remove all <keyframe>...</keyframe> sections from the XML.
This is necessary when adding free joints (cubes) because keyframes have fixed qpos sizes
that become invalid when the joint configuration changes.
"""
function remove_keyframes(xml::String)
    # Remove keyframe section (single or multi-line)
    # Pattern matches <keyframe> ... </keyframe> with any content between
    return replace(xml, r"<keyframe[^>]*>.*?</keyframe>"s => "")
end

"""
    default_gripper_collisions() -> Vector{CollisionPrimitive}

Returns default collision primitives for the SO101 gripper.
These add box colliders to the static and moving jaw for reliable grasping.

The gripper has:
- Static jaw: Part of the `gripper` body (wrist_roll_follower mesh)
- Moving jaw: The `moving_jaw_so101_v1` body

Collision boxes are positioned to approximate the finger geometry.
Use semi-transparent colors initially for debugging, then set alpha=0 for production.

Contact parameters are tuned for secure grasping:
- High friction (sliding=2.0, torsional=0.1, rolling=0.001)
- Stiff contact response (solref=[0.002, 1.0])
- Hard contact with minimal penetration (solimp=[0.95, 0.99, 0.001])
- Pyramidal friction with torsion (condim=4)
"""
function default_gripper_collisions()
    # Shared contact parameters for VERY secure grasping
    # Extremely high friction - almost like glue
    grip_friction = [10.0, 1.0, 0.1]        # Extremely high friction
    grip_solref = [0.0005, 1.0]             # Extremely stiff contact
    grip_solimp = [0.99, 0.9999, 0.00001]   # Extremely hard contact, almost no penetration
    grip_condim = 6                          # Elliptic friction cone (best friction model)

    return [
        # Static jaw (fixed finger) - attached to gripper body
        # The static jaw extends in the -Z direction from the gripper body
        CollisionPrimitive(
            name = "static_finger_col",
            body = "gripper",
            geom_type = :box,
            pos = [-0.022, 0.0, -0.07],     # Position of static finger
            size = [0.008, 0.012, 0.035],   # Half-extents: thin in X, wider in Y, long in Z
            quat = [1.0, 0.0, 0.0, 0.0],
            rgba = [1.0, 0.3, 0.3, 0.5],    # Semi-transparent red for debugging
            friction = grip_friction,
            solref = grip_solref,
            solimp = grip_solimp,
            condim = grip_condim
        ),
        # Moving jaw (moving finger) - attached to moving_jaw_so101_v1 body
        # The moving jaw rotates around the gripper joint
        CollisionPrimitive(
            name = "moving_finger_col",
            body = "moving_jaw_so101_v1",
            geom_type = :box,
            pos = [0.0, -0.035, 0.018],     # Position of moving finger tip area
            size = [0.008, 0.025, 0.012],   # Half-extents
            quat = [1.0, 0.0, 0.0, 0.0],
            rgba = [0.3, 1.0, 0.3, 0.5],    # Semi-transparent green for debugging
            friction = grip_friction,
            solref = grip_solref,
            solimp = grip_solimp,
            condim = grip_condim
        )
    ]
end

"""
    generate_cube_xml(cubes::Vector{CubeSpec}) -> String

Generate MuJoCo XML string for a list of cubes.
Each cube gets a free joint (6 DOF), proper inertia, and contact settings for grasping.

Contact parameters are tuned for secure grasping:
- Very high friction for grip
- Very stiff contact response (solref)
- condim=6 for elliptic friction cone (best friction model)
"""
function generate_cube_xml(cubes::Vector{CubeSpec})
    io = IOBuffer()

    for cube in cubes
        # Calculate diagonal inertia for a solid cube: I = (1/6) * m * side^2
        # where side = 2 * half-extent
        side = 2 * cube.size
        inertia = (1.0 / 6.0) * cube.mass * side^2

        # Format color as string
        color_str = join(cube.color, " ")

        # Contact parameters tuned for EXTREMELY secure grasping
        # friction: [sliding, torsional, rolling] - very high values
        # solref: [timeconst, dampratio] - extremely stiff contact
        # solimp: [dmin, dmax, width] - extremely hard contact
        # condim: 6 = elliptic friction cone (best friction model)
        print(io,
            """
    <body name="$(cube.name)" pos="$(cube.pos[1]) $(cube.pos[2]) $(cube.pos[3])">
        <joint name="$(cube.name)_joint" type="free" frictionloss="0.01"/>
        <inertial pos="0 0 0" mass="$(cube.mass)" diaginertia="$(inertia) $(inertia) $(inertia)"/>
        <geom name="$(cube.name)_geom" type="box" size="$(cube.size) $(cube.size) $(cube.size)"
              rgba="$(color_str)"
              friction="$(cube.friction) 1.0 0.1"
              solref="0.0005 1" solimp="0.99 0.9999 0.00001"
              condim="6"/>
    </body>
""")
    end

    return String(take!(io))
end

"""
    build_scene(base_xml_path::String, cubes::Vector{CubeSpec}; 
                cameras::Vector{BodyCamera}=BodyCamera[],
                collisions::Vector{CollisionPrimitive}=CollisionPrimitive[]) -> (model, data)

Load a MuJoCo scene with programmatically added cubes, body-attached cameras,
and collision primitives.

Creates a temporary XML file in the same directory as the base scene.
Handles <include> directives by inlining them before injection.
The temp file is kept for debugging purposes.

# Arguments
- `base_xml_path`: Path to the original scene.xml file
- `cubes`: Vector of CubeSpec defining cubes to add
- `cameras`: Vector of BodyCamera defining cameras attached to bodies
- `collisions`: Vector of CollisionPrimitive for additional collision geometry

# Returns
- `model`: MuJoCo Model with cubes, cameras, and collisions added
- `data`: MuJoCo Data initialized for the model
"""
function build_scene(base_xml_path::String, cubes::Vector{CubeSpec};
        cameras::Vector{BodyCamera} = BodyCamera[],
        collisions::Vector{CollisionPrimitive} = CollisionPrimitive[])
    # Read original XML
    if !isfile(base_xml_path)
        error("Scene file not found: $base_xml_path")
    end
    original_xml = read(base_xml_path, String)
    base_dir = dirname(base_xml_path)

    # Resolve includes if we have cameras or collisions to inject (need access to all bodies)
    if !isempty(cameras) || !isempty(collisions)
        modified_xml = resolve_includes(original_xml, base_dir)
    else
        modified_xml = original_xml
    end

    # Inject cameras into their parent bodies
    modified_xml = inject_cameras(modified_xml, cameras)

    # Inject collision primitives into their parent bodies
    modified_xml = inject_collision_primitives(modified_xml, collisions)

    # Generate cube XML
    cube_xml = generate_cube_xml(cubes)

    # Inject cubes before </worldbody>
    # Note: there may be multiple </worldbody> tags after inlining includes
    # We want the LAST one (the main scene's worldbody)
    if !occursin("</worldbody>", modified_xml)
        error("Could not find </worldbody> tag in scene XML")
    end

    # Find the last </worldbody> and inject cubes there
    last_worldbody = findlast("</worldbody>", modified_xml)
    if last_worldbody !== nothing
        insert_pos = first(last_worldbody)
        modified_xml = modified_xml[1:(insert_pos - 1)] * cube_xml * "    </worldbody>" *
                       modified_xml[(last(last_worldbody) + 1):end]
    end

    # Remove keyframes if we added any free joints (cubes)
    # Keyframes have fixed qpos sizes that become invalid when joint configuration changes
    if !isempty(cubes)
        modified_xml = remove_keyframes(modified_xml)
    end

    # Write to temp file in same directory (so mesh paths resolve)
    temp_path = joinpath(base_dir, ".scene_generated.xml")
    write(temp_path, modified_xml)

    # Load model
    model = load_model(temp_path)
    data = init_data(model)

    num_cams = length(cameras)
    num_cols = length(collisions)
    cam_str = num_cams > 0 ? ", $(num_cams) body cameras" : ""
    col_str = num_cols > 0 ? ", $(num_cols) collision primitives" : ""
    println("Scene built: $(model.nbody) bodies, $(model.njnt) joints, $(length(cubes)) cubes$(cam_str)$(col_str)")

    return model, data
end

"""
    build_scene(base_xml_path::String; 
                cameras::Vector{BodyCamera}=BodyCamera[],
                collisions::Vector{CollisionPrimitive}=CollisionPrimitive[]) -> (model, data)

Load a MuJoCo scene with optional body-attached cameras and collision primitives
but no additional objects.
"""
function build_scene(base_xml_path::String;
        cameras::Vector{BodyCamera} = BodyCamera[],
        collisions::Vector{CollisionPrimitive} = CollisionPrimitive[])
    if isempty(cameras) && isempty(collisions)
        return load_model(base_xml_path), init_data(load_model(base_xml_path))
    else
        return build_scene(
            base_xml_path, CubeSpec[], cameras = cameras, collisions = collisions)
    end
end

"""
    set_cube_position!(model, data, cube_name::String, pos::Vector{Float64}; 
                       quat::Vector{Float64}=[1.0, 0.0, 0.0, 0.0])

Move a cube to a new position and orientation at runtime.

# Arguments
- `model`: MuJoCo Model
- `data`: MuJoCo Data
- `cube_name`: Name of the cube (as specified in CubeSpec)
- `pos`: New [x, y, z] position
- `quat`: New orientation as quaternion [w, x, y, z] (default: identity)

# Example
```julia
set_cube_position!(model, data, "red_cube", [0.2, 0.1, 0.05])
```
"""
function set_cube_position!(model, data, cube_name::String, pos::Vector{Float64};
        quat::Vector{Float64} = [1.0, 0.0, 0.0, 0.0])
    joint_name = "$(cube_name)_joint"
    joint_id = mj_name2id(model, Int32(LibMuJoCo.mjOBJ_JOINT), joint_name)

    if joint_id < 0
        error("Joint not found: $joint_name")
    end

    # Get qpos address (0-indexed in MuJoCo, but Julia arrays are 1-indexed)
    addr = model.jnt_qposadr[joint_id + 1] + 1

    # Set position (x, y, z)
    data.qpos[addr:(addr + 2)] .= pos

    # Set quaternion (w, x, y, z)
    data.qpos[(addr + 3):(addr + 6)] .= quat

    # Also zero out velocities
    vel_addr = model.jnt_dofadr[joint_id + 1] + 1
    data.qvel[vel_addr:(vel_addr + 5)] .= 0.0

    # Update forward kinematics
    mj_forward(model, data)
end

"""
    get_cube_position(model, data, cube_name::String) -> (pos, quat)

Get the current position and orientation of a cube.

# Arguments
- `model`: MuJoCo Model
- `data`: MuJoCo Data
- `cube_name`: Name of the cube (as specified in CubeSpec)

# Returns
- `pos`: Current [x, y, z] position
- `quat`: Current orientation as quaternion [w, x, y, z]
"""
function get_cube_position(model, data, cube_name::String)
    joint_name = "$(cube_name)_joint"
    joint_id = mj_name2id(model, Int32(LibMuJoCo.mjOBJ_JOINT), joint_name)

    if joint_id < 0
        error("Joint not found: $joint_name")
    end

    # Get qpos address
    addr = model.jnt_qposadr[joint_id + 1] + 1

    pos = Vector{Float64}(data.qpos[addr:(addr + 2)])
    quat = Vector{Float64}(data.qpos[(addr + 3):(addr + 6)])

    return pos, quat
end

"""
    list_cubes(model) -> Vector{String}

List all cube names in the model (bodies with names ending in _joint that are free joints).
"""
function list_cubes(model)
    cube_names = String[]

    for i in 0:(model.njnt - 1)
        joint_name = unsafe_string(mj_id2name(model, Int32(LibMuJoCo.mjOBJ_JOINT), i))

        # Check if it's a free joint (type 0) and follows our naming convention
        if model.jnt_type[i + 1] == 0 && endswith(joint_name, "_joint")
            cube_name = replace(joint_name, "_joint" => "")
            push!(cube_names, cube_name)
        end
    end

    return cube_names
end

"""
    generate_cubes(n::Int; radius_min=0.10, radius_max=0.38, z=0.025) -> Vector{CubeSpec}

Generate `n` cubes distributed in a semicircle in front of the robot.

Cubes are placed in the front half of a circle (angle range -90° to +90°),
with radius biased toward the middle of the range for better reachability.

# Arguments
- `n`: Number of cubes to generate
- `radius_min`: Minimum distance from origin (default 0.10m for SO101)
- `radius_max`: Maximum distance from origin (default 0.38m for SO101)
- `z`: Height of cube centers (default 0.025m, on table surface)

# Returns
- Vector of CubeSpec with random positions and vibrant colors

# Examples
```julia
# SO101 robot (shorter reach)
cubes = generate_cubes(50, radius_min=0.10, radius_max=0.38)

# Trossen robot (longer reach)
cubes = generate_cubes(50, radius_min=0.12, radius_max=0.55)
```
"""
function generate_cubes(n::Int; radius_min::Float64 = 0.10, radius_max::Float64 = 0.38,
        z::Float64 = 0.025)
    cubes = CubeSpec[]
    for i in 1:n
        # Distribute cubes in a semicircle in FRONT of the robot
        # Angle range: -90° to +90° (front half only)
        # 0° = directly in front (+X), ±90° = sides (±Y)
        angle = (rand() - 0.5) * π  # Random angle between -π/2 and +π/2

        # Bias radius toward the middle of the range (better reachability)
        radius = radius_min + (radius_max - radius_min) * (0.3 + 0.7 * rand())

        x = radius * cos(angle)
        y = radius * sin(angle)

        # Random color (keep it vibrant)
        color = [0.3 + 0.7 * rand(), 0.3 + 0.7 * rand(), 0.3 + 0.7 * rand(), 1.0]

        push!(cubes, CubeSpec(
            name = "cube_$i",
            pos = [x, y, z],
            color = color
        ))
    end
    return cubes
end
