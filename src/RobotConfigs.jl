# RobotConfigs.jl - Configuration structures and factory functions for robot types
#
# This module extracts robot-specific setup logic from individual simulation files
# into reusable configuration objects for a unified multi-robot simulation server.
#
# Usage:
#   include("src/RobotConfigs.jl")
#   config = get_robot_config("so101", project_root)
#   # Use config.mjcf_path, config.cameras, etc. to set up simulation

# Requires RobotTypes to be loaded first for robot type definitions
# include("RobotTypes.jl")

# =============================================================================
# Camera Specification Helper
# =============================================================================

"""
    CameraConfigSpec

Lightweight camera specification for robot configurations.
Unlike CameraSpec from capture/types.jl, this doesn't include output backend
since that's determined at runtime by the simulation server.

# Fields
- `name::String`: Camera identifier (e.g., "front", "gripper")
- `lookat::Vector{Float64}`: 3D point the camera looks at
- `distance::Float64`: Distance from lookat point
- `azimuth::Float64`: Horizontal angle in degrees
- `elevation::Float64`: Vertical angle in degrees
- `orbiting::Bool`: If true, azimuth changes over time
- `orbit_speed::Float64`: Rotation speed in degrees per second
- `mode::Symbol`: Camera mode (:free or :fixed)
- `model_camera::Union{String, Nothing}`: Name of model-defined camera for :fixed mode
"""
Base.@kwdef struct CameraConfigSpec
    name::String
    lookat::Vector{Float64} = [0.0, 0.0, 0.2]
    distance::Float64 = 1.2
    azimuth::Float64 = 180.0
    elevation::Float64 = -20.0
    orbiting::Bool = false
    orbit_speed::Float64 = 0.0
    mode::Symbol = :free
    model_camera::Union{String, Nothing} = nothing
end

"""
    camera_spec(; name, lookat=[0,0,0.2], distance=1.2, azimuth=180.0,
                  elevation=-20.0, orbiting=false, orbit_speed=30.0,
                  mode=:free, model_camera=nothing) -> CameraConfigSpec

Create a camera specification with sensible defaults.

# Arguments
- `name::String`: Required. Camera identifier.
- `lookat::Vector`: 3D point the camera looks at (default: [0, 0, 0.2])
- `distance::Float64`: Distance from lookat point (default: 1.2)
- `azimuth::Float64`: Horizontal angle in degrees (default: 180.0, front view)
- `elevation::Float64`: Vertical angle in degrees (default: -20.0)
- `orbiting::Bool`: Enable orbital motion (default: false)
- `orbit_speed::Float64`: Rotation speed in deg/s when orbiting (default: 30.0)
- `mode::Symbol`: :free for external camera, :fixed for body-mounted (default: :free)
- `model_camera::Union{String, Nothing}`: Model camera name for :fixed mode

# Examples
```julia
# External front camera
cam = camera_spec(name="front", lookat=[0, 0, 0.2], distance=1.2, azimuth=180.0)

# Orbiting camera
cam = camera_spec(name="orbit", orbiting=true, orbit_speed=30.0)

# Body-mounted gripper camera
cam = camera_spec(name="gripper", mode=:fixed, model_camera="gripper_cam")
```
"""
function camera_spec(;
        name::String,
        lookat::Vector{<:Real} = [0.0, 0.0, 0.2],
        distance::Real = 1.2,
        azimuth::Real = 180.0,
        elevation::Real = -20.0,
        orbiting::Bool = false,
        orbit_speed::Real = 30.0,
        mode::Symbol = :free,
        model_camera::Union{String, Nothing} = nothing)
    return CameraConfigSpec(
        name = name,
        lookat = Float64.(lookat),
        distance = Float64(distance),
        azimuth = Float64(azimuth),
        elevation = Float64(elevation),
        orbiting = orbiting,
        orbit_speed = Float64(orbit_speed),
        mode = mode,
        model_camera = model_camera
    )
end

# =============================================================================
# Body Camera Specification
# =============================================================================

"""
    BodyCameraConfigSpec

Specification for a camera attached to a robot body (moves with the body).
These cameras are injected into the MuJoCo XML at scene build time.

# Fields
- `name::String`: Camera name (used to reference in capture config)
- `body::String`: Name of the body to attach to (e.g., "gripper", "link_6")
- `pos::Vector{Float64}`: Position relative to body frame [x, y, z]
- `quat::Vector{Float64}`: Orientation quaternion [w, x, y, z]
- `fovy::Float64`: Vertical field of view in degrees
"""
Base.@kwdef struct BodyCameraConfigSpec
    name::String
    body::String
    pos::Vector{Float64} = [0.0, 0.0, 0.0]
    quat::Vector{Float64} = [1.0, 0.0, 0.0, 0.0]
    fovy::Float64 = 60.0
end

# =============================================================================
# Robot Configuration
# =============================================================================

"""
    RobotConfig

Complete configuration for a robot type, encapsulating all setup logic needed
to initialize a simulation.

# Fields
- `robot_id::String`: Unique identifier (e.g., "so101", "lekiwi", "trossen/wxai")
- `display_name::String`: Human-readable name (e.g., "SO101 Robot Arm")
- `mjcf_path::String`: Path to MuJoCo scene.xml (relative to project root)
- `urdf_path::Union{String, Nothing}`: Path to URDF if available
- `assets_dir::String`: Directory containing meshes (relative to project root)
- `follower_type::Type`: Robot type for teleoperation (SO101, TrossenWXAI, etc.)
- `default_leader_type::Type`: Default leader type for mapping (usually SO101)
- `cube_config::NamedTuple`: Cube generation parameters (radius_min, radius_max, size, z)
- `cameras::Vector{CameraConfigSpec}`: External camera specifications
- `body_cameras::Vector{BodyCameraConfigSpec}`: Body-mounted camera specs
- `gripper_collisions::Bool`: Whether to add gripper collision boxes
- `has_mobile_base::Bool`: Whether robot has a mobile base (e.g., LeKiwi)
- `extra_setup::Union{Function, Nothing}`: Additional setup function called after scene build
"""
struct RobotConfig
    robot_id::String
    display_name::String
    mjcf_path::String
    urdf_path::Union{String, Nothing}
    assets_dir::String
    follower_type::Type
    default_leader_type::Type
    cube_config::NamedTuple{(:radius_min, :radius_max, :size, :z), NTuple{4, Float64}}
    cameras::Vector{CameraConfigSpec}
    body_cameras::Vector{BodyCameraConfigSpec}
    gripper_collisions::Bool
    has_mobile_base::Bool
    extra_setup::Union{Function, Nothing}
end

# =============================================================================
# SO101 Configuration
# =============================================================================

"""
    so101_config(project_root::String) -> RobotConfig

Create configuration for the SO-ARM100 (SO101) robot arm.

The SO101 is a 6-DOF desktop robot arm with:
- 5 revolute joints + revolute gripper
- ~0.38m reach
- Gripper-mounted camera
- Simple box collision primitives for grasping
"""
function so101_config(project_root::String)
    return RobotConfig(
        "so101",
        "SO101 Robot Arm",
        joinpath("robots", "SO-ARM100", "Simulation", "SO101", "scene.xml"),
        joinpath("robots", "SO-ARM100", "Simulation", "SO101", "so101_new_calib.urdf"),
        joinpath("robots", "SO-ARM100", "Simulation", "SO101"),
        SO101,
        SO101,
        (radius_min = 0.10, radius_max = 0.38, size = 0.015, z = 0.025),
        [
            camera_spec(
                name = "front",
                lookat = [0.0, 0.0, 0.2],
                distance = 1.2,
                azimuth = 180.0,
                elevation = -20.0
            ),
            camera_spec(
                name = "side",
                lookat = [0.0, 0.0, 0.2],
                distance = 1.2,
                azimuth = 90.0,
                elevation = -20.0
            ),
            camera_spec(
                name = "orbit",
                lookat = [0.0, 0.0, 0.2],
                distance = 1.5,
                azimuth = 0.0,
                elevation = -30.0,
                orbiting = true,
                orbit_speed = 30.0
            ),
            camera_spec(
                name = "gripper",
                mode = :fixed,
                model_camera = "gripper_cam"
            )
        ],
        [
            BodyCameraConfigSpec(
            name = "gripper_cam",
            body = "gripper",
            pos = [0.0, 0.0, -0.04],
            quat = [1.0, 0.0, 0.0, 0.0],
            fovy = 90.0
        )
        ],
        true,   # gripper_collisions
        false,  # has_mobile_base
        nothing # extra_setup
    )
end

# =============================================================================
# LeKiwi Configuration
# =============================================================================

"""
    lekiwi_config(project_root::String) -> RobotConfig

Create configuration for the LeKiwi mobile manipulator.

The LeKiwi is a mobile robot with:
- 3-wheel omnidirectional base
- SO101-based 6-DOF arm
- Built-in front and wrist cameras in the model
- ~0.35m arm reach
"""
function lekiwi_config(project_root::String)
    return RobotConfig(
        "lekiwi",
        "LeKiwi Mobile Manipulator",
        joinpath("examples", "lekiwi", "scene.xml"),
        nothing,  # No standalone URDF
        joinpath("examples", "lekiwi"),
        LeKiwiArm,
        SO101,
        (radius_min = 0.15, radius_max = 0.40, size = 0.015, z = 0.08),
        [
            # Front and wrist are model cameras (already in XML)
            camera_spec(
                name = "front",
                mode = :fixed,
                model_camera = "front"
            ),
            camera_spec(
                name = "wrist",
                mode = :fixed,
                model_camera = "wrist"
            ),
            # External side cameras
            camera_spec(
                name = "side_left",
                lookat = [0.0, 0.0, 0.15],
                distance = 1.0,
                azimuth = 90.0,
                elevation = -20.0
            ),
            camera_spec(
                name = "side_right",
                lookat = [0.0, 0.0, 0.15],
                distance = 1.0,
                azimuth = -90.0,
                elevation = -20.0
            )
        ],
        BodyCameraConfigSpec[],  # Cameras already in model
        false,  # gripper_collisions (not needed for LeKiwi)
        true,   # has_mobile_base
        nothing # extra_setup
    )
end

# =============================================================================
# Trossen WXAI Configuration
# =============================================================================

"""
    trossen_config(project_root::String) -> RobotConfig

Create configuration for the Trossen Robotics WXAI arm.

The Trossen WXAI is a 6-DOF robot arm with:
- 6 revolute joints + prismatic (slide) gripper
- ~0.55m reach
- Gripper-mounted camera on link_6
"""
function trossen_config(project_root::String)
    return RobotConfig(
        "trossen/wxai",
        "Trossen WXAI Robot Arm",
        joinpath("robots", "trossen_arm_mujoco", "trossen_arm_mujoco", "assets", "wxai",
            "scene.xml"),
        nothing,  # No URDF provided
        joinpath("robots", "trossen_arm_mujoco", "trossen_arm_mujoco", "assets", "wxai"),
        TrossenWXAI,
        SO101,
        (radius_min = 0.12, radius_max = 0.55, size = 0.015, z = 0.025),
        [
            camera_spec(
                name = "front",
                lookat = [0.0, 0.0, 0.25],
                distance = 1.4,
                azimuth = 180.0,
                elevation = -20.0
            ),
            camera_spec(
                name = "side",
                lookat = [0.0, 0.0, 0.25],
                distance = 1.4,
                azimuth = 90.0,
                elevation = -20.0
            ),
            camera_spec(
                name = "orbit",
                lookat = [0.0, 0.0, 0.25],
                distance = 1.8,
                azimuth = 0.0,
                elevation = -30.0,
                orbiting = true,
                orbit_speed = 30.0
            ),
            camera_spec(
                name = "gripper",
                mode = :fixed,
                model_camera = "gripper_cam"
            )
        ],
        [
            BodyCameraConfigSpec(
            name = "gripper_cam",
            body = "link_6",
            pos = [0.05, 0.0, 0.0],
            quat = [0.5, 0.5, -0.5, -0.5],
            fovy = 90.0
        )
        ],
        false,  # gripper_collisions (Trossen model has collision boxes)
        false,  # has_mobile_base
        nothing # extra_setup
    )
end

# =============================================================================
# Franka Panda Configuration
# =============================================================================

"""
    franka_config(project_root::String) -> RobotConfig

Create configuration for the Franka Emika Panda robot arm.

The Franka Panda is a 7-DOF research robot with:
- 7 revolute joints + tendon-driven gripper (0-255 actuator units)
- ~0.855m reach
- High precision and force feedback capabilities
- Wrist and gripper-mounted cameras
"""
function franka_config(project_root::String)
    return RobotConfig(
        "franka",
        "Franka Panda Robot Arm",
        joinpath("robots", "google-deepmind", "franka_emika_panda", "scene.xml"),
        nothing,  # URDF available but not used
        joinpath("robots", "google-deepmind", "franka_emika_panda"),
        FrankaPanda,
        SO101,
        (radius_min = 0.30, radius_max = 0.70, size = 0.025, z = 0.03),
        [
            camera_spec(
                name = "front",
                lookat = [0.0, 0.0, 0.5],
                distance = 2.0,
                azimuth = 180.0,
                elevation = -20.0
            ),
            camera_spec(
                name = "side",
                lookat = [0.0, 0.0, 0.5],
                distance = 2.0,
                azimuth = 90.0,
                elevation = -20.0
            ),
            camera_spec(
                name = "orbit",
                lookat = [0.0, 0.0, 0.5],
                distance = 2.6,
                azimuth = 0.0,
                elevation = -30.0,
                orbiting = true,
                orbit_speed = 30.0
            ),
            camera_spec(
                name = "gripper",
                mode = :fixed,
                model_camera = "gripper_cam"
            ),
            camera_spec(
                name = "wrist",
                mode = :fixed,
                model_camera = "wrist_cam"
            )
        ],
        [
            BodyCameraConfigSpec(
                name = "wrist_cam",
                body = "link7",
                pos = [0.05, -0.05, 0.15],
                quat = [0.707, 0.0, 0.707, 0.0],
                fovy = 60.0
            ),
            BodyCameraConfigSpec(
                name = "gripper_cam",
                body = "hand",
                pos = [0.0, 0.0, 0.10],
                quat = [0.707, 0.0, 0.707, 0.0],
                fovy = 90.0
            )
        ],
        false,  # gripper_collisions
        false,  # has_mobile_base
        nothing # extra_setup
    )
end

# =============================================================================
# Fanuc Configuration
# =============================================================================

"""
    fanuc_config(project_root::String, variant::String="m10ia") -> RobotConfig

Create configuration for Fanuc industrial robot arms.

Fanuc robots are industrial 6-DOF arms with multiple variants:
- m10ia: Classic yellow industrial robot (default)
- crx10ial: CRX collaborative robot
- m900ib700: Large 12-DOF robot
- And many more variants

Cube placement and camera distances scale dynamically based on robot size.

# Arguments
- `project_root::AbstractString`: Path to project root
- `variant::AbstractString`: Robot variant name (default: "m10ia")
"""
function fanuc_config(project_root::AbstractString, variant::AbstractString = "m10ia")
    # Calculate scale factors based on robot variant
    robot_scale, cube_scale = _fanuc_scale_factors(variant)

    # Calculate camera distances and heights
    base_distance = 1.5 * robot_scale
    lookat_height = 0.4 * robot_scale

    return RobotConfig(
        "fanuc/$(variant)",
        "Fanuc $(uppercase(variant)) Industrial Robot",
        joinpath("robots", "fanuc_mujoco", variant, "scene.xml"),
        nothing,  # No URDF
        joinpath("robots", "fanuc_mujoco", variant),
        FanucArm,
        SO101,
        (
            radius_min = 0.15 * cube_scale,
            radius_max = 0.50 * cube_scale,
            size = 0.03 * cube_scale,
            z = 0.04 * cube_scale
        ),
        [
            camera_spec(
                name = "front",
                lookat = [0.0, 0.0, lookat_height],
                distance = base_distance,
                azimuth = 180.0,
                elevation = -20.0
            ),
            camera_spec(
                name = "side",
                lookat = [0.0, 0.0, lookat_height],
                distance = base_distance,
                azimuth = 90.0,
                elevation = -20.0
            ),
            camera_spec(
                name = "orbit",
                lookat = [0.0, 0.0, lookat_height],
                distance = base_distance * 1.3,
                azimuth = 0.0,
                elevation = -30.0,
                orbiting = true,
                orbit_speed = 30.0
            ),
            camera_spec(
                name = "gripper",
                mode = :fixed,
                model_camera = "gripper_cam"
            )
        ],
        [
            BodyCameraConfigSpec(
            name = "gripper_cam",
            body = "link_6",
            pos = [0.05, 0.0, 0.0],
            quat = [0.5, 0.5, -0.5, 0.5],
            fovy = 90.0
        )
        ],
        false,  # gripper_collisions
        false,  # has_mobile_base
        nothing # extra_setup
    )
end

"""
    _fanuc_scale_factors(variant::AbstractString) -> Tuple{Float64, Float64}

Calculate robot and cube scale factors based on Fanuc variant.
Returns (robot_scale, cube_scale) for camera distance and cube placement.
"""
function _fanuc_scale_factors(variant::AbstractString)
    robot_scale = if occursin("m900", variant)
        2.5  # Very large robot
    elseif occursin("m710", variant) || occursin("r2000", variant) ||
           occursin("r1000", variant)
        1.8  # Large robot
    elseif occursin("crx", variant) || occursin("lrmate", variant) ||
           occursin("cr7", variant)
        1.0  # Small/medium robot
    else
        1.3  # Default industrial size
    end

    cube_scale = if occursin("m900", variant)
        3.0  # Very large robot
    elseif occursin("m710", variant) || occursin("r2000", variant) ||
           occursin("r1000", variant)
        2.0  # Large robot
    elseif occursin("crx", variant) || occursin("lrmate", variant) ||
           occursin("cr7", variant)
        1.0  # Small/medium robot
    else
        1.5  # Default industrial size
    end

    return robot_scale, cube_scale
end

# =============================================================================
# Robot Registry
# =============================================================================

"""
    ROBOT_CONFIGS

Registry mapping robot identifiers to their configuration factory functions.
Each factory function takes `project_root::String` and returns a `RobotConfig`.

# Supported Robots
- `"so101"` - SO-ARM100 desktop robot arm
- `"lekiwi"` - LeKiwi mobile manipulator
- `"trossen/wxai"` - Trossen Robotics WXAI arm
- `"franka"` - Franka Emika Panda
- `"fanuc"` - Fanuc M-10iA (default variant)
- `"fanuc/m10ia"`, `"fanuc/crx10ial"`, etc. - Specific Fanuc variants
"""
const ROBOT_CONFIGS = Dict{String, Function}(
    "so101" => so101_config,
    "lekiwi" => lekiwi_config,
    "trossen/wxai" => trossen_config,
    "trossen" => trossen_config,  # Alias
    "franka" => franka_config,
    "fanuc" => (root) -> fanuc_config(root, "m10ia")  # Default variant
)

"""
    get_robot_config(robot_id::String, project_root::String) -> RobotConfig

Get the configuration for a robot by its identifier.

Supports exact matches from ROBOT_CONFIGS, as well as Fanuc variant patterns
like "fanuc/m10ia" or "fanuc/crx10ial".

# Arguments
- `robot_id::String`: Robot identifier (e.g., "so101", "fanuc/crx10ial")
- `project_root::String`: Path to project root directory

# Returns
- `RobotConfig`: Complete robot configuration

# Throws
- `ArgumentError`: If robot_id is not recognized

# Examples
```julia
config = get_robot_config("so101", "/path/to/project")
config = get_robot_config("fanuc/crx10ial", "/path/to/project")
```
"""
function get_robot_config(robot_id::AbstractString, project_root::AbstractString)
    # Direct lookup
    if haskey(ROBOT_CONFIGS, robot_id)
        return ROBOT_CONFIGS[robot_id](project_root)
    end

    # Handle fanuc variants (fanuc/variant)
    if startswith(robot_id, "fanuc/")
        variant = replace(robot_id, "fanuc/" => "")
        return fanuc_config(project_root, variant)
    end

    # Unknown robot
    available = sort(collect(keys(ROBOT_CONFIGS)))
    throw(ArgumentError(
        "Unknown robot_id: '$robot_id'. Available: $(join(available, ", ")). " *
        "For Fanuc variants, use 'fanuc/<variant>' (e.g., 'fanuc/crx10ial')."))
end

"""
    list_available_robots() -> Vector{String}

Return a sorted list of available robot identifiers.
"""
function list_available_robots()
    return sort(collect(keys(ROBOT_CONFIGS)))
end

# =============================================================================
# Utility Functions
# =============================================================================

"""
    get_full_mjcf_path(config::RobotConfig, project_root::String) -> String

Get the full absolute path to the MuJoCo XML file.
"""
function get_full_mjcf_path(config::RobotConfig, project_root::String)
    return joinpath(project_root, config.mjcf_path)
end

"""
    get_full_urdf_path(config::RobotConfig, project_root::String) -> Union{String, Nothing}

Get the full absolute path to the URDF file, or nothing if not available.
"""
function get_full_urdf_path(config::RobotConfig, project_root::String)
    if config.urdf_path === nothing
        return nothing
    end
    return joinpath(project_root, config.urdf_path)
end
