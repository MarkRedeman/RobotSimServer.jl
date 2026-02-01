# LeKiwiTraits.jl - Trait implementations for LeKiwi mobile manipulator arm
#
# The LeKiwi is a mobile manipulator with an SO-ARM100 compatible arm:
# - 3 omniwheels for mobile base (not covered here, only arm joints)
# - 6 revolute arm joints: Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw
#
# Joint limits are derived from the MuJoCo model: examples/lekiwi/scene.xml

"""
    joint_specs(::Type{LeKiwiArm}) -> Vector{JointSpec}

Return joint specifications for the LeKiwi arm (SO-ARM100 variant).

Joint limits in degrees (from MuJoCo model radians):
- Rotation: ±110° (±1.92 rad)
- Pitch: ±100° (±1.747 rad)
- Elbow: ±95° (±1.657 rad)
- Wrist_Pitch: ±95° (±1.66 rad)
- Wrist_Roll: ±160° (±2.79 rad)
- Jaw: 1.2° to 97° (0.0208 to 1.693 rad)
"""
function joint_specs(::Type{LeKiwiArm})
    return [
        JointSpec("Rotation", -110.0, 110.0, :revolute),
        JointSpec("Pitch", -100.0, 100.0, :revolute),
        JointSpec("Elbow", -95.0, 95.0, :revolute),
        JointSpec("Wrist_Pitch", -95.0, 95.0, :revolute),
        JointSpec("Wrist_Roll", -160.0, 160.0, :revolute),
        JointSpec("Jaw", 1.2, 97.0, :revolute)
    ]
end

"""
    joint_names(::Type{LeKiwiArm}) -> Vector{String}

Return ordered list of native joint names for LeKiwi arm.
These names match the actuator/joint names in the MuJoCo model.
"""
function joint_names(::Type{LeKiwiArm})
    return [
        "Rotation",
        "Pitch",
        "Elbow",
        "Wrist_Pitch",
        "Wrist_Roll",
        "Jaw"
    ]
end

"""
    ee_body_name(::Type{LeKiwiArm}) -> String

Return the end-effector body name for LeKiwi.
The Fixed_Jaw body is the end-effector reference point.
"""
ee_body_name(::Type{LeKiwiArm}) = "Fixed_Jaw"

"""
    model_path(::Type{LeKiwiArm}) -> String

Return the path to the LeKiwi MuJoCo model file.
Path is relative to the project root directory.
"""
function model_path(::Type{LeKiwiArm})
    return joinpath("examples", "lekiwi", "scene.xml")
end

# =============================================================================
# SO101 ↔ LeKiwi Joint Name Mapping
# =============================================================================

"""
    LEKIWI_JOINT_NAME_MAP

Bidirectional mapping between SO101-compatible joint names and LeKiwi native
joint names. This allows clients to use SO101 names with LeKiwi robots.

Maps in both directions:
- SO101 name → LeKiwi native name
- LeKiwi native name → LeKiwi native name (identity for direct use)
"""
const LEKIWI_JOINT_NAME_MAP = Dict{String, String}(
    # SO101-compatible names -> LeKiwi native names
    "shoulder_pan" => "Rotation",
    "shoulder_lift" => "Pitch",
    "elbow_flex" => "Elbow",
    "wrist_flex" => "Wrist_Pitch",
    "wrist_roll" => "Wrist_Roll",
    "gripper" => "Jaw",
    # LeKiwi native names -> themselves (for direct use)
    "Rotation" => "Rotation",
    "Pitch" => "Pitch",
    "Elbow" => "Elbow",
    "Wrist_Pitch" => "Wrist_Pitch",
    "Wrist_Roll" => "Wrist_Roll",
    "Jaw" => "Jaw"
)

"""
    LEKIWI_JOINT_NAME_REVERSE_MAP

Mapping from LeKiwi native joint names to SO101-compatible joint names.
Used for state broadcasting in SO101-compatible format.
"""
const LEKIWI_JOINT_NAME_REVERSE_MAP = Dict{String, String}(
    "Rotation" => "shoulder_pan",
    "Pitch" => "shoulder_lift",
    "Elbow" => "elbow_flex",
    "Wrist_Pitch" => "wrist_flex",
    "Wrist_Roll" => "wrist_roll",
    "Jaw" => "gripper"
)

"""
    so101_to_lekiwi_joint(so101_name::String) -> Union{String, Nothing}

Convert an SO101 joint name to the corresponding LeKiwi native joint name.
Returns `nothing` if the joint name is not recognized.

# Example
```julia
lekiwi_name = so101_to_lekiwi_joint("shoulder_pan")  # "Rotation"
```
"""
function so101_to_lekiwi_joint(so101_name::String)
    return get(LEKIWI_JOINT_NAME_MAP, so101_name, nothing)
end

"""
    lekiwi_to_so101_joint(lekiwi_name::String) -> Union{String, Nothing}

Convert a LeKiwi native joint name to the corresponding SO101 joint name.
Returns `nothing` if the joint name is not recognized.

# Example
```julia
so101_name = lekiwi_to_so101_joint("Rotation")  # "shoulder_pan"
```
"""
function lekiwi_to_so101_joint(lekiwi_name::String)
    return get(LEKIWI_JOINT_NAME_REVERSE_MAP, lekiwi_name, nothing)
end

"""
    gripper_range(::Type{LeKiwiArm}) -> Tuple{Float64, Float64}

Return the gripper (Jaw) range in degrees for LeKiwi.
Returns (closed_position, open_position).
"""
gripper_range(::Type{LeKiwiArm}) = (1.2, 97.0)
