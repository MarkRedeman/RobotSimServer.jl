# FrankaTraits.jl - Trait implementations for Franka Emika Panda robot arm
#
# The Franka Panda is a 7-DOF robot arm with:
# - 7 revolute arm joints (joint1 through joint7)
# - 2 prismatic finger joints (finger_joint1, finger_joint2) coupled via tendon
# - High precision and force feedback capabilities
#
# Joint limits are derived from the MuJoCo Menagerie model.

"""
    joint_specs(::Type{FrankaPanda}) -> Vector{JointSpec}

Return joint specifications for the Franka Panda robot arm.

Joint limits in degrees (from MuJoCo model):
- joint1: ±166° (base rotation)
- joint2: ±101° (shoulder)
- joint3: ±166° (upper arm rotation)
- joint4: -176° to -4° (elbow)
- joint5: ±166° (forearm rotation)
- joint6: -1° to 215° (wrist flex)
- joint7: ±166° (wrist rotation)
- finger_joint1/2: 0 to 0.04m (prismatic, coupled)
"""
function joint_specs(::Type{FrankaPanda})
    return [
        JointSpec("joint1", -166.0, 166.0, :revolute),
        JointSpec("joint2", -101.0, 101.0, :revolute),
        JointSpec("joint3", -166.0, 166.0, :revolute),
        JointSpec("joint4", -176.0, -4.0, :revolute),
        JointSpec("joint5", -166.0, 166.0, :revolute),
        JointSpec("joint6", -1.0, 215.0, :revolute),
        JointSpec("joint7", -166.0, 166.0, :revolute),
        JointSpec("finger_joint1", 0.0, 0.04, :prismatic),  # meters
        JointSpec("finger_joint2", 0.0, 0.04, :prismatic)   # meters (coupled)
    ]
end

"""
    joint_names(::Type{FrankaPanda}) -> Vector{String}

Return ordered list of joint names for Franka Panda.
These names match the joint names in the MuJoCo model.
"""
function joint_names(::Type{FrankaPanda})
    return [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint7",
        "finger_joint1",
        "finger_joint2"
    ]
end

"""
    arm_joint_names(::Type{FrankaPanda}) -> Vector{String}

Return only the arm joint names (excluding gripper fingers).
"""
function arm_joint_names(::Type{FrankaPanda})
    return ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
end

"""
    ee_body_name(::Type{FrankaPanda}) -> String

Return the end-effector body name for Franka Panda.
link7 is the wrist flange; use "hand" for the gripper base.
"""
ee_body_name(::Type{FrankaPanda}) = "link7"

"""
    model_path(::Type{FrankaPanda}) -> String

Return the path to the Franka Panda MuJoCo model file.
Path is relative to the project root directory.
"""
function model_path(::Type{FrankaPanda})
    return joinpath("robots", "google-deepmind", "franka_emika_panda", "scene.xml")
end

# =============================================================================
# Gripper Mapping (SO101 degrees ↔ Franka actuator units)
# =============================================================================

# Gripper parameters from Menagerie model
const FRANKA_GRIPPER_ACTUATOR_MIN = 0.0     # Fully closed (actuator units)
const FRANKA_GRIPPER_ACTUATOR_MAX = 255.0   # Fully open (actuator units)
const FRANKA_FINGER_MIN = 0.0               # Fully closed (meters)
const FRANKA_FINGER_MAX = 0.04              # Fully open (meters)
const SO101_GRIPPER_MIN = -10.0             # Fully closed (degrees)
const SO101_GRIPPER_MAX = 100.0             # Fully open (degrees)

"""
    map_gripper_so101_to_franka(so101_deg::Float64) -> Float64

Convert SO101 gripper position (degrees) to Franka actuator units (0-255).

# Mapping
- SO101 gripper: -10° (closed) to 100° (open)
- Franka actuator: 0 (closed) to 255 (open)
"""
function map_gripper_so101_to_franka(so101_deg::Float64)
    normalized = clamp(
        (so101_deg - SO101_GRIPPER_MIN) / (SO101_GRIPPER_MAX - SO101_GRIPPER_MIN),
        0.0, 1.0
    )
    return FRANKA_GRIPPER_ACTUATOR_MIN +
           normalized * (FRANKA_GRIPPER_ACTUATOR_MAX - FRANKA_GRIPPER_ACTUATOR_MIN)
end

"""
    map_gripper_franka_to_so101(franka_pos::Float64) -> Float64

Convert Franka finger position (meters) to SO101 gripper position (degrees).

# Mapping
- Franka finger position: 0m (closed) to 0.04m (open)
- SO101 gripper: -10° (closed) to 100° (open)
"""
function map_gripper_franka_to_so101(franka_pos::Float64)
    normalized = clamp(franka_pos / FRANKA_FINGER_MAX, 0.0, 1.0)
    return SO101_GRIPPER_MIN + normalized * (SO101_GRIPPER_MAX - SO101_GRIPPER_MIN)
end

# =============================================================================
# Franka ↔ SO101 Joint Name Mapping
# =============================================================================

"""
    FRANKA_TO_SO101_MAP

Mapping from Franka joint names to SO101-compatible names for state reporting.

The Franka has 7 DOF vs SO101's 5 DOF, so extra joints get unique names:
- joint6 → wrist_yaw (extra DOF)
- joint7 → wrist_twist (extra DOF)
"""
const FRANKA_TO_SO101_MAP = Dict{String, String}(
    "joint1" => "shoulder_pan",
    "joint2" => "shoulder_lift",
    "joint3" => "elbow_flex",
    "joint4" => "wrist_flex",
    "joint5" => "wrist_roll",
    "joint6" => "wrist_yaw",     # Extra DOF - unique name
    "joint7" => "wrist_twist"    # Extra DOF - unique name
)

"""
    SO101_TO_FRANKA_MAP

Mapping from SO101 joint names to Franka joint names.
Note: SO101 only has 5 arm joints, so wrist_yaw/wrist_twist don't map back.
"""
const SO101_TO_FRANKA_MAP = Dict{String, String}(
    "shoulder_pan" => "joint1",
    "shoulder_lift" => "joint2",
    "elbow_flex" => "joint3",
    "wrist_flex" => "joint4",
    "wrist_roll" => "joint5"    # No mapping for gripper - handled separately
)

"""
    franka_to_so101_joint(franka_name::String) -> Union{String, Nothing}

Convert a Franka joint name to the corresponding SO101 joint name.
Returns `nothing` if the joint name is not in the mapping.

# Example
```julia
so101_name = franka_to_so101_joint("joint1")  # "shoulder_pan"
```
"""
function franka_to_so101_joint(franka_name::String)
    return get(FRANKA_TO_SO101_MAP, franka_name, nothing)
end

"""
    gripper_range(::Type{FrankaPanda}) -> Tuple{Float64, Float64}

Return the gripper range in meters for Franka Panda.
Returns (closed_position, open_position) for finger joints.
"""
gripper_range(::Type{FrankaPanda}) = (0.0, 0.04)
