# TrossenTraits.jl - Trait implementations for Trossen WXAI robot arm
#
# The Trossen WXAI is a 6-DOF robot arm with:
# - 6 revolute arm joints (joint_0 through joint_5)
# - 1 prismatic (slide) gripper (left_gripper, 0-0.044m range)
#
# Joint limits are derived from the MuJoCo model: wxai_base.xml

"""
    joint_specs(::Type{TrossenWXAI}) -> Vector{JointSpec}

Return joint specifications for the Trossen WXAI robot arm.

Joint limits derived from the MuJoCo model:
- joint_0 (base rotation): ±175° (±3.054 rad)
- joint_1 (shoulder): 0° to 180° (0 to π rad)
- joint_2 (elbow): 0° to 135° (0 to 2.356 rad)
- joint_3 (wrist flex): ±90° (±1.571 rad)
- joint_4 (wrist yaw): ±90° (±1.571 rad)
- joint_5 (wrist roll): ±180° (±π rad)
- left_gripper: 0 to 0.044 meters (prismatic)
"""
function joint_specs(::Type{TrossenWXAI})
    return [
        JointSpec("joint_0", -175.0, 175.0, :revolute),
        JointSpec("joint_1", 0.0, 180.0, :revolute),
        JointSpec("joint_2", 0.0, 135.0, :revolute),
        JointSpec("joint_3", -90.0, 90.0, :revolute),
        JointSpec("joint_4", -90.0, 90.0, :revolute),
        JointSpec("joint_5", -180.0, 180.0, :revolute),
        JointSpec("left_gripper", 0.0, 0.044, :prismatic)  # meters
    ]
end

"""
    joint_names(::Type{TrossenWXAI}) -> Vector{String}

Return ordered list of joint names for Trossen WXAI.
These names match the actuator/joint names in the MuJoCo model.
"""
function joint_names(::Type{TrossenWXAI})
    return [
        "joint_0",
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "left_gripper"
    ]
end

"""
    ee_body_name(::Type{TrossenWXAI}) -> String

Return the end-effector body name for Trossen WXAI.
link_6 is the wrist link where the gripper is attached.
"""
ee_body_name(::Type{TrossenWXAI}) = "link_6"

"""
    model_path(::Type{TrossenWXAI}) -> String

Return the path to the Trossen WXAI MuJoCo model file.
Path is relative to the project root directory.
"""
function model_path(::Type{TrossenWXAI})
    return joinpath("robots", "trossen_arm_mujoco", "trossen_arm_mujoco",
        "assets", "wxai", "scene.xml")
end

# =============================================================================
# SO101 ↔ Trossen Transform Functions
# =============================================================================

"""
    map_gripper_so101_to_trossen(so101_gripper_rad::Float64) -> Float64

Convert SO101 gripper position (radians, from hinge joint) to Trossen gripper
position (meters, slide joint).

# Mapping
- SO101 gripper: -10° to 100° (-0.1745 to 1.7453 rad)
  - -10° = closed, 100° = open
- Trossen gripper: 0m to 0.044m
  - 0m = closed, 0.044m = fully open
"""
function map_gripper_so101_to_trossen(so101_gripper_rad::Float64)
    # Convert radians to degrees for easier reasoning
    so101_deg = rad2deg(so101_gripper_rad)
    # Normalize: -10° → 0.0, 100° → 1.0
    normalized = clamp((so101_deg + 10.0) / 110.0, 0.0, 1.0)
    # Map to Trossen slide range: 0 to 0.044 meters
    return normalized * 0.044
end

"""
    map_gripper_trossen_to_so101(trossen_gripper_m::Float64) -> Float64

Convert Trossen gripper position (meters) to SO101 gripper position (degrees).

# Mapping
- Trossen gripper: 0m (closed) to 0.044m (open)
- SO101 gripper: -10° (closed) to 100° (open)
"""
function map_gripper_trossen_to_so101(trossen_gripper_m::Float64)
    # Normalize: 0m → 0.0, 0.044m → 1.0
    normalized = clamp(trossen_gripper_m / 0.044, 0.0, 1.0)
    # Map to SO101 range: -10° to 100°
    return normalized * 110.0 - 10.0
end

"""
    map_shoulder_lift_so101_to_trossen(so101_rad::Float64) -> Float64

Convert SO101 shoulder_lift to Trossen joint_1.

# Mapping
- SO101 shoulder_lift: -100° to 100° (0° = horizontal forward)
- Trossen joint_1: 0° to 180° (0° = straight up, 90° = horizontal forward)
- Formula: trossen = so101 + π/2
"""
function map_shoulder_lift_so101_to_trossen(so101_rad::Float64)
    return so101_rad + π / 2
end

"""
    map_shoulder_lift_trossen_to_so101(trossen_rad::Float64) -> Float64

Convert Trossen joint_1 to SO101 shoulder_lift.

# Mapping
- Trossen joint_1: 0° to 180° (90° = horizontal forward)
- SO101 shoulder_lift: -100° to 100° (0° = horizontal forward)
- Formula: so101 = trossen - π/2
"""
function map_shoulder_lift_trossen_to_so101(trossen_rad::Float64)
    return trossen_rad - π / 2
end

"""
    map_elbow_flex_so101_to_trossen(so101_rad::Float64) -> Float64

Convert SO101 elbow_flex to Trossen joint_2.

# Mapping
- SO101 elbow_flex: -97° to 97° (centered at 0° = straight arm)
- Trossen joint_2: 0° to 135° (0° = arm extended, 135° = fully bent)
- The axes are inverted with an offset.
- Formula: trossen = -so101 + 67.5° (center of Trossen range)
"""
function map_elbow_flex_so101_to_trossen(so101_rad::Float64)
    # Trossen joint_2 range center is ~67.5° (1.178 rad)
    return -so101_rad + deg2rad(67.5)
end

"""
    map_elbow_flex_trossen_to_so101(trossen_rad::Float64) -> Float64

Convert Trossen joint_2 to SO101 elbow_flex.

# Mapping
- Trossen joint_2: 0° to 135° (67.5° = center)
- SO101 elbow_flex: -97° to 97° (0° = center)
- Formula: so101 = -(trossen - 67.5°)
"""
function map_elbow_flex_trossen_to_so101(trossen_rad::Float64)
    return -(trossen_rad - deg2rad(67.5))
end

# =============================================================================
# Joint Mapping Dictionaries
# =============================================================================

"""
    SO101_TO_TROSSEN_MAP

Mapping table from SO101 joint names to Trossen actuator names and transform
functions. Each entry is (trossen_name, transform_function).

Used to convert SO101 joint commands to Trossen actuator commands.
"""
const SO101_TO_TROSSEN_MAP = Dict{String, Tuple{String, Function}}(
    "shoulder_pan" => ("joint_0", identity),
    "shoulder_lift" => ("joint_1", map_shoulder_lift_so101_to_trossen),
    "elbow_flex" => ("joint_2", map_elbow_flex_so101_to_trossen),
    "wrist_flex" => ("joint_3", identity),
    "wrist_roll" => ("joint_5", identity),  # Skip joint_4 (wrist yaw)
    "gripper" => ("left_gripper", map_gripper_so101_to_trossen)
)

"""
    TROSSEN_TO_SO101_MAP

Reverse mapping from Trossen actuator names to SO101 joint names and
inverse transform functions. Each entry is (so101_name, inverse_transform).

Used to report Trossen state in SO101-compatible joint names.
"""
const TROSSEN_TO_SO101_MAP = Dict{String, Tuple{String, Function}}(
    "joint_0" => ("shoulder_pan", identity),
    "joint_1" => ("shoulder_lift", map_shoulder_lift_trossen_to_so101),
    "joint_2" => ("elbow_flex", map_elbow_flex_trossen_to_so101),
    "joint_3" => ("wrist_flex", identity),
    "joint_5" => ("wrist_roll", identity),
    "left_gripper" => ("gripper", map_gripper_trossen_to_so101)
)

"""
    gripper_range(::Type{TrossenWXAI}) -> Tuple{Float64, Float64}

Return the gripper range in meters for Trossen WXAI.
Returns (closed_position, open_position).
"""
gripper_range(::Type{TrossenWXAI}) = (0.0, 0.044)
