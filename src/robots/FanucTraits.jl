# FanucTraits.jl - Trait implementations for Fanuc industrial robot arms
#
# Fanuc industrial robots are 6-DOF arms with:
# - 6 revolute arm joints (joint_1 through joint_6)
# - No integrated gripper (end-effector tools are application-specific)
#
# Multiple variants are supported (M-10iA, CRX-10iA/L, M-900iB, etc.)
# with different reach and payload characteristics.

"""
    joint_specs(::Type{FanucArm}) -> Vector{JointSpec}

Return joint specifications for a generic Fanuc industrial robot arm.

Joint limits are approximate values that work across multiple Fanuc variants.
Actual limits vary by model. These are based on the M-10iA:
- joint_1: ±170° (base rotation)
- joint_2: ±60° (shoulder)
- joint_3: -75° to +240° (elbow)
- joint_4: ±190° (wrist roll)
- joint_5: ±125° (wrist pitch)
- joint_6: ±360° (wrist rotation)
"""
function joint_specs(::Type{FanucArm})
    return [
        JointSpec("joint_1", -170.0, 170.0, :revolute),
        JointSpec("joint_2", -60.0, 60.0, :revolute),
        JointSpec("joint_3", -75.0, 240.0, :revolute),
        JointSpec("joint_4", -190.0, 190.0, :revolute),
        JointSpec("joint_5", -125.0, 125.0, :revolute),
        JointSpec("joint_6", -360.0, 360.0, :revolute)
    ]
end

"""
    joint_names(::Type{FanucArm}) -> Vector{String}

Return ordered list of joint names for Fanuc industrial robot.
These names match the joint names in the MuJoCo model.
"""
function joint_names(::Type{FanucArm})
    return [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6"
    ]
end

"""
    ee_body_name(::Type{FanucArm}) -> String

Return the end-effector body name for Fanuc robots.
link_6 is the tool flange where end-effectors are mounted.
"""
ee_body_name(::Type{FanucArm}) = "link_6"

"""
    model_path(::Type{FanucArm}) -> String

Return the default path to a Fanuc MuJoCo model file (M-10iA).
For other variants, use `fanuc_model_path(variant_name)`.
"""
function model_path(::Type{FanucArm})
    return joinpath("robots", "fanuc_mujoco", "m10ia", "scene.xml")
end

"""
    fanuc_model_path(variant::String) -> String

Return the path to a specific Fanuc variant's MuJoCo model file.

# Available variants (as of last update)
- m10ia, m16ib20, m20ia, m20ib25
- m430ia2f, m6ib, m710ic50
- m900ia260l, m900ib700
- r1000ia80f, r2000ib210f, r2000ic165f
- cr7ia, cr35ia, crx10ial
- lrmate200i, lrmate200ib, lrmate200ic, lrmate200id

# Example
```julia
path = fanuc_model_path("crx10ial")
```
"""
function fanuc_model_path(variant::String)
    return joinpath("robots", "fanuc_mujoco", variant, "scene.xml")
end

# =============================================================================
# Fanuc ↔ SO101 Joint Name Mapping
# =============================================================================

"""
    FANUC_TO_SO101_MAP

Mapping from Fanuc joint names to SO101-compatible names for state reporting.

The Fanuc has 6 arm DOF vs SO101's 5 arm DOF, so joint_5 gets a unique name:
- joint_5 → wrist_flex2 (extra DOF compared to SO101's simplified wrist)
"""
const FANUC_TO_SO101_MAP = Dict{String, String}(
    "joint_1" => "shoulder_pan",
    "joint_2" => "shoulder_lift",
    "joint_3" => "elbow_flex",
    "joint_4" => "wrist_flex",
    "joint_5" => "wrist_flex2",   # Extra DOF - unique name
    "joint_6" => "wrist_roll"
)

"""
    SO101_TO_FANUC_MAP

Mapping from SO101 joint names to Fanuc joint names.
Note: SO101's simplified wrist maps to Fanuc's more complex wrist differently.
"""
const SO101_TO_FANUC_MAP = Dict{String, String}(
    "shoulder_pan" => "joint_1",
    "shoulder_lift" => "joint_2",
    "elbow_flex" => "joint_3",
    "wrist_flex" => "joint_4",
    "wrist_roll" => "joint_6"    # joint_5 (wrist_flex2) has no direct SO101 equivalent
)

"""
    fanuc_to_so101_joint(fanuc_name::String) -> Union{String, Nothing}

Convert a Fanuc joint name to the corresponding SO101 joint name.
Returns `nothing` if the joint name is not in the mapping.

# Example
```julia
so101_name = fanuc_to_so101_joint("joint_1")  # "shoulder_pan"
```
"""
function fanuc_to_so101_joint(fanuc_name::String)
    return get(FANUC_TO_SO101_MAP, fanuc_name, nothing)
end

"""
    so101_to_fanuc_joint(so101_name::String) -> Union{String, Nothing}

Convert an SO101 joint name to the corresponding Fanuc joint name.
Returns `nothing` if the joint name is not in the mapping.

# Example
```julia
fanuc_name = so101_to_fanuc_joint("shoulder_pan")  # "joint_1"
```
"""
function so101_to_fanuc_joint(so101_name::String)
    return get(SO101_TO_FANUC_MAP, so101_name, nothing)
end

"""
    fanuc_variants() -> Vector{String}

Return a list of known Fanuc robot variants that have MuJoCo models.

# Example
```julia
variants = fanuc_variants()
for v in variants
    println("Available: \$v")
end
```
"""
function fanuc_variants()
    return [
        "cr7ia",
        "cr35ia",
        "crx10ial",
        "lrmate200i",
        "lrmate200ib",
        "lrmate200ic",
        "lrmate200id",
        "m6ib",
        "m10ia",
        "m16ib20",
        "m20ia",
        "m20ib25",
        "m430ia2f",
        "m710ic50",
        "m900ia260l",
        "m900ib700",
        "r1000ia80f",
        "r2000ib210f",
        "r2000ic165f"
    ]
end
