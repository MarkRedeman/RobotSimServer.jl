# SO101Traits.jl - Trait implementations for SO-ARM100 robot arm
#
# The SO-ARM100 is a 6-DOF desktop robot arm from TheRobotStudio with:
# - 5 revolute arm joints (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll)
# - 1 revolute gripper joint
#
# Joint limits are derived from the MuJoCo model: so101_new_calib.xml

"""
    joint_specs(::Type{SO101}) -> Vector{JointSpec}

Return joint specifications for the SO-ARM100 robot arm.

Joint limits are in degrees and derived from the MuJoCo model radians:
- shoulder_pan: ±110° (±1.92 rad)
- shoulder_lift: ±100° (±1.745 rad)
- elbow_flex: ±97° (±1.69 rad)
- wrist_flex: ±95° (±1.658 rad)
- wrist_roll: -157° to +163° (-2.744 to +2.841 rad)
- gripper: -10° to +100° (-0.175 to +1.745 rad)
"""
function joint_specs(::Type{SO101})
    return [
        JointSpec("shoulder_pan", -110.0, 110.0, :revolute),
        JointSpec("shoulder_lift", -100.0, 100.0, :revolute),
        JointSpec("elbow_flex", -97.0, 97.0, :revolute),
        JointSpec("wrist_flex", -95.0, 95.0, :revolute),
        JointSpec("wrist_roll", -157.0, 163.0, :revolute),
        JointSpec("gripper", -10.0, 100.0, :revolute)
    ]
end

"""
    joint_names(::Type{SO101}) -> Vector{String}

Return ordered list of joint names for SO-ARM100.
These names match the actuator/joint names in the MuJoCo model.
"""
function joint_names(::Type{SO101})
    return [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper"
    ]
end

"""
    ee_body_name(::Type{SO101}) -> String

Return the end-effector body name for SO-ARM100.
The gripper body is the end-effector used for IK targeting.
"""
ee_body_name(::Type{SO101}) = "gripper"

"""
    model_path(::Type{SO101}) -> String

Return the path to the SO-ARM100 MuJoCo model file.
Path is relative to the project root directory.
"""
function model_path(::Type{SO101})
    return joinpath("robots", "SO-ARM100", "Simulation", "SO101", "so101_new_calib.xml")
end

"""
    gripper_range(::Type{SO101}) -> Tuple{Float64, Float64}

Return the gripper range in degrees for SO-ARM100.
Returns (closed_position, open_position).
"""
gripper_range(::Type{SO101}) = (-10.0, 100.0)
