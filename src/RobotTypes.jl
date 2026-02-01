# RobotTypes.jl - Core type system for robot arms in the teleoperation framework
#
# This module defines the abstract type hierarchy and concrete robot types for
# a Julia-based MuJoCo teleoperation framework. Uses Julia's multiple dispatch
# to provide zero-cost phantom types for compile-time robot differentiation.
#
# Usage:
#   include("src/RobotTypes.jl")
#   specs = joint_specs(SO101)
#   path = model_path(SO101)

"""
    AbstractRobotArm

Abstract base type for all robot arm implementations. Concrete robot types
should inherit from this to enable multiple dispatch on robot-specific traits.

# Example
```julia
struct MyRobot <: AbstractRobotArm end
joint_specs(::Type{MyRobot}) = [JointSpec("joint1", -90.0, 90.0, :revolute)]
```
"""
abstract type AbstractRobotArm end

# =============================================================================
# Concrete Robot Type Tags (Zero-cost phantom types for dispatch)
# =============================================================================

"""
    SO101 <: AbstractRobotArm

SO-ARM100 robot arm from TheRobotStudio. A 6-DOF desktop robot arm with
5 revolute joints and a revolute gripper.
"""
struct SO101 <: AbstractRobotArm end

"""
    TrossenWXAI <: AbstractRobotArm

Trossen Robotics WXAI arm. A 6-DOF robot arm with 6 revolute joints and
a prismatic (slide) gripper.
"""
struct TrossenWXAI <: AbstractRobotArm end

"""
    LeKiwiArm <: AbstractRobotArm

LeKiwi mobile manipulator arm (based on SO-ARM100). The arm portion has
6 revolute joints matching the SO101 configuration.
"""
struct LeKiwiArm <: AbstractRobotArm end

"""
    FrankaPanda <: AbstractRobotArm

Franka Emika Panda robot arm. A 7-DOF research robot arm with
high precision and force feedback capabilities.
"""
struct FrankaPanda <: AbstractRobotArm end

"""
    FanucArm <: AbstractRobotArm

Fanuc industrial robot arm. A 6-DOF industrial robot available in
multiple variants (M-10iA, CRX-10iA/L, etc.).
"""
struct FanucArm <: AbstractRobotArm end

# =============================================================================
# Joint Specification
# =============================================================================

"""
    JointSpec

Specification for a single robot joint, including name, limits, and type.

# Fields
- `name::String`: Joint name as used in the MuJoCo model
- `limit_low::Float64`: Lower joint limit in degrees (or meters for prismatic)
- `limit_high::Float64`: Upper joint limit in degrees (or meters for prismatic)
- `joint_type::Symbol`: Either `:revolute` (hinge) or `:prismatic` (slide)

# Example
```julia
JointSpec("shoulder_pan", -110.0, 110.0, :revolute)
JointSpec("gripper", 0.0, 0.044, :prismatic)  # meters for slide joints
```
"""
struct JointSpec
    name::String
    limit_low::Float64   # degrees for revolute, meters for prismatic
    limit_high::Float64  # degrees for revolute, meters for prismatic
    joint_type::Symbol   # :revolute or :prismatic
end

# =============================================================================
# Robot Type Map for CLI Parsing
# =============================================================================

"""
    ROBOT_TYPE_MAP

Dictionary mapping CLI string identifiers to robot type tags.
Used for parsing command-line arguments and configuration files.

# Example
```julia
robot_type = ROBOT_TYPE_MAP["so101"]  # Returns SO101
specs = joint_specs(robot_type)
```
"""
const ROBOT_TYPE_MAP = Dict{String, Type{<:AbstractRobotArm}}(
    "so101" => SO101,
    "trossen" => TrossenWXAI,
    "trossen_wxai" => TrossenWXAI,
    "wxai" => TrossenWXAI,
    "lekiwi" => LeKiwiArm,
    "lekiwi_arm" => LeKiwiArm,
    "franka" => FrankaPanda,
    "panda" => FrankaPanda,
    "franka_panda" => FrankaPanda,
    "fanuc" => FanucArm
)

# =============================================================================
# Trait Function Stubs (to be implemented by robot-specific files)
# =============================================================================

"""
    joint_specs(::Type{T}) where T <: AbstractRobotArm -> Vector{JointSpec}

Return the joint specifications for a robot type.
Each robot-specific traits file must implement this method.

# Example
```julia
specs = joint_specs(SO101)
for spec in specs
    println("\$(spec.name): \$(spec.limit_low)° to \$(spec.limit_high)°")
end
```
"""
function joint_specs end

"""
    joint_names(::Type{T}) where T <: AbstractRobotArm -> Vector{String}

Return the ordered list of joint names for a robot type.

# Example
```julia
names = joint_names(SO101)
# ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
```
"""
function joint_names end

"""
    ee_body_name(::Type{T}) where T <: AbstractRobotArm -> String

Return the name of the end-effector body in the MuJoCo model.
Used for forward/inverse kinematics calculations.

# Example
```julia
ee_name = ee_body_name(SO101)  # "gripper"
```
"""
function ee_body_name end

"""
    model_path(::Type{T}) where T <: AbstractRobotArm -> String

Return the path to the MuJoCo XML model file, relative to the project root.

# Example
```julia
path = model_path(SO101)
full_path = joinpath(@__DIR__, "..", path)
```
"""
function model_path end

"""
    num_joints(::Type{T}) where T <: AbstractRobotArm -> Int

Return the number of controllable joints for a robot type.
Default implementation uses length of joint_specs.

# Example
```julia
n = num_joints(SO101)  # 6
```
"""
num_joints(::Type{T}) where {T <: AbstractRobotArm} = length(joint_specs(T))

"""
    has_prismatic_gripper(::Type{T}) where T <: AbstractRobotArm -> Bool

Return true if the robot has a prismatic (slide) gripper instead of revolute.
Default implementation checks the last joint type.

# Example
```julia
has_prismatic_gripper(TrossenWXAI)  # true
has_prismatic_gripper(SO101)        # false
```
"""
function has_prismatic_gripper(::Type{T}) where {T <: AbstractRobotArm}
    specs = joint_specs(T)
    if isempty(specs)
        return false
    end
    return last(specs).joint_type == :prismatic
end

# =============================================================================
# Include Robot-Specific Trait Files
# =============================================================================

include("robots/SO101Traits.jl")
include("robots/TrossenTraits.jl")
include("robots/LeKiwiTraits.jl")
include("robots/FrankaTraits.jl")
include("robots/FanucTraits.jl")
