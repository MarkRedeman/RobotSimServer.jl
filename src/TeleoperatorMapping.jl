# TeleoperatorMapping - Maps joint commands between different robot types
#
# This module provides strategies for mapping leader robot joint commands to
# follower robot actuators. It uses Julia's multiple dispatch to select the
# appropriate strategy based on the leader/follower type pair.
#
# Strategies:
# - DirectMapping: Same robot type, pass through unchanged
# - JointNameMapping: Compatible robots with only name differences (SO101 ↔ LeKiwi)
# - IKBasedMapping: Different kinematics, use FK on leader + IK on follower
#
# Usage:
#   include("src/RobotTypes.jl")
#   include("src/Kinematics.jl")
#   include("src/TeleoperatorMapping.jl")
#   
#   ctx = create_teleop_context(SO101, TrossenWXAI, follower_model, follower_data)
#   mapped_joints = map_joints(ctx, leader_joints, follower_model, follower_data)
#
# Dependencies:
#   This file assumes RobotTypes.jl and Kinematics.jl are already loaded.

using LinearAlgebra
using MuJoCo
using MuJoCo.LibMuJoCo

# Note: Robot types (SO101, TrossenWXAI, LeKiwiArm, FrankaPanda, FanucArm) and
# their trait methods (joint_names, ee_body_name, model_path) are defined in
# RobotTypes.jl which must be included before this file.
#
# The trait files (SO101Traits.jl, TrossenTraits.jl, etc.) define the actual
# joint specifications, joint names, and transform functions for each robot.

# =============================================================================
# Joint Name Mapping Tables
# =============================================================================

"""
SO101 ↔ LeKiwi joint name mapping.
Both robots have identical joint structure, just different naming conventions.
"""
const SO101_LEKIWI_MAP = Dict{String, String}(
    "shoulder_pan" => "Rotation",
    "shoulder_lift" => "Pitch",
    "elbow_flex" => "Elbow",
    "wrist_flex" => "Wrist_Pitch",
    "wrist_roll" => "Wrist_Roll",
    "gripper" => "Jaw"
)

"""
LeKiwi → SO101 joint name mapping (reverse of SO101_LEKIWI_MAP).
"""
const LEKIWI_SO101_MAP = Dict{String, String}(v => k for (k, v) in SO101_LEKIWI_MAP)

"""
SO101 → Trossen WXAI joint mapping with transform functions.
Trossen has different joint conventions requiring value transforms.

Format: SO101_name => (Trossen_name, transform_to_trossen, transform_from_trossen)
"""
const SO101_TROSSEN_MAP = Dict{String, Tuple{String, Function, Function}}(
    "shoulder_pan" => ("joint_0", identity, identity),
    "shoulder_lift" => ("joint_1",
        deg -> deg + 90.0,      # SO101 0° = Trossen 90°
        deg -> deg - 90.0),
    "elbow_flex" => ("joint_2",
        deg -> -deg + 67.5,     # Inverted and offset
        deg -> -(deg - 67.5)),
    "wrist_flex" => ("joint_3", identity, identity),
    "wrist_roll" => ("joint_5", identity, identity),
    # gripper: SO101 degrees → Trossen meters
    "gripper" => ("gripper",
        deg -> clamp((deg + 10.0) / 110.0, 0.0, 1.0) * 0.044,
        meters -> (meters / 0.044) * 110.0 - 10.0)
)

# =============================================================================
# Mapping Strategy Types
# =============================================================================

"""
    MappingStrategy

Abstract base type for joint mapping strategies.
Subtypes determine how leader joint commands are transformed to follower format.
"""
abstract type MappingStrategy end

"""
    DirectMapping <: MappingStrategy

Pass-through mapping for same robot type pairs.
No transformation applied; joints are returned unchanged.
"""
struct DirectMapping <: MappingStrategy end

"""
    JointNameMapping <: MappingStrategy

Name-only mapping for compatible robot types (SO101 ↔ LeKiwi).
Joint values are preserved, only names are translated.
"""
struct JointNameMapping <: MappingStrategy end

"""
    IKBasedMapping <: MappingStrategy

Inverse kinematics-based mapping for robots with different kinematics.

Algorithm:
1. Apply leader joints to shadow model
2. Forward kinematics to get leader EE position
3. Scale position from leader workspace to follower workspace
4. Inverse kinematics to compute follower joint angles
"""
struct IKBasedMapping <: MappingStrategy end

# =============================================================================
# Strategy Selection via Multiple Dispatch
# =============================================================================

"""
    mapping_strategy(::Type{L}, ::Type{F}) where {L, F}

Determine the appropriate mapping strategy for a leader→follower pair.

Returns:
- `DirectMapping()` when leader and follower are the same type
- `JointNameMapping()` for compatible types (SO101 ↔ LeKiwi)
- `IKBasedMapping()` for robots with different kinematics

# Examples
```julia
mapping_strategy(SO101, SO101)           # => DirectMapping()
mapping_strategy(SO101, LeKiwiArm)       # => JointNameMapping()
mapping_strategy(SO101, FrankaPanda)     # => IKBasedMapping()
```
"""
mapping_strategy(::Type{L}, ::Type{L}) where {L <: AbstractRobotArm} = DirectMapping()

# SO101 ↔ LeKiwi are compatible (same joint structure, different names)
mapping_strategy(::Type{SO101}, ::Type{LeKiwiArm}) = JointNameMapping()
mapping_strategy(::Type{LeKiwiArm}, ::Type{SO101}) = JointNameMapping()

# All other combinations use IK-based mapping
function mapping_strategy(::Type{L}, ::Type{F}) where {L <: AbstractRobotArm,
        F <: AbstractRobotArm}
    return IKBasedMapping()
end

# =============================================================================
# Teleoperator Context
# =============================================================================

"""
    TeleoperatorContext{L, F}

Holds configuration and cached data for teleoperation between robot types.

# Fields
- `leader_type`: Type of the leader robot
- `follower_type`: Type of the follower robot
- `strategy`: MappingStrategy instance
- `shadow_model`: MuJoCo model of leader (for IK-based mapping), or nothing
- `shadow_data`: MuJoCo data for leader shadow model, or nothing
- `ik_config`: IKConfig for inverse kinematics (if applicable)
- `leader_home`: Home position of leader EE (for workspace scaling)
- `follower_home`: Home position of follower EE (for workspace scaling)
- `leader_joint_map`: Dict mapping joint names to qpos indices for leader
- `follower_joint_map`: Dict mapping joint names to qpos indices for follower

# Usage
```julia
ctx = create_teleop_context(SO101, FrankaPanda, follower_model, follower_data)
mapped = map_joints(ctx, leader_joints, follower_model, follower_data)
```
"""
mutable struct TeleoperatorContext{L <: AbstractRobotArm, F <: AbstractRobotArm}
    leader_type::Type{L}
    follower_type::Type{F}
    strategy::MappingStrategy
    shadow_model::Any  # Union{Model, Nothing}
    shadow_data::Any   # Union{Data, Nothing}
    ik_config::Any     # IKConfig or nothing
    leader_home::Vector{Float64}
    follower_home::Vector{Float64}
    leader_joint_map::Dict{String, Int}
    follower_joint_map::Dict{String, Int}
end

# =============================================================================
# Context Creation
# =============================================================================

"""
    build_joint_map(model, joint_names::Vector{String}) -> Dict{String, Int}

Build a mapping from joint names to qpos indices for a MuJoCo model.

# Arguments
- `model`: MuJoCo Model
- `joint_names`: List of joint names to map

# Returns
- Dict mapping joint name => qpos index (1-indexed)
"""
function build_joint_map(model, joint_names::Vector{String})
    joint_map = Dict{String, Int}()
    for name in joint_names
        joint_id = mj_name2id(model, Int32(LibMuJoCo.mjOBJ_JOINT), name)
        if joint_id >= 0
            addr = model.jnt_qposadr[joint_id + 1] + 1  # 1-indexed
            joint_map[name] = addr
        end
    end
    return joint_map
end

"""
    create_teleop_context(leader_type::Type{L}, follower_type::Type{F},
                          follower_model, follower_data;
                          project_root::String="") where {L, F}

Create a TeleoperatorContext for mapping between robot types.

# Arguments
- `leader_type`: Type of the leader robot (e.g., SO101)
- `follower_type`: Type of the follower robot (e.g., FrankaPanda)
- `follower_model`: MuJoCo model of the follower robot
- `follower_data`: MuJoCo data for the follower robot
- `project_root`: Optional path to project root (for loading shadow models)

# Returns
- `TeleoperatorContext` with appropriate mapping strategy and cached data

# Details
For `DirectMapping` and `JointNameMapping`:
- No shadow model is loaded
- Home positions are computed from follower model

For `IKBasedMapping`:
- Loads leader shadow model for forward kinematics
- Computes home positions for both robots
- Initializes IK configuration

# Examples
```julia
# Same robot type - direct mapping
ctx = create_teleop_context(SO101, SO101, model, data)

# Compatible robots - name mapping only
ctx = create_teleop_context(SO101, LeKiwiArm, model, data)

# Different kinematics - IK-based mapping
ctx = create_teleop_context(SO101, FrankaPanda, model, data)
```
"""
function create_teleop_context(
        leader_type::Type{L}, follower_type::Type{F},
        follower_model, follower_data;
        project_root::String = "") where {L <: AbstractRobotArm, F <: AbstractRobotArm}
    strategy = mapping_strategy(leader_type, follower_type)

    # Initialize with defaults
    shadow_model = nothing
    shadow_data = nothing
    ik_config = nothing
    leader_home = Float64[]
    follower_home = Float64[]
    leader_joint_map = Dict{String, Int}()
    follower_joint_map = Dict{String, Int}()

    # Build follower joint map
    follower_joint_map = build_joint_map(follower_model, joint_names(follower_type))

    # Compute follower home position
    follower_ee = ee_body_name(follower_type)
    follower_home = get_home_position(follower_model, follower_data, follower_ee)

    if strategy isa DirectMapping
        # No additional setup needed for direct mapping
        leader_home = follower_home
        leader_joint_map = follower_joint_map

    elseif strategy isa JointNameMapping
        # No shadow model needed for name-only mapping
        # Leader and follower have same joint structure
        leader_home = follower_home

    elseif strategy isa IKBasedMapping
        # Load leader shadow model for FK
        leader_xml = if isempty(project_root)
            model_path(leader_type)
        else
            joinpath(project_root, model_path(leader_type))
        end

        if isfile(leader_xml)
            shadow_model = load_model(leader_xml)
            shadow_data = init_data(shadow_model)

            # Build leader joint map
            leader_joint_map = build_joint_map(shadow_model, joint_names(leader_type))

            # Compute leader home position
            leader_ee = ee_body_name(leader_type)
            leader_home = get_home_position(shadow_model, shadow_data, leader_ee)

            # Create IK configuration tuned for real-time control
            ik_config = IKConfig(
                max_iter = 50,      # Fewer iterations for real-time (30fps)
                tol = 0.005,        # 5mm tolerance
                step_size = 0.5,
                damping = 0.01
            )
        else
            @warn "Leader model not found, falling back to follower-only mode" path = leader_xml
            leader_home = follower_home
        end
    end

    return TeleoperatorContext{L, F}(
        leader_type,
        follower_type,
        strategy,
        shadow_model,
        shadow_data,
        ik_config,
        leader_home,
        follower_home,
        leader_joint_map,
        follower_joint_map
    )
end

# =============================================================================
# Helper Functions
# =============================================================================

"""
    is_revolute_joint(model, joint_name::String) -> Bool

Check if a joint is revolute (hinge) type.
"""
function is_revolute_joint(model, joint_name::String)
    joint_id = mj_name2id(model, Int32(LibMuJoCo.mjOBJ_JOINT), joint_name)
    if joint_id < 0
        return true  # Default to revolute if not found
    end
    # mjJNT_HINGE = 3
    return model.jnt_type[joint_id + 1] == 3
end

"""
    apply_joints_to_shadow!(data, model, ::Type{L}, joints::Dict{String, Float64},
                            joint_map::Dict{String, Int}) where L

Apply joint dictionary to shadow model's qpos.

Converts degrees to radians for revolute joints.
Handles robot-specific quirks (e.g., Trossen gripper is in meters).

# Arguments
- `data`: MuJoCo data to modify
- `model`: MuJoCo model (for joint type checking)
- `L`: Leader robot type
- `joints`: Dict of joint_name => value (degrees for revolute, meters for slide)
- `joint_map`: Dict of joint_name => qpos index
"""
function apply_joints_to_shadow!(data, model, ::Type{L}, joints::Dict{String, Float64},
        joint_map::Dict{String, Int}) where {L <: AbstractRobotArm}
    for (name, val) in joints
        if haskey(joint_map, name)
            idx = joint_map[name]
            # Check if joint is revolute (convert deg→rad) or slide (keep as-is)
            if is_revolute_joint(model, name)
                data.qpos[idx] = deg2rad(val)
            else
                data.qpos[idx] = val  # Slide joint, already in meters
            end
        end
    end
end

# Special handling for Trossen gripper (slide joint in meters)
function apply_joints_to_shadow!(data, model, ::Type{TrossenWXAI},
        joints::Dict{String, Float64},
        joint_map::Dict{String, Int})
    for (name, val) in joints
        if haskey(joint_map, name)
            idx = joint_map[name]
            if name == "gripper"
                # Trossen gripper is a slide joint in meters
                data.qpos[idx] = val
            elseif is_revolute_joint(model, name)
                data.qpos[idx] = deg2rad(val)
            else
                data.qpos[idx] = val
            end
        end
    end
end

"""
    extract_follower_joints(model, data, ::Type{F},
                            joint_map::Dict{String, Int}) where F -> Dict{String, Float64}

Read qpos from model/data and return Dict with robot's native joint names.

Converts radians to degrees for revolute joints.

# Arguments
- `model`: MuJoCo model
- `data`: MuJoCo data
- `F`: Follower robot type
- `joint_map`: Dict of joint_name => qpos index

# Returns
- Dict of joint_name => value (degrees for revolute, meters for slide)
"""
function extract_follower_joints(model, data, ::Type{F},
        joint_map::Dict{String, Int}) where {F <: AbstractRobotArm}
    result = Dict{String, Float64}()
    for (name, idx) in joint_map
        val = data.qpos[idx]
        if is_revolute_joint(model, name)
            result[name] = rad2deg(val)
        else
            result[name] = val  # Slide joint, already in meters
        end
    end
    return result
end

# Special handling for Trossen gripper
function extract_follower_joints(model, data, ::Type{TrossenWXAI},
        joint_map::Dict{String, Int})
    result = Dict{String, Float64}()
    for (name, idx) in joint_map
        val = data.qpos[idx]
        if name == "gripper"
            # Trossen gripper is slide joint, keep in meters
            result[name] = val
        elseif is_revolute_joint(model, name)
            result[name] = rad2deg(val)
        else
            result[name] = val
        end
    end
    return result
end

# =============================================================================
# Mapping Implementations
# =============================================================================

"""
    map_joints_impl(::DirectMapping, ctx, joints, model, data) -> Dict{String, Float64}

Direct pass-through mapping for same robot type pairs.
Returns joints unchanged.
"""
function map_joints_impl(::DirectMapping, ctx::TeleoperatorContext,
        joints::Dict{String, Float64}, model, data)
    return copy(joints)
end

"""
    map_joints_impl(::JointNameMapping, ctx, joints, model, data) -> Dict{String, Float64}

Name translation mapping for compatible robot types (SO101 ↔ LeKiwi).
Joint values are preserved, only names are translated.
"""
function map_joints_impl(::JointNameMapping, ctx::TeleoperatorContext,
        joints::Dict{String, Float64}, model, data)
    result = Dict{String, Float64}()

    # Determine mapping direction
    if ctx.leader_type == SO101 && ctx.follower_type == LeKiwiArm
        name_map = SO101_LEKIWI_MAP
    elseif ctx.leader_type == LeKiwiArm && ctx.follower_type == SO101
        name_map = LEKIWI_SO101_MAP
    else
        # Fallback: no translation
        return copy(joints)
    end

    for (leader_name, val) in joints
        if haskey(name_map, leader_name)
            follower_name = name_map[leader_name]
            result[follower_name] = val
        else
            # Pass through unknown joints unchanged
            result[leader_name] = val
        end
    end

    return result
end

"""
    map_joints_impl(::IKBasedMapping, ctx, joints, model, data) -> Dict{String, Float64}

IK-based mapping for robots with different kinematics.

Algorithm:
1. Apply leader joints to shadow model (degrees→radians)
2. Forward kinematics to get leader EE position
3. Scale position from leader workspace to follower workspace
4. Inverse kinematics to compute follower joint angles
5. Extract and return as Dict with follower's native joint names

# Notes
- Gripper is handled separately (direct mapping with appropriate scaling)
- Returns empty dict if shadow model is not loaded
"""
function map_joints_impl(::IKBasedMapping, ctx::TeleoperatorContext,
        joints::Dict{String, Float64}, model, data)
    # Check if shadow model is available
    if ctx.shadow_model === nothing || ctx.shadow_data === nothing
        @warn "Shadow model not available for IK-based mapping"
        return Dict{String, Float64}()
    end

    # Extract gripper command for separate handling
    gripper_cmd = get(joints, "gripper", nothing)
    arm_joints = filter(p -> p.first != "gripper", joints)

    # Step 1: Apply leader joints to shadow model
    apply_joints_to_shadow!(ctx.shadow_data, ctx.shadow_model,
        ctx.leader_type, arm_joints, ctx.leader_joint_map)

    # Step 2: Forward kinematics to get leader EE position
    leader_ee = ee_body_name(ctx.leader_type)
    leader_pos = forward_kinematics(ctx.shadow_model, ctx.shadow_data, leader_ee)

    # Step 3: Scale position to follower workspace
    target_pos = scale_position(leader_pos, ctx.leader_home, ctx.follower_home)

    # Step 4: Solve IK for follower joint angles
    # Use a temporary copy to avoid modifying the actual follower data
    ik_data = init_data(model)
    ik_data.qpos .= data.qpos  # Start from current position for continuity

    follower_ee = ee_body_name(ctx.follower_type)
    result = inverse_kinematics!(model, ik_data, follower_ee, target_pos;
        config = ctx.ik_config)

    # Step 5: Extract follower joints from IK result
    mapped_joints = extract_follower_joints(model, ik_data,
        ctx.follower_type, ctx.follower_joint_map)

    # Step 6: Handle gripper separately
    if gripper_cmd !== nothing
        mapped_joints["gripper"] = map_gripper(ctx.leader_type, ctx.follower_type,
            gripper_cmd)
    end

    return mapped_joints
end

"""
    map_gripper(::Type{L}, ::Type{F}, value::Float64) where {L, F} -> Float64

Map gripper value from leader format to follower format.

# Arguments
- `L`: Leader robot type
- `F`: Follower robot type
- `value`: Gripper value in leader format (degrees for SO101/LeKiwi)

# Returns
- Gripper value in follower format (degrees or meters)
"""
function map_gripper(::Type{L}, ::Type{F}, value::Float64) where {L <: AbstractRobotArm,
        F <: AbstractRobotArm}
    # Default: pass through unchanged
    return value
end

# SO101 → Trossen: degrees → meters
function map_gripper(::Type{SO101}, ::Type{TrossenWXAI}, value::Float64)
    # SO101 gripper: -10° to 100° where -10° = closed, 100° = open
    # Trossen gripper: 0m to 0.044m where 0m = closed, 0.044m = fully open
    normalized = clamp((value + 10.0) / 110.0, 0.0, 1.0)
    return normalized * 0.044
end

# Trossen → SO101: meters → degrees
function map_gripper(::Type{TrossenWXAI}, ::Type{SO101}, value::Float64)
    normalized = clamp(value / 0.044, 0.0, 1.0)
    return normalized * 110.0 - 10.0
end

# SO101 → Franka: degrees → actuator units (0-255)
function map_gripper(::Type{SO101}, ::Type{FrankaPanda}, value::Float64)
    normalized = clamp((value + 10.0) / 110.0, 0.0, 1.0)
    return normalized * 255.0
end

# Franka → SO101: finger position (meters) → degrees
function map_gripper(::Type{FrankaPanda}, ::Type{SO101}, value::Float64)
    # Franka finger position: 0m (closed) to 0.04m (open)
    normalized = clamp(value / 0.04, 0.0, 1.0)
    return normalized * 110.0 - 10.0
end

# =============================================================================
# Main Mapping Interface
# =============================================================================

"""
    map_joints(ctx::TeleoperatorContext, joints::Dict{String, Float64},
               follower_model, follower_data) -> Dict{String, Float64}

Map leader joint commands to follower robot format.

Dispatches to the appropriate `map_joints_impl()` based on the context's
mapping strategy.

# Arguments
- `ctx`: TeleoperatorContext with mapping configuration
- `joints`: Dict of leader joint names → values (degrees for revolute)
- `follower_model`: MuJoCo model of follower robot
- `follower_data`: MuJoCo data of follower robot

# Returns
- Dict of follower joint names → values in follower's native format

# Examples
```julia
# Create context
ctx = create_teleop_context(SO101, FrankaPanda, model, data)

# Map leader commands to follower
leader_joints = Dict("shoulder_pan" => 45.0, "gripper" => 50.0)
follower_joints = map_joints(ctx, leader_joints, model, data)
```
"""
function map_joints(ctx::TeleoperatorContext, joints::Dict{String, Float64},
        follower_model, follower_data)
    return map_joints_impl(ctx.strategy, ctx, joints, follower_model, follower_data)
end

# Convenience method accepting Dict with non-Float64 values
function map_joints(ctx::TeleoperatorContext, joints::Dict{String, <:Real},
        follower_model, follower_data)
    converted = Dict{String, Float64}(k => Float64(v) for (k, v) in joints)
    return map_joints(ctx, converted, follower_model, follower_data)
end

# =============================================================================
# State Reporting (Follower → Leader format)
# =============================================================================

"""
    get_state_for_leader(ctx::TeleoperatorContext, model, data) -> Dict{String, Float64}

Get follower's current joint state in leader's joint name format.

Used for broadcasting state back to the leader controller using its
expected joint naming convention.

# Arguments
- `ctx`: TeleoperatorContext with mapping configuration
- `model`: MuJoCo model of follower robot
- `data`: MuJoCo data of follower robot

# Returns
- Dict of joint_name => value in leader's naming convention

# Notes
For DirectMapping: Returns follower joint names unchanged
For JointNameMapping: Translates follower names to leader names
For IKBasedMapping: Returns mapped approximation of leader joints
"""
function get_state_for_leader(ctx::TeleoperatorContext, model, data)
    # Get current follower state
    follower_state = extract_follower_joints(model, data,
        ctx.follower_type, ctx.follower_joint_map)

    return get_state_for_leader_impl(ctx.strategy, ctx, follower_state, model, data)
end

function get_state_for_leader_impl(::DirectMapping, ctx::TeleoperatorContext,
        state::Dict{String, Float64}, model, data)
    return state
end

function get_state_for_leader_impl(::JointNameMapping, ctx::TeleoperatorContext,
        state::Dict{String, Float64}, model, data)
    result = Dict{String, Float64}()

    # Determine reverse mapping direction
    if ctx.leader_type == SO101 && ctx.follower_type == LeKiwiArm
        name_map = LEKIWI_SO101_MAP  # Reverse: LeKiwi → SO101
    elseif ctx.leader_type == LeKiwiArm && ctx.follower_type == SO101
        name_map = SO101_LEKIWI_MAP  # Reverse: SO101 → LeKiwi
    else
        return state
    end

    for (follower_name, val) in state
        if haskey(name_map, follower_name)
            leader_name = name_map[follower_name]
            result[leader_name] = val
        else
            result[follower_name] = val
        end
    end

    return result
end

function get_state_for_leader_impl(::IKBasedMapping, ctx::TeleoperatorContext,
        state::Dict{String, Float64}, model, data)
    # For IK-based mapping, we report the follower state with leader-compatible names
    # This is an approximation since the kinematics are different
    result = Dict{String, Float64}()

    # Map common joint names
    leader_names = joint_names(ctx.leader_type)
    follower_names = joint_names(ctx.follower_type)

    # Create a best-effort mapping based on joint position
    for (i, leader_name) in enumerate(leader_names)
        if i <= length(follower_names)
            follower_name = follower_names[i]
            if haskey(state, follower_name)
                result[leader_name] = state[follower_name]
            end
        end
    end

    # Handle gripper specially
    if haskey(state, "gripper")
        result["gripper"] = map_gripper(ctx.follower_type, ctx.leader_type,
            state["gripper"])
    end

    return result
end

# =============================================================================
# Utility Functions
# =============================================================================

"""
    workspace_scale(ctx::TeleoperatorContext) -> Float64

Get the workspace scale factor between leader and follower robots.
This is the ratio of follower home distance to leader home distance.
"""
function workspace_scale(ctx::TeleoperatorContext)
    leader_dist = norm(ctx.leader_home)
    if leader_dist < 1e-6
        return 1.0
    end
    return norm(ctx.follower_home) / leader_dist
end

"""
    describe(ctx::TeleoperatorContext) -> String

Get a human-readable description of the teleoperator context.
"""
function describe(ctx::TeleoperatorContext)
    scale = workspace_scale(ctx)
    return """
    TeleoperatorContext:
      Leader: $(ctx.leader_type)
      Follower: $(ctx.follower_type)
      Strategy: $(typeof(ctx.strategy))
      Leader home: $(round.(ctx.leader_home, digits=3)) ($(round(norm(ctx.leader_home), digits=3))m)
      Follower home: $(round.(ctx.follower_home, digits=3)) ($(round(norm(ctx.follower_home), digits=3))m)
      Workspace scale: $(round(scale, digits=2))x
      Shadow model: $(ctx.shadow_model !== nothing ? "loaded" : "none")
    """
end
