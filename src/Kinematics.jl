# Kinematics - Forward and Inverse Kinematics for MuJoCo robots
#
# This module provides:
# - Forward kinematics (FK): Get end-effector position from joint angles
# - Inverse kinematics (IK): Compute joint angles for a target position
# - Workspace scaling: Map positions between robots with different sizes
#
# The IK solver uses Damped Least Squares (DLS) for numerical stability,
# especially near kinematic singularities.
#
# Usage:
#   include("src/Kinematics.jl")
#
#   # Get end-effector position
#   pos = forward_kinematics(model, data, "gripper")
#
#   # Solve IK for target position
#   result = inverse_kinematics!(model, data, "link_6", [0.5, 0.0, 0.8])
#
#   # Scale position between robots
#   target = scale_position(so101_pos, so101_home, fanuc_home)

using MuJoCo
using MuJoCo.LibMuJoCo
using LinearAlgebra

"""
    IKConfig

Configuration parameters for the inverse kinematics solver.

# Fields
- `max_iter::Int`: Maximum iterations (default 100)
- `tol::Float64`: Position tolerance in meters (default 0.001 = 1mm)
- `step_size::Float64`: Step size for joint updates (default 0.5)
- `damping::Float64`: Damping factor λ for DLS (default 0.01)

# Notes
- Increase `damping` if the solver is unstable near singularities
- Decrease `step_size` if the solver oscillates
- Increase `max_iter` if the solver doesn't converge in time

# Future: Orientation matching
To add orientation matching later, extend this struct with:
- `orientation_weight::Float64`: Weight for orientation error vs position
- `orientation_tol::Float64`: Orientation tolerance in radians
"""
Base.@kwdef struct IKConfig
    max_iter::Int = 100
    tol::Float64 = 0.001
    step_size::Float64 = 0.5
    damping::Float64 = 0.01
end

"""
    IKResult

Result of an inverse kinematics solve.

# Fields
- `success::Bool`: Whether the solver converged within tolerance
- `iterations::Int`: Number of iterations used
- `final_error::Float64`: Final position error in meters
- `position::Vector{Float64}`: Final end-effector position achieved

# Future: Orientation matching
To add orientation matching later, extend this struct with:
- `orientation_error::Float64`: Final orientation error in radians
- `orientation::Vector{Float64}`: Final end-effector orientation (quaternion)
"""
struct IKResult
    success::Bool
    iterations::Int
    final_error::Float64
    position::Vector{Float64}
end

"""
    get_body_id(model, body_name::String) -> Int

Get the MuJoCo body ID for a named body.
Returns -1 if body not found.
"""
function get_body_id(model, body_name::String)
    return mj_name2id(model, Int32(LibMuJoCo.mjOBJ_BODY), body_name)
end

"""
    forward_kinematics(model, data, body_name::String) -> Vector{Float64}

Get the world-frame position of a body's origin.

Runs `mj_forward()` to update kinematics, then returns the 3D position.

# Arguments
- `model`: MuJoCo Model
- `data`: MuJoCo Data (with current qpos set)
- `body_name`: Name of the body (e.g., "gripper", "link_6")

# Returns
- 3-element Vector{Float64} with [x, y, z] position in meters

# Example
```julia
data.qpos[1:6] .= joint_angles
pos = forward_kinematics(model, data, "gripper")
```

# Future: Orientation
To get orientation as well, access `data.xquat[body_id+1, :]` after mj_forward().
"""
function forward_kinematics(model, data, body_name::String)
    body_id = get_body_id(model, body_name)
    if body_id < 0
        error("Body not found: $body_name")
    end

    # Update forward kinematics
    mj_forward(model, data)

    # Return position (Julia 1-indexed, body_id is 0-indexed)
    return Vector{Float64}(data.xpos[body_id + 1, :])
end

"""
    get_body_jacobian(model, data, body_name::String) -> (jacp, jacr)

Compute the body Jacobian for position and rotation.

# Arguments
- `model`: MuJoCo Model
- `data`: MuJoCo Data (with current qpos set)
- `body_name`: Name of the body

# Returns
- `jacp`: 3×nv position Jacobian (∂position/∂qpos)
- `jacr`: 3×nv rotation Jacobian (∂rotation/∂qpos)

# Notes
The Jacobians relate joint velocities to end-effector velocities:
- ẋ = jacp * q̇ (linear velocity)
- ω = jacr * q̇ (angular velocity)

For IK, we use the position Jacobian to compute joint updates:
- Δq = J⁺ Δx (where J⁺ is the pseudoinverse or damped inverse)

# Future: Orientation matching
The rotation Jacobian `jacr` can be used for full 6-DOF IK by stacking:
```julia
J = [jacp; jacr]  # 6×nv Jacobian
error = [pos_error; orient_error]  # 6-element error vector
```
"""
function get_body_jacobian(model, data, body_name::String)
    body_id = get_body_id(model, body_name)
    if body_id < 0
        error("Body not found: $body_name")
    end

    # Allocate Jacobian matrices (3 x nv) using mj_zeros for row-major layout
    # MuJoCo expects row-major arrays; mj_zeros returns a Transpose wrapper
    # that stores data in the correct order for MuJoCo while appearing column-major to Julia
    nv = model.nv
    jacp = mj_zeros(3, nv)
    jacr = mj_zeros(3, nv)

    # Compute Jacobians at body origin
    mj_jacBody(model, data, jacp, jacr, body_id)

    # Convert to regular Matrix for easier manipulation
    return Matrix(jacp), Matrix(jacr)
end

"""
    get_joint_limits(model) -> (lower, upper)

Get joint position limits from the model.

# Returns
- `lower`: Vector of lower limits (radians for hinge joints)
- `upper`: Vector of upper limits (radians for hinge joints)

# Notes
Uses `model.jnt_range` which has shape (njnt, 2).
For joints without limits, returns -Inf/+Inf.
"""
function get_joint_limits(model)
    nq = model.nq
    lower = fill(-Inf, nq)
    upper = fill(+Inf, nq)

    for i in 0:(model.njnt - 1)
        # Check if joint has limits (jnt_limited is a boolean array)
        if model.jnt_limited[i + 1] != 0
            addr = model.jnt_qposadr[i + 1] + 1  # 1-indexed
            lower[addr] = model.jnt_range[i + 1, 1]
            upper[addr] = model.jnt_range[i + 1, 2]
        end
    end

    return lower, upper
end

"""
    clamp_to_limits!(qpos, lower, upper; warn_threshold=0.01)

Clamp joint positions to limits, warning if clamping is significant.

# Arguments
- `qpos`: Joint positions to clamp (modified in place)
- `lower`: Lower limits
- `upper`: Upper limits
- `warn_threshold`: Warn if clamping exceeds this amount (radians, default 0.01)

# Returns
- `true` if any significant clamping occurred, `false` otherwise
"""
function clamp_to_limits!(qpos, lower, upper; warn_threshold = 0.01)
    clamped = false
    for i in eachindex(qpos)
        if qpos[i] < lower[i]
            if lower[i] - qpos[i] > warn_threshold
                @warn "Joint $i clamped to lower limit" value=qpos[i] limit=lower[i]
                clamped = true
            end
            qpos[i] = lower[i]
        elseif qpos[i] > upper[i]
            if qpos[i] - upper[i] > warn_threshold
                @warn "Joint $i clamped to upper limit" value=qpos[i] limit=upper[i]
                clamped = true
            end
            qpos[i] = upper[i]
        end
    end
    return clamped
end

"""
    inverse_kinematics!(model, data, body_name::String, target_pos::Vector{Float64};
                        config::IKConfig=IKConfig()) -> IKResult

Solve inverse kinematics using Damped Least Squares (DLS).

Iteratively adjusts joint positions to move the specified body toward
the target position. Modifies `data.qpos` in place.

# Algorithm
1. Compute current end-effector position via FK
2. Compute position error = target - current
3. If error < tolerance, return success
4. Compute position Jacobian J (3×nv)
5. Compute damped least squares: Δq = Jᵀ(JJᵀ + λI)⁻¹ error
6. Update qpos with step_size * Δq
7. Clamp to joint limits (with warning)
8. Repeat until converged or max iterations

# Arguments
- `model`: MuJoCo Model
- `data`: MuJoCo Data (qpos will be modified)
- `body_name`: Name of the end-effector body
- `target_pos`: Target [x, y, z] position in world frame
- `config`: IKConfig with solver parameters

# Returns
- `IKResult` with success status, iterations, final error, and achieved position

# Example
```julia
result = inverse_kinematics!(model, data, "link_6", [0.5, 0.0, 0.8])
if result.success
    println("IK converged in \$(result.iterations) iterations")
else
    println("IK failed, error: \$(result.final_error)m")
end
```

# Future: Orientation matching
To add 6-DOF IK (position + orientation):
1. Accept optional `target_quat::Vector{Float64}` parameter
2. Compute orientation error as axis-angle from current to target
3. Stack position and orientation Jacobians: J = [Jp; Jr]
4. Stack errors: e = [pos_error; orientation_weight * orient_error]
5. Solve as before with the 6×nv Jacobian
"""
function inverse_kinematics!(model, data, body_name::String, target_pos::Vector{Float64};
        config::IKConfig = IKConfig())
    body_id = get_body_id(model, body_name)
    if body_id < 0
        error("Body not found: $body_name")
    end

    # Get joint limits for clamping
    lower, upper = get_joint_limits(model)
    nv = model.nv

    # Identity matrix for damping
    I_mat = Matrix{Float64}(I, 3, 3)

    for iter in 1:(config.max_iter)
        # Update forward kinematics
        mj_forward(model, data)

        # Get current position
        current_pos = Vector{Float64}(data.xpos[body_id + 1, :])

        # Compute error
        error_vec = target_pos - current_pos
        error_norm = norm(error_vec)

        # Check convergence
        if error_norm < config.tol
            return IKResult(true, iter, error_norm, current_pos)
        end

        # Compute Jacobian using mj_zeros for correct row-major layout
        jacp = mj_zeros(3, nv)
        jacr = mj_zeros(3, nv)
        mj_jacBody(model, data, jacp, jacr, body_id)

        # Convert to Matrix for linear algebra operations
        J = Matrix(jacp)

        # Damped Least Squares: Δq = Jᵀ(JJᵀ + λI)⁻¹ error
        # This is more stable than pseudoinverse near singularities
        JJT = J * J'
        damped_inv = (JJT + config.damping * I_mat) \ error_vec
        delta_q = J' * damped_inv

        # Update joint positions with step size
        data.qpos[1:nv] .+= config.step_size * delta_q

        # Clamp to joint limits
        clamp_to_limits!(view(data.qpos, 1:nv), lower, upper)
    end

    # Did not converge - return best solution
    mj_forward(model, data)
    final_pos = Vector{Float64}(data.xpos[body_id + 1, :])
    final_error = norm(target_pos - final_pos)

    @warn "IK did not converge" target=target_pos achieved=final_pos error=final_error

    return IKResult(false, config.max_iter, final_error, final_pos)
end

"""
    get_home_position(model, data, body_name::String) -> Vector{Float64}

Get the end-effector position when all joints are at zero (home position).

Creates a temporary copy of data to avoid modifying the current state.

# Arguments
- `model`: MuJoCo Model
- `data`: MuJoCo Data
- `body_name`: Name of the end-effector body

# Returns
- 3-element Vector{Float64} with [x, y, z] home position

# Example
```julia
so101_home = get_home_position(so101_model, so101_data, "gripper")
fanuc_home = get_home_position(fanuc_model, fanuc_data, "link_6")
```
"""
function get_home_position(model, data, body_name::String)
    # Create temporary data to avoid modifying current state
    temp_data = init_data(model)

    # Zero all joint positions
    temp_data.qpos .= 0.0

    # Get position at home
    return forward_kinematics(model, temp_data, body_name)
end

"""
    scale_position(pos::Vector{Float64}, source_home::Vector{Float64}, 
                   target_home::Vector{Float64}) -> Vector{Float64}

Scale a position from one robot's workspace to another's.

Uses linear scaling based on the distance from origin at home position.
The scaling factor is: norm(target_home) / norm(source_home)

# Arguments
- `pos`: Position in source robot's workspace
- `source_home`: Home position of source robot's end-effector
- `target_home`: Home position of target robot's end-effector

# Returns
- Scaled position in target robot's workspace

# Algorithm
```
offset = pos - source_home           # Offset from source home
scale = norm(target_home) / norm(source_home)
target = target_home + offset * scale
```

# Example
```julia
# SO101 gripper is at [0.29, 0.0, 0.23], Fanuc is at [0.89, 0.0, 1.25]
so101_home = [0.29, 0.0, 0.23]
fanuc_home = [0.89, 0.0, 1.25]

# Map SO101 position to Fanuc workspace
so101_pos = [0.35, 0.1, 0.30]
fanuc_target = scale_position(so101_pos, so101_home, fanuc_home)
```
"""
function scale_position(pos::Vector{Float64}, source_home::Vector{Float64},
        target_home::Vector{Float64})
    # Compute scale factor based on home position distances from origin
    source_dist = norm(source_home)
    target_dist = norm(target_home)

    if source_dist < 1e-6
        error("Source home position is too close to origin")
    end

    scale = target_dist / source_dist

    # Offset from source home, scaled, then added to target home
    offset = pos - source_home
    return target_home + offset * scale
end

"""
    RobotKinematicsCache

Pre-computed kinematics data for a robot, used for efficient cross-robot mapping.

# Fields
- `model`: MuJoCo Model
- `data`: MuJoCo Data
- `ee_body::String`: End-effector body name
- `home_pos::Vector{Float64}`: End-effector position at qpos=0
- `joint_names::Vector{String}`: Names of robot joints
- `joint_map::Dict{String,Int}`: Joint name to qpos index mapping

# Future: Orientation matching
Add `home_quat::Vector{Float64}` for home orientation.
"""
mutable struct RobotKinematicsCache
    model::Any  # Model type varies
    data::Any   # Data type varies
    ee_body::String
    home_pos::Vector{Float64}
    joint_names::Vector{String}
    joint_map::Dict{String, Int}
end

"""
    build_kinematics_cache(xml_path::String, ee_body::String, 
                           joint_prefix::String="") -> RobotKinematicsCache

Load a robot model and pre-compute kinematics data.

# Arguments
- `xml_path`: Path to robot MJCF XML file
- `ee_body`: Name of end-effector body
- `joint_prefix`: Optional prefix to filter joints (e.g., "joint_" for Fanuc)

# Returns
- `RobotKinematicsCache` with model, data, and pre-computed values

# Example
```julia
so101 = build_kinematics_cache(
    "robots/SO-ARM100/Simulation/SO101/so101_new_calib.xml",
    "gripper"
)

fanuc = build_kinematics_cache(
    "robots/fanuc_mujoco/m10ia/scene.xml",
    "link_6",
    "joint_"
)
```
"""
function build_kinematics_cache(xml_path::String, ee_body::String;
        joint_prefix::String = "")
    model = load_model(xml_path)
    data = init_data(model)

    # Get home position
    home_pos = get_home_position(model, data, ee_body)

    # Build joint name list and mapping
    joint_names = String[]
    joint_map = Dict{String, Int}()

    for i in 0:(model.njnt - 1)
        name = unsafe_string(mj_id2name(model, Int32(LibMuJoCo.mjOBJ_JOINT), i))
        if !isempty(name)
            if isempty(joint_prefix) || startswith(name, joint_prefix)
                push!(joint_names, name)
                # Map to qpos address (1-indexed)
                addr = model.jnt_qposadr[i + 1] + 1
                joint_map[name] = addr
            end
        end
    end

    return RobotKinematicsCache(model, data, ee_body, home_pos, joint_names, joint_map)
end

"""
    apply_joints!(cache::RobotKinematicsCache, joints::Dict{String,<:Real})

Apply joint positions to a robot's data.

# Arguments
- `cache`: RobotKinematicsCache for the robot
- `joints`: Dict of joint_name => value (in radians)

Unrecognized joint names are silently ignored.
"""
function apply_joints!(cache::RobotKinematicsCache, joints::Dict{String, <:Real})
    for (name, val) in joints
        if haskey(cache.joint_map, name)
            idx = cache.joint_map[name]
            cache.data.qpos[idx] = Float64(val)
        end
    end
end
