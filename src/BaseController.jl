# Base Velocity Controller - Unified control for mobile robot bases
#
# Supports multiple input sources with last-input-wins semantics:
# - Keyboard (WASD + QE for strafing, Shift for speed boost)
# - WebSocket commands
# - Timeout to zero velocity when no input received
#
# Usage:
#   include("../../src/BaseController.jl")
#   
#   base_ctrl = BaseVelocityController(
#       max_vx = 0.15,      # Max forward/back speed (m/s)
#       max_vy = 0.10,      # Max strafe speed (m/s)
#       max_omega = 0.8,    # Max angular velocity (rad/s)
#       timeout = 0.5       # Seconds before velocities reset to 0
#   )
#   
#   # In keyboard handler (called from UI loop):
#   update_from_keyboard!(base_ctrl, window, GLFW)
#   
#   # In controller (called from physics loop):
#   vx, vy, omega = get_velocities(base_ctrl)
#   wheel_vels = body_to_wheel_velocities(vx, vy, omega)
#
# Keyboard Controls:
#   W/S - Forward/backward (vx)
#   A/D - Rotate left/right (omega)  
#   Q/E - Strafe left/right (vy)
#   Shift - Double speed

"""
    BaseVelocityController

Manages velocity commands for a mobile robot base from multiple input sources.

Supports keyboard (WASD+QE) and WebSocket control with automatic timeout
to zero velocity when no input is received.

# Fields
- `vx`, `vy`, `omega`: Current velocity commands
- `max_vx`, `max_vy`, `max_omega`: Maximum velocities (before speed multiplier)
- `speed_multiplier`: Current speed multiplier (1.0 normal, 2.0 with Shift)
- `last_update`: Time of last velocity update
- `timeout`: Seconds after which velocities reset to zero
- `source`: Symbol indicating last input source (:keyboard, :websocket, :none)
"""
mutable struct BaseVelocityController
    # Current velocities
    vx::Float64           # Forward/back (m/s), positive = forward
    vy::Float64           # Strafe (m/s), positive = left
    omega::Float64        # Angular (rad/s), positive = counter-clockwise

    # Speed limits (before multiplier)
    max_vx::Float64
    max_vy::Float64
    max_omega::Float64

    # Speed multiplier (Shift = 2x)
    speed_multiplier::Float64

    # Timeout handling
    last_update::Float64
    timeout::Float64

    # Source tracking
    source::Symbol
end

"""
    BaseVelocityController(; max_vx=0.15, max_vy=0.10, max_omega=0.8, timeout=0.5)

Create a new base velocity controller with specified limits.

# Arguments
- `max_vx`: Maximum forward/backward speed in m/s (default 0.15)
- `max_vy`: Maximum strafe speed in m/s (default 0.10)
- `max_omega`: Maximum angular velocity in rad/s (default 0.8)
- `timeout`: Seconds of no input before velocities reset to zero (default 0.5)
"""
function BaseVelocityController(;
        max_vx::Float64 = 0.15,
        max_vy::Float64 = 0.10,
        max_omega::Float64 = 0.8,
        timeout::Float64 = 0.5)
    return BaseVelocityController(
        0.0, 0.0, 0.0,            # velocities start at zero
        max_vx, max_vy, max_omega,
        1.0,                       # speed_multiplier
        0.0, timeout,              # last_update, timeout
        :none                      # source
    )
end

"""
    update_from_keyboard!(ctrl, window, GLFW)

Poll WASD/QE keys and update velocities based on current key state.
Call this once per frame from the UI loop (after GLFW.PollEvents).

# Keyboard Mapping
- `W` / `S`: Forward / backward (affects vx)
- `A` / `D`: Rotate left / right (affects omega)
- `Q` / `E`: Strafe left / right (affects vy)
- `Shift`: Double all speeds

# Arguments
- `ctrl`: BaseVelocityController to update
- `window`: GLFW window handle
- `GLFW`: GLFW module (passed to avoid import issues)
"""
function update_from_keyboard!(ctrl::BaseVelocityController, window, GLFW)
    # Check shift for speed boost
    shift = GLFW.GetKey(window, GLFW.KEY_LEFT_SHIFT) == GLFW.PRESS ||
            GLFW.GetKey(window, GLFW.KEY_RIGHT_SHIFT) == GLFW.PRESS

    ctrl.speed_multiplier = shift ? 2.0 : 1.0

    # Poll movement keys
    w_pressed = GLFW.GetKey(window, GLFW.KEY_W) == GLFW.PRESS
    s_pressed = GLFW.GetKey(window, GLFW.KEY_S) == GLFW.PRESS
    a_pressed = GLFW.GetKey(window, GLFW.KEY_A) == GLFW.PRESS
    d_pressed = GLFW.GetKey(window, GLFW.KEY_D) == GLFW.PRESS
    q_pressed = GLFW.GetKey(window, GLFW.KEY_Q) == GLFW.PRESS
    e_pressed = GLFW.GetKey(window, GLFW.KEY_E) == GLFW.PRESS

    any_key = w_pressed || s_pressed || a_pressed || d_pressed || q_pressed || e_pressed

    if any_key
        # Forward/backward (W/S)
        vx = 0.0
        if w_pressed
            vx += ctrl.max_vx
        end
        if s_pressed
            vx -= ctrl.max_vx
        end

        # Rotation (A/D) - A = rotate left (positive omega)
        omega = 0.0
        if a_pressed
            omega += ctrl.max_omega
        end
        if d_pressed
            omega -= ctrl.max_omega
        end

        # Strafe (Q/E) - Q = strafe left (positive vy)
        vy = 0.0
        if q_pressed
            vy += ctrl.max_vy
        end
        if e_pressed
            vy -= ctrl.max_vy
        end

        # Apply speed multiplier
        ctrl.vx = vx * ctrl.speed_multiplier
        ctrl.vy = vy * ctrl.speed_multiplier
        ctrl.omega = omega * ctrl.speed_multiplier

        ctrl.last_update = time()
        ctrl.source = :keyboard
    end
end

"""
    update_from_websocket!(ctrl, vx, vy, omega)

Set velocities from a WebSocket command. Values are clamped to 2x max velocities.

# Arguments
- `ctrl`: BaseVelocityController to update
- `vx`: Forward/backward velocity in m/s
- `vy`: Strafe velocity in m/s
- `omega`: Angular velocity in rad/s
"""
function update_from_websocket!(ctrl::BaseVelocityController, vx::Float64, vy::Float64,
        omega::Float64)
    # Clamp to 2x max (allowing some headroom for boosted speeds)
    ctrl.vx = clamp(vx, -ctrl.max_vx * 2, ctrl.max_vx * 2)
    ctrl.vy = clamp(vy, -ctrl.max_vy * 2, ctrl.max_vy * 2)
    ctrl.omega = clamp(omega, -ctrl.max_omega * 2, ctrl.max_omega * 2)
    ctrl.last_update = time()
    ctrl.source = :websocket
end

"""
    get_velocities(ctrl) -> (vx, vy, omega)

Get current velocity commands, applying timeout if needed.

Returns `(0.0, 0.0, 0.0)` if no input has been received within the timeout period.
This ensures the robot stops when control input ceases.

# Returns
- `vx`: Forward/backward velocity in m/s
- `vy`: Strafe velocity in m/s  
- `omega`: Angular velocity in rad/s
"""
function get_velocities(ctrl::BaseVelocityController)
    if time() - ctrl.last_update > ctrl.timeout
        # Timeout expired - stop the robot
        return (0.0, 0.0, 0.0)
    end
    return (ctrl.vx, ctrl.vy, ctrl.omega)
end

"""
    is_active(ctrl) -> Bool

Check if the controller has received recent input (within timeout).
"""
function is_active(ctrl::BaseVelocityController)
    return time() - ctrl.last_update <= ctrl.timeout
end

"""
    get_source(ctrl) -> Symbol

Get the source of the last velocity update.
Returns `:keyboard`, `:websocket`, or `:none`.
"""
function get_source(ctrl::BaseVelocityController)
    if !is_active(ctrl)
        return :none
    end
    return ctrl.source
end
