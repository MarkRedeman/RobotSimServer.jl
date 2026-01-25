import { useState, useEffect, useRef, useCallback } from 'react';
import useWebSocket, { ReadyState } from 'react-use-websocket';

// =============================================================================
// Constants (matching gamepad.html)
// =============================================================================
const WS_URL = 'ws://localhost:8081';
const SEND_RATE_MS = 33; // ~30Hz

// Velocity limits (matching BaseController.jl defaults)
const MAX_VX = 0.15; // m/s forward/back
const MAX_VY = 0.1; // m/s strafe
const MAX_OMEGA = 0.8; // rad/s rotation
const MAX_SPEED_BOOST = 5.0; // Maximum boost at full LT
const DEADZONE = 0.1;

// =============================================================================
// Types
// =============================================================================
export interface Commands {
  vx: number; // m/s forward/back
  vy: number; // m/s strafe
  omega: number; // rad/s rotation
  boostMultiplier: number; // 1.0 - MAX_SPEED_BOOST
}

export interface GamepadState {
  leftStick: { x: number; y: number };
  rightStick: { x: number; y: number };
  lt: number; // 0-1
  rt: number; // 0-1
  buttons: boolean[]; // 17 buttons
}

export interface RobotControlState {
  // Connection status
  gamepadConnected: boolean;
  gamepadName: string | null;
  wsConnected: boolean;

  // Current commands being sent (updated at 30Hz)
  commands: Commands;

  // Raw gamepad state (for UI visualization)
  gamepad: GamepadState | null;
}

// WebSocket command types
interface BaseVelocityCommand {
  command: 'set_base_velocity';
  vx: number;
  vy: number;
  omega: number;
}

// =============================================================================
// Utility Functions
// =============================================================================
function applyDeadzone(value: number, deadzone: number = DEADZONE): number {
  if (Math.abs(value) < deadzone) return 0;
  const sign = value > 0 ? 1 : -1;
  return (sign * (Math.abs(value) - deadzone)) / (1 - deadzone);
}

// =============================================================================
// Main Hook
// =============================================================================
export function useRobotControl(): RobotControlState {
  // ---------------------------------------------------------------------------
  // WebSocket setup
  // ---------------------------------------------------------------------------
  const { sendJsonMessage, readyState } = useWebSocket(WS_URL, {
    shouldReconnect: () => true,
    reconnectAttempts: 10,
    reconnectInterval: 3000,
  });

  const wsConnected = readyState === ReadyState.OPEN;

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------
  const [state, setState] = useState<RobotControlState>({
    gamepadConnected: false,
    gamepadName: null,
    wsConnected: false,
    commands: {
      vx: 0,
      vy: 0,
      omega: 0,
      boostMultiplier: 1.0,
    },
    gamepad: null,
  });

  // ---------------------------------------------------------------------------
  // Refs for animation loop
  // ---------------------------------------------------------------------------
  const animationRef = useRef<number | null>(null);
  const lastSendTimeRef = useRef<number>(0);
  const gamepadIndexRef = useRef<number | null>(null);

  // ---------------------------------------------------------------------------
  // Send command to robot
  // ---------------------------------------------------------------------------
  const sendCommand = useCallback(
    (commands: Commands) => {
      if (!wsConnected) return;

      const cmd: BaseVelocityCommand = {
        command: 'set_base_velocity',
        vx: commands.vx,
        vy: commands.vy,
        omega: commands.omega,
      };
      sendJsonMessage(cmd);
    },
    [wsConnected, sendJsonMessage]
  );

  // ---------------------------------------------------------------------------
  // Gamepad connection handlers
  // ---------------------------------------------------------------------------
  useEffect(() => {
    const handleConnect = (e: GamepadEvent) => {
      console.log('Gamepad connected:', e.gamepad.id);
      gamepadIndexRef.current = e.gamepad.index;
      setState((prev) => ({
        ...prev,
        gamepadConnected: true,
        gamepadName: e.gamepad.id,
      }));
    };

    const handleDisconnect = () => {
      console.log('Gamepad disconnected');
      gamepadIndexRef.current = null;
      setState((prev) => ({
        ...prev,
        gamepadConnected: false,
        gamepadName: null,
        gamepad: null,
        commands: {
          vx: 0,
          vy: 0,
          omega: 0,
          boostMultiplier: 1.0,
        },
      }));
    };

    // Check for existing gamepads on mount
    const checkExistingGamepads = () => {
      const gamepads = navigator.getGamepads();
      for (let i = 0; i < gamepads.length; i++) {
        const gp = gamepads[i];
        if (gp) {
          gamepadIndexRef.current = gp.index;
          setState((prev) => ({
            ...prev,
            gamepadConnected: true,
            gamepadName: gp.id,
          }));
          break;
        }
      }
    };

    window.addEventListener('gamepadconnected', handleConnect);
    window.addEventListener('gamepaddisconnected', handleDisconnect);
    checkExistingGamepads();

    return () => {
      window.removeEventListener('gamepadconnected', handleConnect);
      window.removeEventListener('gamepaddisconnected', handleDisconnect);
    };
  }, []);

  // ---------------------------------------------------------------------------
  // Main polling loop
  // ---------------------------------------------------------------------------
  useEffect(() => {
    const update = () => {
      const now = performance.now();
      const shouldUpdate = now - lastSendTimeRef.current >= SEND_RATE_MS;

      if (gamepadIndexRef.current !== null) {
        const gamepads = navigator.getGamepads();
        const gp = gamepads[gamepadIndexRef.current];

        if (gp) {
          // Read axes with deadzone
          const lx = applyDeadzone(gp.axes[0] || 0);
          const ly = applyDeadzone(gp.axes[1] || 0);
          const rx = applyDeadzone(gp.axes[2] || 0);
          const ry = applyDeadzone(gp.axes[3] || 0);

          // Read triggers
          const lt = gp.buttons[6] ? gp.buttons[6].value : 0;
          const rt = gp.buttons[7] ? gp.buttons[7].value : 0;

          // Read buttons
          const buttons = gp.buttons.map((b) => b.pressed);

          // LT for speed boost (quadratic scaling for fine control)
          const boostRange = MAX_SPEED_BOOST - 1.0;
          const boostMultiplier = 1.0 + lt * lt * boostRange;

          // Calculate robot commands
          // Left stick Y -> vx (stick up = -1, forward = +vx, so use ly directly after deadzone)
          // Left stick X -> vy (stick right = +1, strafe right = -vy, so invert)
          // Right stick X -> omega (stick right = +1, turn right = +omega)
          const commands: Commands = {
            vx: ly * MAX_VX * boostMultiplier,
            vy: -lx * MAX_VY * boostMultiplier,
            omega: rx * MAX_OMEGA * boostMultiplier,
            boostMultiplier,
          };

          // Only update state and send at throttled rate
          if (shouldUpdate) {
            lastSendTimeRef.current = now;

            const gamepadState: GamepadState = {
              leftStick: { x: lx, y: ly },
              rightStick: { x: rx, y: ry },
              lt,
              rt,
              buttons,
            };

            setState((prev) => ({
              ...prev,
              wsConnected,
              commands,
              gamepad: gamepadState,
            }));

            sendCommand(commands);
          }
        }
      } else if (shouldUpdate) {
        // No gamepad connected, just update wsConnected status
        lastSendTimeRef.current = now;
        setState((prev) => ({
          ...prev,
          wsConnected,
        }));
      }

      animationRef.current = requestAnimationFrame(update);
    };

    animationRef.current = requestAnimationFrame(update);

    return () => {
      if (animationRef.current !== null) {
        cancelAnimationFrame(animationRef.current);
      }
    };
  }, [wsConnected, sendCommand]);

  return state;
}

export default useRobotControl;
