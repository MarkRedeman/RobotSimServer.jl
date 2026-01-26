# Geti Action MuJoCo Simulation Example

A proof-of-concept demonstrating how to integrate MuJoCo robot arm simulations with [Geti Action](https://geti.ai).

## What is Geti Action?

Geti Action is a local desktop application for robotics researchers and developers to:

1. **Collect teleoperation data** on their own hardware
2. **Train VLA (Vision-Language-Action) models** locally on their machine
3. **Deploy optimized models** to robots using Intel silicon

Think "Stable Diffusion WebUI" but for robot training.

## What is this repository?

This repository provides example MuJoCo simulations that can be controlled via WebSocket, allowing Geti Action to:

- Send joint commands to simulated robot arms
- Receive real-time joint state feedback
- Stream camera feeds from multiple viewpoints (including gripper-mounted cameras)

## Features

- **Multi-robot support**: SO101, Trossen WXAI, Franka Panda, and 19 Fanuc industrial robot families (33 variants)
- **WebSocket control interface**: JSON-based protocol on port 8081
- **ZMQ control interface**: Alternative REQ/REP protocol on port 5555
- **Multi-camera capture system**:
  - WebSocket streaming (multiple cameras, multiple ports)
  - Video file output (FFMPEG-based, crash-safe)
  - Image sequence output (JPEG/PNG)
- **Scene builder**: Programmatically add graspable objects, body-mounted cameras, and collision primitives
- **Real-time state broadcasting**: 30fps joint state updates to all connected clients

## Quick Start

### Using Mise (Recommended)

This project uses [mise](https://mise.jdx.dev) for tool version management, environment configuration, and task running.

```bash
# Clone with submodules
git clone --recursive git@github.com:MarkRedeman/geti-action-mujoco-sim.git
cd geti-action-mujoco-sim

# Trust and install tools (installs Julia 1.12.4)
mise trust && mise install

# Install dependencies and MuJoCo visualizer
mise run setup

# Run SO101 with WebSocket control
mise run so101

# Run Trossen WXAI with WebSocket control
mise run trossen
```

**Available tasks:**

| Task | Description |
|------|-------------|
| `mise run setup` | Install dependencies and MuJoCo visualizer |
| `mise run so101` | Run SO101 robot with WebSocket control |
| `mise run trossen` | Run Trossen robot with WebSocket control |
| `mise run franka` | Run Franka Panda with WebSocket control |
| `mise run lekiwi` | Run LeKiwi mobile robot with WebSocket arm control |
| `mise run fanuc` | Run Fanuc robot with WebSocket control |
| `mise run so101-basic` | Run SO101 basic demo (no WebSocket) |
| `mise run trossen-basic` | Run Trossen basic demo (no WebSocket) |
| `mise run franka-basic` | Run Franka Panda basic demo (no WebSocket) |
| `mise run lekiwi-basic` | Run LeKiwi basic demo (drives in circles) |
| `mise run fanuc-basic` | Run Fanuc basic demo (no WebSocket) |
| `mise run format` | Format all Julia code |
| `mise run client` | Run WebSocket test client |

### Alternative (Without Mise)

If you prefer not to use mise, ensure [Julia](https://julialang.org/downloads/) is installed:

```bash
# Clone with submodules
git clone --recursive git@github.com:MarkRedeman/geti-action-mujoco-sim.git
cd geti-action-mujoco-sim

# Install dependencies
julia --project=. -e 'using Pkg; Pkg.instantiate()'

# Install MuJoCo visualizer (first time only)
julia --project=. -e 'using MuJoCo; install_visualiser()'

# Run SO101 with WebSocket control
julia --project=. -t 4 examples/so101/websocket_sim.jl

# Run Trossen WXAI with WebSocket control
julia --project=. -t 4 examples/trossen/websocket_sim.jl
```

> **Note**: The `-t 4` flag enables multi-threading. With mise, `JULIA_NUM_THREADS=auto` is set automatically.

## Project Structure

```
.
├── src/                           # Reusable library code
│   ├── SceneBuilder.jl            # Add objects, cameras, collisions to scenes
│   ├── WebSocketServer.jl         # Shared WebSocket control server logic
│   └── capture/                   # Multi-camera capture system
│       ├── Capture.jl             # Main capture module
│       ├── types.jl               # Type definitions
│       ├── manager.jl             # Capture orchestration
│       ├── worker.jl              # Async I/O worker
│       └── backends/              # Output backends
│           ├── file.jl            # Save frames to disk
│           ├── video.jl           # FFMPEG video encoding
│           └── websocket.jl       # Stream to WebSocket clients
│
├── examples/                      # Runnable simulation scripts
│   ├── so101/                     # SO101 robot arm examples
│   │   ├── basic_sim.jl           # Simple sine wave demo
│   │   ├── websocket_sim.jl       # Full WebSocket + cameras
│   │   ├── headless_sim.jl        # No GUI, WebSocket only
│   │   └── zmq_sim.jl             # ZMQ control interface
│   ├── trossen/                   # Trossen WXAI examples
│   │   ├── basic_sim.jl           # Simple sine wave demo
│   │   └── websocket_sim.jl       # Full WebSocket + cameras
│   ├── franka/                    # Franka Panda examples
│   │   ├── basic_sim.jl           # Simple sine wave demo
│   │   └── websocket_sim.jl       # Full WebSocket + cameras + IK
│   ├── fanuc/                     # Fanuc industrial robot examples
│   │   └── basic_sim.jl           # Multi-robot demo (19+ robots)
│   └── clients/                   # Test client examples
│       ├── ws_client.jl           # Julia WebSocket client
│       ├── zmq_client.jl          # Julia ZMQ client
│       └── zmq_client.py          # Python ZMQ client
│
├── robots/                        # Robot model submodules
│   ├── SO-ARM100/                 # SO101 robot (git submodule)
│   ├── trossen_arm_mujoco/        # Trossen WXAI (git submodule)
│   ├── franka/                    # Franka Panda (git submodule)
│   ├── google-deepmind/           # MuJoCo Menagerie models (git submodule)
│   ├── fanuc-industrial/          # ROS-Industrial Fanuc (git submodule)
│   └── fanuc_mujoco/              # Generated MuJoCo XMLs for Fanuc
│
├── scripts/                       # Utility scripts
│   └── convert_fanuc_industrial.py  # URDF to MuJoCo converter
│
├── Project.toml                   # Julia project dependencies
├── Manifest.toml                  # Locked dependency versions
└── mise.toml                      # Mise configuration (tools, env, tasks)
```

## Supported Robots

### SO101 (SO-ARM100)

A 6-DOF desktop robot arm with:
- **Joints**: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
- **Control**: Position control in degrees
- **Gripper**: -10° (closed) to 100° (open)

### Trossen WXAI

A 6-DOF research robot arm with:
- **Joints**: joint_0 through joint_5, left_gripper
- **SO101 Compatibility**: Accepts SO101 joint names, maps internally
- **Gripper**: Slide joint (0 to 0.044 meters)

### Franka Panda

A 7-DOF research/industrial robot arm with parallel-jaw gripper:
- **Joints**: joint1 through joint7
- **Control**: IK-based mapping from SO101 commands
- **Gripper**: 0-255 actuator range (0-0.04m opening)
- **State reporting**: 7 joints mapped to SO101-compatible names + extra DOFs (wrist_yaw, wrist_twist)
- **Model**: Uses MuJoCo Menagerie franka_emika_panda

### Fanuc Industrial Robots

19 industrial robot families from the [ROS-Industrial fanuc](https://github.com/ros-industrial/fanuc) repository:

| Family | Default Variant | Joints | Description |
|--------|-----------------|--------|-------------|
| CR-7iA | cr7ia | 6 | Collaborative robot |
| CR-35iA | cr35ia | 6 | Heavy payload collaborative |
| CRX-10iA/L | crx10ial | 6 | Lightweight collaborative |
| LRMate 200i | lrmate200i | 6 | Compact industrial |
| LRMate 200iB | lrmate200ib | 6 | Compact industrial |
| LRMate 200iC | lrmate200ic | 6 | Compact industrial |
| LRMate 200iD | lrmate200id | 6 | Compact industrial |
| M-6iB | m6ib | 6 | Small industrial |
| M-10iA | m10ia | 6 | Medium industrial |
| M-16iB | m16ib20 | 6 | Medium industrial |
| M-20iA | m20ia | 6 | Medium industrial |
| M-20iB | m20ib25 | 6 | Medium industrial |
| M-430iA | m430ia2f | 5 | Delta/SCARA style |
| M-710iC | m710ic50 | 6 | Large industrial |
| M-900iA | m900ia260l | 8 | Heavy payload |
| M-900iB | m900ib700 | 12 | Extra heavy payload |
| R-1000iA | r1000ia80f | 6 | High-speed spot welding |
| R-2000iB | r2000ib210f | 6 | Heavy payload |
| R-2000iC | r2000ic165f | 6 | Heavy payload |

Additional variants available (33 total). Run `python3 scripts/convert_fanuc_industrial.py --list` to see all.

## Running Examples

### SO101 Examples

```bash
# Basic demo - arm moves in sine wave pattern
julia --project=. examples/so101/basic_sim.jl

# WebSocket control with multi-camera capture
julia --project=. -t 4 examples/so101/websocket_sim.jl

# ZMQ control (alternative protocol)
julia --project=. examples/so101/zmq_sim.jl

# Headless mode (no GUI window)
julia --project=. examples/so101/headless_sim.jl
```

### Trossen Examples

```bash
# Basic demo
julia --project=. examples/trossen/basic_sim.jl

# WebSocket control with multi-camera capture
julia --project=. -t 4 examples/trossen/websocket_sim.jl
```

### Franka Examples

```bash
# Basic demo
julia --project=. examples/franka/basic_sim.jl

# WebSocket control with IK-based mapping
julia --project=. -t 4 examples/franka/websocket_sim.jl
```

### Fanuc Examples

```bash
# Run with default robot (M-10iA - classic yellow industrial arm)
julia --project=. examples/fanuc/basic_sim.jl

# Run with specific robot
julia --project=. examples/fanuc/basic_sim.jl crx10ial
julia --project=. examples/fanuc/basic_sim.jl r2000ic165f
julia --project=. examples/fanuc/basic_sim.jl m900ib700

# Using mise
mise run fanuc

# Generate additional variants
python3 scripts/convert_fanuc_industrial.py m10ia7l crx10ial
```

### Client Examples

```bash
# Julia WebSocket client (sweeping motion demo)
julia --project=. examples/clients/ws_client.jl

# Julia ZMQ client
julia --project=. examples/clients/zmq_client.jl

# Python ZMQ client (requires pyzmq)
python examples/clients/zmq_client.py
```

## WebSocket API

### Connection

Connect to `ws://localhost:8081` for joint control.

### Commands

#### Set Joint Positions

```json
{
    "command": "set_joints_state",
    "joints": {
        "shoulder_pan": 45.0,
        "shoulder_lift": 30.0,
        "elbow_flex": -20.0,
        "wrist_flex": 0.0,
        "wrist_roll": 0.0,
        "gripper": 50.0
    }
}
```

All joint values are in **degrees**.

#### Ping

```json
{
    "command": "ping"
}
```

Response:

```json
{
    "event": "pong",
    "timestamp": 1234567890.123
}
```

### Events (Server → Client)

#### State Update

Broadcast at 30fps when joint state changes:

```json
{
    "event": "state_was_updated",
    "timestamp": 1234567890.123,
    "state": {
        "shoulder_pan": 45.0,
        "shoulder_lift": 30.0,
        "elbow_flex": -20.0,
        "wrist_flex": 0.0,
        "wrist_roll": 0.0,
        "gripper": 50.0
    },
    "is_controlled": false
}
```

## Camera Streaming

When running WebSocket simulations, camera feeds are available on separate ports:

| Camera | Port | Description |
|--------|------|-------------|
| Front | 8082 | External view from front |
| Side | 8083 | External view from side |
| Orbit | 8084 | Rotating external view |
| Gripper | 8085 | First-person gripper view |
| Wrist | 8086 | Wrist-mounted camera (Franka only) |

Each port streams raw JPEG frames over WebSocket.

> **Note**: Franka Panda uses 5 cameras (ports 8082-8086) including both wrist and gripper cameras.

## Scene Builder

The `SceneBuilder` module allows programmatic scene modification:

### Adding Graspable Cubes

```julia
include("src/SceneBuilder.jl")

cubes = [
    CubeSpec(name="red_cube", pos=[0.15, 0.0, 0.025], color=[0.9, 0.2, 0.2, 1.0]),
    CubeSpec(name="green_cube", pos=[0.15, 0.08, 0.025], color=[0.2, 0.9, 0.2, 1.0]),
]

model, data = build_scene("path/to/scene.xml", cubes)
```

### Adding Body-Mounted Cameras

```julia
gripper_camera = BodyCamera(
    name = "gripper_cam",
    body = "gripper",
    pos = [0.0, 0.0, -0.04],
    quat = [1.0, 0.0, 0.0, 0.0],
    fovy = 90.0
)

model, data = build_scene("scene.xml", cubes, cameras=[gripper_camera])
```

### Adding Collision Primitives

```julia
collisions = default_gripper_collisions()  # Pre-configured for SO101 gripper
model, data = build_scene("scene.xml", cubes, collisions=collisions)
```

## Capture System

The capture system supports multiple output backends:

```julia
include("src/capture/Capture.jl")

config = CaptureConfig(
    width = 640,
    height = 480,
    fps = 30.0,
    cameras = [
        # Save to video file
        CameraSpec(name="front", azimuth=180.0, output=VideoOutput("output/front.mp4")),
        
        # Stream via WebSocket
        CameraSpec(name="side", azimuth=90.0, output=WebSocketOutput(port=8082)),
        
        # Save individual frames
        CameraSpec(name="top", elevation=-90.0, output=FileOutput("output/frames")),
        
        # Body-mounted camera
        CameraSpec(name="gripper", mode=:fixed, model_camera="gripper_cam",
                   output=WebSocketOutput(port=8085)),
    ]
)

run_with_capture!(model, data, controller=ctrl!, capture=config)
```

## Geti Action Integration

To connect Geti Action to this simulation:

1. Start a WebSocket simulation:
   ```bash
   julia --project=. -t 4 examples/so101/websocket_sim.jl
   ```

2. In Geti Action, configure the connection:
   - **Control endpoint**: `ws://localhost:8081`
   - **Camera streams**: `ws://localhost:8082` (front), `ws://localhost:8085` (gripper), etc.

3. Geti Action can now:
   - Send teleoperation commands from a leader arm
   - Record synchronized camera feeds
   - Collect training data from the simulated environment

## Dependencies

- [MuJoCo.jl](https://github.com/JamieMair/MuJoCo.jl) - MuJoCo physics simulation
- [HTTP.jl](https://github.com/JuliaWeb/HTTP.jl) - WebSocket server
- [JSON.jl](https://github.com/JuliaIO/JSON.jl) - JSON parsing
- [ZMQ.jl](https://github.com/JuliaInterop/ZMQ.jl) - ZeroMQ bindings
- [Images.jl](https://github.com/JuliaImages/Images.jl) - Image processing
- [FileIO.jl](https://github.com/JuliaIO/FileIO.jl) - File I/O

## License

See individual robot model licenses in their respective submodule directories.
