# RobotSimServer.jl

> [!NOTE]
> This project started as an experiment in building a MuJoCo simulation
> in Julia with [opencode](https://opencode.ai/). It has since evolved
> into a server that lets clients control simulated robots via WebSocket,
> serving as a reference for integrating and testing new robots without
> needing physical hardware.

A multi-robot MuJoCo simulation server with WebSocket/ZMQ control, multi-camera streaming, and cross-robot teleoperation. Written in Julia.

## What is this repository?

This repository provides a simulation server for controlling robot arms via WebSocket or ZMQ. It supports multiple robot types with:

- Joint position control and real-time state feedback
- Multi-camera streaming (front, side, orbit, gripper, wrist views)
- Cross-robot teleoperation (e.g., SO101 leader controlling a Franka follower)
- URDF generation and mesh asset serving for web-based 3D visualization

## Features

- **Unified multi-robot server**: Single server (port 8080) for all robot types with lazy startup/auto-shutdown
- **Multi-robot support**: SO101, LeKiwi, Trossen WXAI, Franka Panda, and Fanuc industrial robots
- **Cross-robot teleoperation**: Control any robot with any leader (e.g., SO101 controlling Franka)
- **WebSocket control interface**: JSON-based protocol with per-client configuration
- **ZMQ control interface**: Alternative REQ/REP protocol on port 5555
- **Multi-camera capture system**:
  - WebSocket streaming (multiple cameras per robot)
  - Video file output (FFMPEG-based, crash-safe)
  - Image sequence output (JPEG/PNG)
- **Asset serving**: URDF and mesh files served via HTTP for web-based 3D visualization
- **Scene builder**: Programmatically add graspable objects, body-mounted cameras, and collision primitives
- **Real-time state broadcasting**: 30fps joint state updates to all connected clients

## Quick Start

### Using Mise (Recommended)

This project uses [mise](https://mise.jdx.dev) for tool version management, environment configuration, and task running.

```bash
# Clone with submodules
git clone --recursive git@github.com:MarkRedeman/RobotSimServer.jl.git
cd RobotSimServer.jl

# Trust and install tools (installs Julia 1.12.4)
mise trust && mise install

# Install dependencies and MuJoCo visualizer
mise run setup

# Start the unified multi-robot server (recommended)
mise run server

# Or run individual robot simulations
mise run so101
mise run trossen
```

**Available tasks:**

| Task | Description |
|------|-------------|
| `mise run setup` | Install dependencies and MuJoCo visualizer |
| `mise run server` | Start unified multi-robot server (port 8080) |
| `mise run server:8888` | Start unified server on port 8888 |
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
git clone --recursive git@github.com:MarkRedeman/RobotSimServer.jl.git
cd RobotSimServer.jl

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

### Using Docker Compose

If you don't want to install Julia locally, you can run the server with Docker Compose:

```bash
# Clone with submodules
git clone --recursive git@github.com:MarkRedeman/RobotSimServer.jl.git
cd RobotSimServer.jl

# Build and start the server
docker compose up -d

# Check logs
docker compose logs -f

# Stop the server
docker compose down
```

The server will be available at `http://localhost:8080`. To use a different port:

```bash
PORT=9090 docker compose up -d
```

> **Note**: The first build takes a while as it installs Julia packages and precompiles them. Subsequent builds are cached.

## Project Structure

```
.
├── src/                           # Reusable library code
│   ├── SceneBuilder.jl            # Add objects, cameras, collisions to scenes
│   ├── RobotTypes.jl              # Robot type definitions and joint mappings
│   ├── TeleoperatorMapping.jl     # Cross-robot teleoperation support
│   ├── WebSocketServer.jl         # Shared WebSocket control server logic
│   ├── HeadlessRenderer.jl        # Offscreen OpenGL rendering
│   ├── SimulationInstance.jl      # Single robot simulation encapsulation
│   ├── SimulationManager.jl       # Multi-robot orchestration
│   ├── RobotConfigs.jl            # Robot configuration factories
│   ├── AssetServer.jl             # HTTP serving for URDF/meshes
│   ├── URDFGenerator.jl           # MJCF to URDF conversion
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
│   ├── lekiwi/                    # LeKiwi mobile robot examples
│   │   ├── basic_sim.jl           # Mobile base demo
│   │   └── websocket_sim.jl       # Full WebSocket + cameras
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
├── mise.toml                      # Mise configuration (tools, env, tasks)
└── unified_server.jl              # Unified multi-robot server entry point
## Unified Multi-Robot Server

The unified server is the recommended way to run simulations. It provides a single HTTP/WebSocket endpoint for all robots with automatic lifecycle management.

### Starting the Server

```bash
# Using mise (recommended)
mise run server

# Using mise with custom port
mise run server:8888

# Manual (without mise)
julia --project=. -t 4 unified_server.jl
julia --project=. -t 4 unified_server.jl --port 8888
```

### Server Features

- **Single port** (default 8080) serves all robot types
- **Lazy startup**: Simulations start when first client connects
- **Auto-shutdown**: Simulations stop 30 seconds after last client disconnects
- **Per-client leader types**: Each client can specify their leader robot via query parameter
- **Asset serving**: URDF and mesh files served via HTTP for web visualization
- **CORS enabled**: Works with browser-based clients

### Server Endpoints

| Endpoint | Description |
|----------|-------------|
| `GET /` | Info page with available robots and active simulations |
| `GET /robots` | JSON list of available and active robots |
| `GET /health` | Health check endpoint |
| `WS /{robot}/control?leader=X` | Control WebSocket for robot |
| `WS /{robot}/cameras/{name}` | Camera stream WebSocket |
| `GET /{robot}/urdf` | Robot URDF (auto-generated if needed) |
| `GET /{robot}/meshes/{path}` | Mesh/texture files |

### Supported Robot IDs

| Robot ID | Description |
|----------|-------------|
| `so101` | SO-ARM100 desktop robot arm |
| `lekiwi` | LeKiwi mobile manipulator |
| `trossen/wxai` or `trossen` | Trossen Robotics WXAI arm |
| `franka` | Franka Emika Panda |
| `fanuc/m10ia` | Fanuc M-10iA (default) |
| `fanuc/{variant}` | Any Fanuc variant (e.g., `fanuc/crx10ial`) |

### Example: Connecting to Multiple Robots

```bash
# Start the unified server
mise run server

# In separate terminals, connect clients to different robots:
# Terminal 1: Control SO101
wscat -c "ws://localhost:8080/so101/control"

# Terminal 2: Control Franka with SO101 as leader
wscat -c "ws://localhost:8080/franka/control?leader=so101"

# Terminal 3: View SO101 front camera
wscat -c "ws://localhost:8080/so101/cameras/front"
```

### Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                     SimulationManager (port 8080)                   │
│                                                                      │
│  HTTP Router                                                        │
│    /{robot}/control  → WebSocket → SimulationInstance               │
│    /{robot}/cameras  → WebSocket → Camera streams                   │
│    /{robot}/urdf     → HTTP      → URDF file                        │
│    /{robot}/meshes   → HTTP      → Mesh files                       │
│                                                                      │
│  Active Simulations (started on-demand)                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐               │
│  │ SO101        │  │ LeKiwi       │  │ Franka       │  ...          │
│  │ (headless)   │  │ (headless)   │  │ (headless)   │               │
│  └──────────────┘  └──────────────┘  └──────────────┘               │
│                                                                      │
│  Lifecycle: start on first client → stop 30s after last disconnect │
└─────────────────────────────────────────────────────────────────────┘
```

## Supported Robots

### SO101 (SO-ARM100)

A 6-DOF desktop robot arm with:
- **Joints**: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
- **Control**: Position control in degrees
- **Gripper**: -10° (closed) to 100° (open)

### LeKiwi

A mobile manipulator with 3-wheel omnidirectional base and SO101-based arm:
- **Arm Joints**: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
- **Base**: 3 omnidirectional wheels for holonomic motion
- **Cameras**: Front-facing and wrist-mounted cameras built into model
- **Control**: Arm joints in degrees, base velocity commands

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

**Unified Server (recommended):**
- Connect to `ws://localhost:8080/{robot}/control` for joint control
- Optionally specify leader type: `ws://localhost:8080/{robot}/control?leader=so101`

**Individual Examples:**
- Connect to `ws://localhost:8081` for joint control

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

### Unified Server

Camera feeds are available at `ws://localhost:8080/{robot}/cameras/{camera_name}`:

| Robot | Available Cameras |
|-------|-------------------|
| so101 | front, side, orbit, gripper |
| lekiwi | front, wrist, side_left, side_right |
| trossen | front, side, orbit, gripper |
| franka | front, side, orbit, gripper, wrist |
| fanuc/* | front, side, orbit, gripper |

Example:
```bash
# Stream SO101 front camera
wscat -c "ws://localhost:8080/so101/cameras/front"

# Stream Franka wrist camera
wscat -c "ws://localhost:8080/franka/cameras/wrist"
```

### Individual Examples

When running individual WebSocket simulations (e.g., `examples/so101/websocket_sim.jl`), camera feeds are available on separate ports:

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

## Dependencies

- [MuJoCo.jl](https://github.com/JamieMair/MuJoCo.jl) - MuJoCo physics simulation
- [HTTP.jl](https://github.com/JuliaWeb/HTTP.jl) - WebSocket server
- [JSON.jl](https://github.com/JuliaIO/JSON.jl) - JSON parsing
- [ZMQ.jl](https://github.com/JuliaInterop/ZMQ.jl) - ZeroMQ bindings
- [Images.jl](https://github.com/JuliaImages/Images.jl) - Image processing
- [FileIO.jl](https://github.com/JuliaIO/FileIO.jl) - File I/O
- [EzXML.jl](https://github.com/JuliaIO/EzXML.jl) - XML parsing (for URDF generation)
- [SHA.jl](https://github.com/JuliaLang/SHA.jl) - Hashing (for URDF caching)
- [GLFW.jl](https://github.com/JuliaGL/GLFW.jl) - OpenGL context (for headless rendering)

## License

See individual robot model licenses in their respective submodule directories.
