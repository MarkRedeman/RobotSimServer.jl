# AGENTS.md - Guide for AI/LLM Contributors

This document provides guidance for AI assistants and LLMs working with this codebase.

## Project Overview

**Name:** Geti Action MuJoCo Simulation Example

**Purpose:** A proof-of-concept demonstrating how to integrate MuJoCo robot arm
simulations with Geti Action - a local desktop application for robotics researchers
to collect teleoperation data, train VLA models, and deploy to robots.

**Language:** Julia

**Type:** Script collection (not a traditional Julia package)

## Project Structure

```
.
├── src/                           # Reusable library code
│   ├── SceneBuilder.jl            # Scene modification utilities
│   ├── WebSocketServer.jl         # Shared WebSocket control server
│   └── capture/                   # Multi-camera capture system
│       ├── Capture.jl             # Main capture module
│       ├── types.jl               # Type definitions
│       ├── manager.jl             # Capture orchestration
│       ├── worker.jl              # Async I/O worker
│       └── backends/              # Output backends (file, video, websocket)
│
├── examples/                      # Runnable simulation scripts
│   ├── so101/                     # SO101 robot examples
│   ├── trossen/                   # Trossen WXAI examples
│   ├── franka/                    # Franka Panda examples
│   └── clients/                   # Test client examples
│
├── robots/                        # Robot model submodules (git submodules)
│   ├── SO-ARM100/
│   ├── trossen_arm_mujoco/
│   ├── franka/
│   └── google-deepmind/           # MuJoCo Menagerie models
│
├── .github/workflows/             # CI/CD configuration
│   └── CI.yml                     # Format check + syntax validation
│
├── Project.toml                   # Julia dependencies
├── Manifest.toml                  # Locked dependency versions
├── mise.toml                      # Mise configuration (tools, env, tasks)
├── .JuliaFormatter.toml           # Code formatter configuration
└── README.md                      # User documentation
```

## Code Style & Formatting

### JuliaFormatter with SciML Style

This project uses [JuliaFormatter.jl](https://github.com/domluna/JuliaFormatter.jl)
with the [SciML style](https://github.com/SciML/SciMLStyle).

**Configuration:** `.JuliaFormatter.toml`

### Key Style Rules

| Rule | Description |
|------|-------------|
| `indent = 4` | 4-space indentation |
| `margin = 92` | 92 character line limit |
| `always_for_in = true` | Use `for x in collection` (not `for x = collection`) |
| `whitespace_typedefs = true` | Space around `::` in type annotations |
| `remove_extra_newlines = true` | No excessive blank lines |
| `short_to_long_function_def = true` | Prefer `function f(x) ... end` over `f(x) = ...` for complex functions |

### Formatting Code Locally

**One-liner (if JuliaFormatter not installed):**
```bash
julia -e 'using Pkg; Pkg.add("JuliaFormatter"); using JuliaFormatter; format(".")'
```

**If JuliaFormatter is already installed:**
```bash
julia -e 'using JuliaFormatter; format(".")'
```

**Check without modifying (CI mode):**
```bash
julia -e 'using JuliaFormatter; format(".", overwrite=false)'
```

### Pre-commit Checklist

Before committing Julia code changes:

1. Run JuliaFormatter: `julia -e 'using JuliaFormatter; format(".")'`
2. Verify syntax: `julia --project=. -e 'include("src/SceneBuilder.jl")'`
3. Test an example if modifying library code

## Running the Project

This project uses [mise](https://mise.jdx.dev) for tool version management and task running.
Mise automatically sets `JULIA_PROJECT=@.` and `JULIA_NUM_THREADS=auto`.

### Using Mise (Recommended)

```bash
# Trust and install tools (Julia 1.12.4)
mise trust && mise install

# Install dependencies and MuJoCo visualizer
mise run setup

# Run simulations
mise run so101              # SO101 with WebSocket control
mise run trossen            # Trossen WXAI with WebSocket control
mise run franka             # Franka Panda with WebSocket control
mise run so101-basic        # SO101 basic demo
mise run trossen-basic      # Trossen basic demo
mise run franka-basic       # Franka Panda basic demo

# Development
mise run format             # Format all Julia code
mise run client             # Run WebSocket test client
```

### Alternative (Without Mise)

```bash
# Install Julia dependencies
julia --project=. -e 'using Pkg; Pkg.instantiate()'

# Install MuJoCo visualizer (first time only)
julia --project=. -e 'using MuJoCo; install_visualiser()'
```

### Running Simulations (Without Mise)

```bash
# SO101 with WebSocket control (requires multi-threading)
julia --project=. -t 4 examples/so101/websocket_sim.jl

# Trossen WXAI with WebSocket control
julia --project=. -t 4 examples/trossen/websocket_sim.jl

# Basic demo (no external control)
julia --project=. examples/so101/basic_sim.jl
```

### Running Client Examples

```bash
# Julia WebSocket client
julia --project=. examples/clients/ws_client.jl

# Julia ZMQ client
julia --project=. examples/clients/zmq_client.jl

# Python ZMQ client
python examples/clients/zmq_client.py
```

## CI/CD Pipeline

### GitHub Actions Workflow

The CI workflow (`.github/workflows/CI.yml`) runs on every push and PR to `main`:

| Job | Description |
|-----|-------------|
| `format` | Checks that all Julia files are properly formatted |
| `syntax` | Validates that all Julia files can be parsed without syntax errors |

### CI Failure Resolution

**Format check failed:**
```bash
# Run formatter locally and commit the changes
julia -e 'using JuliaFormatter; format(".")'
git add -A && git commit -m "style: format Julia code"
```

**Syntax validation failed:**
- Check the error message for the file and line number
- Fix the syntax error
- Test locally before pushing

## Common Development Tasks

### Adding a New Example

1. Create file in appropriate directory:
   - `examples/so101/` for SO101 robot
   - `examples/trossen/` for Trossen robot
   - `examples/franka/` for Franka Panda
   - `examples/clients/` for client utilities

2. Use relative includes for library code:
   ```julia
   include("../../src/SceneBuilder.jl")
   include("../../src/WebSocketServer.jl")  # For WebSocket-controlled simulations
   include("../../src/capture/Capture.jl")
   ```

3. Use relative paths for robot models:
   ```julia
   xml_path = joinpath(@__DIR__, "..", "..", "robots", "SO-ARM100", "Simulation", "SO101", "scene.xml")
   ```

4. Format before committing

### Modifying Library Code (src/)

1. Edit files in `src/`
2. Test changes by running an example script
3. Format before committing

### Adding a New Robot

1. Add robot model as git submodule in `robots/`
2. Create example directory: `examples/<robot_name>/`
3. Create basic simulation script
4. Update README.md with robot documentation

## WebSocket API Reference

### Control Endpoint

**URL:** `ws://localhost:8081`

### Commands (Client → Server)

**Set Joint Positions:**
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
*All values in degrees.*

**Ping:**
```json
{"command": "ping"}
```

### Events (Server → Client)

**State Update (broadcast at 30fps):**
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

### Camera Streams

| Camera | Port | Description |
|--------|------|-------------|
| Front | 8082 | External front view |
| Side | 8083 | External side view |
| Orbit | 8084 | Rotating view |
| Gripper | 8085 | First-person gripper view |

*Each port streams raw JPEG frames over WebSocket.*

## Dependencies

### Runtime Dependencies (Project.toml)

- **MuJoCo.jl** - Physics simulation
- **HTTP.jl** - WebSocket server
- **JSON.jl** - JSON parsing
- **ZMQ.jl** - ZeroMQ bindings
- **Images.jl** / **ImageIO.jl** / **FileIO.jl** - Image processing

### Development Dependencies

- **JuliaFormatter.jl** - Code formatting (install separately)

## Troubleshooting

### "VisualiserExt not loaded"

Run `init_visualiser()` before using visualization functions:
```julia
using MuJoCo
init_visualiser()
```

### WebSocket simulation hangs

Ensure multi-threading is enabled. With mise, this is automatic. Without mise:
```bash
julia --project=. -t 4 examples/so101/websocket_sim.jl
```

### Robot model not found

Ensure submodules are initialized:
```bash
git submodule update --init --recursive
```

## Notes for AI Assistants

1. **This is NOT a Julia package** - it's a script collection. There's no module to import.
2. **Always format before suggesting commits** - CI will fail otherwise.
3. **Use `@__DIR__` for relative paths** - scripts can be run from any directory.
4. **Include statements use relative paths** from the script location.
5. **Test examples with `-t 4`** for multi-threading when WebSocket is involved.
