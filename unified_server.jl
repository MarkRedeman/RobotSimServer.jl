#!/usr/bin/env julia
# unified_server.jl - Entry point for the unified multi-robot simulation server
#
# Starts a single HTTP/WebSocket server that can run any supported robot simulation.
# Simulations start on-demand when clients connect and stop when idle.
#
# Usage:
#   julia --project=. -t 4 unified_server.jl
#   julia --project=. -t 4 unified_server.jl --port 8080
#   julia --project=. -t 4 unified_server.jl --help
#
# Endpoints (default port 8080):
#   ws://localhost:8080/{robot}/control?leader=X  - Control WebSocket
#   ws://localhost:8080/{robot}/cameras/{name}    - Camera stream
#   GET http://localhost:8080/{robot}/urdf        - Robot URDF
#   GET http://localhost:8080/{robot}/meshes/*    - Mesh files
#   GET http://localhost:8080/                    - Info page
#   GET http://localhost:8080/robots              - Available robots (JSON)
#
# Supported robots:
#   so101, lekiwi, trossen/wxai, franka, fanuc/m10ia, ...

using Pkg
Pkg.activate(@__DIR__)

# Check for required threads
if Threads.nthreads() < 2
    @warn """
    Running with only $(Threads.nthreads()) thread(s).
    For best performance, use: julia --project=. -t 4 unified_server.jl
    """
end

println("Loading dependencies...")

using MuJoCo
using MuJoCo.LibMuJoCo
using HTTP
using HTTP.WebSockets
using JSON
using Images
using FileIO
using EzXML
using SHA
using GLFW

# Install MuJoCo visualiser if needed (for GLFW initialization)
try
    init_visualiser()
catch e
    if occursin("not installed", string(e))
        println("Installing MuJoCo visualiser...")
        install_visualiser()
        init_visualiser()
    else
        rethrow()
    end
end

println("Loading simulation framework...")

# Load source files in dependency order
const PROJECT_ROOT = @__DIR__

include(joinpath(PROJECT_ROOT, "src", "SceneBuilder.jl"))
include(joinpath(PROJECT_ROOT, "src", "RobotTypes.jl"))
include(joinpath(PROJECT_ROOT, "src", "Kinematics.jl"))
include(joinpath(PROJECT_ROOT, "src", "TeleoperatorMapping.jl"))
include(joinpath(PROJECT_ROOT, "src", "CLIUtils.jl"))
include(joinpath(PROJECT_ROOT, "src", "HeadlessRenderer.jl"))
include(joinpath(PROJECT_ROOT, "src", "RobotConfigs.jl"))
include(joinpath(PROJECT_ROOT, "src", "BaseController.jl"))
include(joinpath(PROJECT_ROOT, "src", "AssetServer.jl"))
include(joinpath(PROJECT_ROOT, "src", "URDFGenerator.jl"))
include(joinpath(PROJECT_ROOT, "src", "SimulationInstance.jl"))
include(joinpath(PROJECT_ROOT, "src", "SimulationManager.jl"))

# =============================================================================
# Command Line Parsing
# =============================================================================

function parse_args()
    port = 8080
    host = "127.0.0.1"

    i = 1
    while i <= length(ARGS)
        arg = ARGS[i]

        if arg == "--help" || arg == "-h"
            println("""
Unified Multi-Robot Simulation Server

Usage:
    julia --project=. -t 4 unified_server.jl [options]

Options:
    --port PORT     Port to listen on (default: 8080)
    --host HOST     Host/IP to bind to (default: 127.0.0.1)
    --help, -h      Show this help message

Examples:
    julia --project=. -t 4 unified_server.jl
    julia --project=. -t 4 unified_server.jl --port 8888
    julia --project=. -t 4 unified_server.jl --host 0.0.0.0

Endpoints:
    ws://localhost:PORT/{robot}/control?leader=X  - Control WebSocket
    ws://localhost:PORT/{robot}/cameras/{name}    - Camera stream
    GET http://localhost:PORT/{robot}/urdf        - Robot URDF
    GET http://localhost:PORT/{robot}/meshes/*    - Mesh files
    GET http://localhost:PORT/                    - Info page
    GET http://localhost:PORT/robots              - Available robots (JSON)

Supported robots:
    $(join(list_available_robots(), ", "))

Notes:
    - Simulations start when clients connect (lazy startup)
    - Simulations stop 30s after last client disconnects
    - Use ?leader=X query param to specify leader robot type
    - Camera streams send raw JPEG frames over WebSocket
""")
            exit(0)
        elseif arg == "--port"
            i += 1
            if i > length(ARGS)
                @error "Missing port value after --port"
                exit(1)
            end
            port = parse(Int, ARGS[i])
        elseif arg == "--host"
            i += 1
            if i > length(ARGS)
                @error "Missing host value after --host"
                exit(1)
            end
            host = ARGS[i]
        else
            @warn "Unknown argument: $arg"
        end

        i += 1
    end

    return (port = port, host = host)
end

# =============================================================================
# Main
# =============================================================================

function main()
    args = parse_args()

    println()
    println("=" ^ 60)
    println("  Unified Multi-Robot Simulation Server")
    println("=" ^ 60)
    println()

    # Create and start the manager
    manager = SimulationManager(args.port, PROJECT_ROOT; host = args.host)
    start!(manager)

    println()
    println("Press Ctrl+C to stop the server")
    println()

    # Wait for interrupt
    try
        # Keep the main task alive
        while manager.running
            sleep(1)
        end
    catch e
        if e isa InterruptException
            println("\nShutdown requested...")
        else
            rethrow()
        end
    finally
        stop!(manager)
    end

    println("Server stopped.")
end

# Run if executed directly
if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
