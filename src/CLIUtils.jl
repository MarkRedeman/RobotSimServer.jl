# CLIUtils - Command-line and query parameter parsing utilities
#
# Provides helpers for parsing CLI arguments and WebSocket query parameters
# in simulation scripts.
#
# Usage:
#   include("src/RobotTypes.jl")
#   include("src/CLIUtils.jl")
#   
#   # CLI argument parsing
#   leader_type = parse_leader_type(ARGS)
#   
#   # Query parameter parsing (for WebSocket URLs)
#   target = "/lekiwi/control?leader=trossen"
#   leader_type = parse_leader_from_query(target; default=SO101)
#
# Dependencies:
#   This file assumes RobotTypes.jl is already loaded (for AbstractRobotArm,
#   ROBOT_TYPE_MAP, and concrete robot types).

# =============================================================================
# CLI Argument Parsing
# =============================================================================

"""
    parse_leader_type(args::Vector{String}; default::Type{<:AbstractRobotArm}=SO101)

Parse the --leader=<type> argument from command line args.

# Arguments
- `args`: Command line arguments (typically ARGS)
- `default`: Default robot type if --leader not specified

# Returns
- Robot type (e.g., SO101, TrossenWXAI)

# Example
```julia
leader = parse_leader_type(ARGS)  # --leader=trossen -> TrossenWXAI
```
"""
function parse_leader_type(args::Vector{String}; default::Type{<:AbstractRobotArm} = SO101)
    for arg in args
        if startswith(arg, "--leader=")
            type_str = lowercase(split(arg, "=")[2])
            if haskey(ROBOT_TYPE_MAP, type_str)
                return ROBOT_TYPE_MAP[type_str]
            else
                @warn "Unknown leader type: $type_str, using default" default
                println("Available leader types: $(join(keys(ROBOT_TYPE_MAP), ", "))")
            end
        end
    end
    return default
end

# =============================================================================
# Query Parameter Parsing
# =============================================================================

"""
    parse_query_param(target::String, param::String; default::String="")

Parse a query parameter from an HTTP request target.

# Arguments
- `target`: HTTP request target (e.g., "/lekiwi/control?leader=so101&fps=60")
- `param`: Parameter name to extract (e.g., "leader")
- `default`: Default value if parameter not found

# Returns
- Parameter value as String

# Example
```julia
target = "/lekiwi/control?leader=trossen&fps=60"
parse_query_param(target, "leader")   # => "trossen"
parse_query_param(target, "fps")      # => "60"
parse_query_param(target, "missing")  # => ""
```
"""
function parse_query_param(target::String, param::String; default::String = "")
    # Split path and query string
    parts = split(target, "?")
    if length(parts) < 2
        return default
    end

    query_string = parts[2]
    for pair in split(query_string, "&")
        kv = split(pair, "=")
        if length(kv) == 2 && kv[1] == param
            return String(kv[2])
        end
    end

    return default
end

"""
    parse_leader_from_query(target::String; default::Type{<:AbstractRobotArm}=SO101)

Parse leader robot type from URL query parameter.

Extracts the `leader` query parameter from a WebSocket URL target and
maps it to a robot type using ROBOT_TYPE_MAP.

# Arguments
- `target`: HTTP request target (e.g., "/lekiwi/control?leader=trossen")
- `default`: Default robot type if leader param not found or invalid

# Returns
- Robot type (e.g., SO101, TrossenWXAI)

# Example
```julia
target = "/lekiwi/control?leader=trossen"
leader_type = parse_leader_from_query(target)  # => TrossenWXAI

target = "/lekiwi/control"
leader_type = parse_leader_from_query(target; default=LeKiwiArm)  # => LeKiwiArm
```
"""
function parse_leader_from_query(target::String; default::Type{<:AbstractRobotArm} = SO101)
    leader_str = parse_query_param(target, "leader")
    if isempty(leader_str)
        return default
    end

    type_str = lowercase(leader_str)
    if haskey(ROBOT_TYPE_MAP, type_str)
        return ROBOT_TYPE_MAP[type_str]
    else
        @warn "Unknown leader type in query: $type_str, using default" default
        return default
    end
end

"""
    parse_robot_option(args::Vector{String}, option::String; default::String="")

Parse a generic robot-related CLI option.

# Arguments
- `args`: Command line arguments (typically ARGS)
- `option`: Option name without leading dashes (e.g., "robot", "variant")
- `default`: Default value if option not found

# Example
```julia
variant = parse_robot_option(ARGS, "variant"; default="m10ia")
```
"""
function parse_robot_option(args::Vector{String}, option::String; default::String = "")
    prefix = "--$(option)="
    for arg in args
        if startswith(arg, prefix)
            return split(arg, "=")[2]
        end
    end
    return default
end

# =============================================================================
# Banner Printing
# =============================================================================

"""
    print_teleop_banner(leader_type, follower_type, strategy; port=8080)

Print a formatted banner showing teleoperation configuration.

# Arguments
- `leader_type`: Leader robot type (e.g., SO101)
- `follower_type`: Follower robot type (e.g., TrossenWXAI)
- `strategy`: MappingStrategy instance from TeleoperatorMapping
- `port`: WebSocket server port (default 8080)

Note: strategy type checking uses duck typing - any struct with the right
name pattern will work (DirectMapping, JointNameMapping, IKBasedMapping).
"""
function print_teleop_banner(leader_type::Type{L}, follower_type::Type{F},
        strategy; port::Int = 8080) where {L, F}
    leader_name = get_robot_name(leader_type)
    follower_name = get_robot_name(follower_type)
    strategy_name = string(typeof(strategy).name.name)

    println()
    println("=" ^ 70)
    println("Teleoperation Configuration")
    println("=" ^ 70)
    println("  Leader type:   $leader_name ($(L))")
    println("  Follower type: $follower_name ($(F))")
    println("  Strategy:      $strategy_name")
    println()

    # Check strategy type by name since the types are in TeleoperatorMapping
    type_name = string(typeof(strategy).name.name)
    if type_name == "DirectMapping"
        println("  Mode: Direct pass-through (same robot type)")
        println("  Joint names: Native $(leader_name) format")
    elseif type_name == "JointNameMapping"
        println("  Mode: Joint name translation")
        println("  Accepts: $(leader_name) joint names")
        println("  Broadcasts: $(leader_name) joint names")
    elseif type_name == "IKBasedMapping"
        println("  Mode: IK-based workspace mapping")
        println("  Accepts: $(leader_name) joint names")
        println("  Maps: $(leader_name) EE position -> $(follower_name) joints via IK")
    end
    println("=" ^ 70)
end

"""
    print_available_leaders()

Print available leader robot types from ROBOT_TYPE_MAP.
"""
function print_available_leaders()
    println("Available leader types:")
    for (name, robot_type) in ROBOT_TYPE_MAP
        human_name = get_robot_name(robot_type)
        println("  --leader=$name  ($human_name)")
    end
end

# =============================================================================
# Robot Name Helpers
# =============================================================================

"""
    get_robot_name(::Type{T}) where T <: AbstractRobotArm

Get a human-readable name for a robot type.
"""
get_robot_name(::Type{SO101}) = "SO-101"
get_robot_name(::Type{TrossenWXAI}) = "Trossen WXAI"
get_robot_name(::Type{LeKiwiArm}) = "LeKiwi Arm"
get_robot_name(::Type{FrankaPanda}) = "Franka Panda"
get_robot_name(::Type{FanucArm}) = "Fanuc Industrial"
get_robot_name(::Type{T}) where {T <: AbstractRobotArm} = string(T)
