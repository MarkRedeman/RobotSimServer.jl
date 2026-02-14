# =============================================================================
# Stage 1: Builder - Install dependencies and precompile
# =============================================================================
FROM julia:1.12.4-bookworm AS builder

# Install build dependencies for OpenGL/GLFW (needed by MuJoCo visualiser)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglfw3-dev \
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    libgles2-mesa-dev \
    mesa-utils \
    xvfb \
    xauth \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy dependency files first for better layer caching
COPY Project.toml Manifest.toml ./

# Install Julia packages
RUN julia --project=. -e 'using Pkg; Pkg.instantiate()'

# Install MuJoCo visualiser (requires a virtual display for GLFW)
RUN xvfb-run julia --project=. -e 'using MuJoCo; install_visualiser()'

# Precompile all packages used by the server to speed up first startup
# Clean stale Xvfb lock file left by the previous xvfb-run step (baked into this layer)
RUN rm -f /tmp/.X99-lock && xvfb-run julia --project=. -e ' \
    using MuJoCo; \
    using MuJoCo.LibMuJoCo; \
    using HTTP; \
    using HTTP.WebSockets; \
    using JSON; \
    using Images; \
    using FileIO; \
    using EzXML; \
    using SHA; \
    using GLFW; \
    using BangBang; \
    using StaticArrays; \
    using Observables; \
    using Printf; \
    using FFMPEG; \
    using ZMQ; \
    println("All packages precompiled successfully") \
'

# =============================================================================
# Stage 2: Runtime - Minimal image for running the server
# =============================================================================
FROM julia:1.12.4-bookworm

# Install runtime dependencies only (no -dev packages)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglfw3 \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libegl1-mesa \
    libgles2-mesa \
    libglvnd0 \
    xvfb \
    xauth \
    curl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy Julia depot (installed packages + precompilation cache) from builder
COPY --from=builder /root/.julia /root/.julia

# Copy application source and assets directly from build context
COPY Project.toml Manifest.toml ./
COPY unified_server.jl ./
COPY src/ src/
COPY robots/ robots/
COPY examples/ examples/

# Server port (configurable at runtime via PORT env var)
ENV PORT=8080
ENV JULIA_NUM_THREADS=auto

EXPOSE 8080

HEALTHCHECK --interval=30s --timeout=5s --start-period=120s --retries=3 \
    CMD curl -f http://localhost:${PORT}/robots || exit 1

# Start Xvfb manually and exec Julia as PID 1 for proper signal handling.
# xvfb-run is not used because `exec xvfb-run` hangs when xvfb-run becomes PID 1.
ENTRYPOINT ["/bin/sh", "-c", "Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp & sleep 0.5; export DISPLAY=:99; exec julia --project=. -t auto unified_server.jl --host 0.0.0.0 --port $PORT"]
