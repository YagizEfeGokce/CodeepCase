# CodeepCase — reproducible environment (Ubuntu 24.04 + all deps baked in).
# Default CMD runs Gate B headless (no X/GPU needed) as a "it works on every
# device" smoke test. For the GUI MuJoCo viewer, see docs/docker.md.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-venv python3-pip python3-dev \
        git cmake build-essential curl ca-certificates \
        libglfw3 libgl1 libglx-mesa0 libxrandr2 libxinerama1 \
        libxcursor1 libxi6 libxxf86vm1 libxext6 libegl1 libgles2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
# .dockerignore excludes external/ .venv/ .git so the build context is tiny;
# scripts/setup.sh re-creates them inside the image (clones deps, builds
# CycloneDDS, pip-installs, installs scenes).
COPY . /app
RUN bash scripts/setup.sh

# Headless by default (no display). Override with -e HEADLESS=0 + X11 for GUI.
ENV HEADLESS=1 SDL_VIDEODRIVER=dummy

# Smoke test: stand the Go2 headless.
CMD ["bash", "run.sh", "b", "--duration", "10"]