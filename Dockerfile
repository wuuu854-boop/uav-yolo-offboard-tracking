FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends -o Acquire::Retries=8 \
         python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

# Dependencies first so the layer is cached across source changes.
COPY requirements.txt requirements-dev.txt ./
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir -r requirements-dev.txt

COPY . .

# `-e .` so the package imports the same way it does on a workstation. The
# collect-only run is part of the build: if the image cannot see or import the
# tests, the build fails here instead of in a red CI job later.
RUN python3 -m pip install --no-cache-dir -e . \
    && python3 -m pytest --collect-only -q tests | tail -n 2

# Unit tests are the only thing this image is expected to run: CI builds it and
# runs pytest inside it on every push (the `docker` job), so the image cannot rot
# silently. SITL itself needs the PX4 patch and a GPU-capable Gazebo, which is a
# different problem - see docs/setup-log.md rather than expecting this image to
# fly.
CMD ["python3", "-m", "pytest", "tests", "-q"]
