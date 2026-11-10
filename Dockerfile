FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-venv git make cmake build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

# Dependencies first so the layer is cached across source changes.
COPY requirements.txt requirements-dev.txt ./
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir -r requirements-dev.txt

COPY . .

# Unit tests are the only thing this image is expected to run: CI builds it and
# runs pytest inside it on every push (the `docker` job), so the image cannot rot
# silently. SITL itself needs the PX4 patch and a GPU-capable Gazebo, which is a
# different problem - see docs/setup-log.md rather than expecting this image to
# fly.
CMD ["make", "test"]
