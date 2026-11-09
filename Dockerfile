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

# Unit tests are the only thing this image is expected to run in CI.
# The SITL smoke workflow runs scripts/run_e2e.sh explicitly.
CMD ["make", "test"]
