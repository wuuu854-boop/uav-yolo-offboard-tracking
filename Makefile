# uav-yolo-offboard-tracking — one-command workflows
# Recipes use tabs (GNU make requirement). Do not reformat with spaces.

SHELL    := /bin/bash
PY       ?= python3
PX4_DIR  ?= $(HOME)/PX4-Autopilot
WORLD    ?= target_tracking

.PHONY: help setup sim detect e2e test lint fmt lock clean

help: ## show available targets
	@grep -E '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) | awk 'BEGIN{FS=":.*?## "}{printf "  \033[36m%-8s\033[0m %s\n", $$1, $$2}'

setup: ## check environment, create venv, install dependencies
	bash scripts/setup.sh

sim: ## launch Gazebo + PX4 SITL with the target world
	cd $(PX4_DIR) && PX4_GZ_WORLD=$(WORLD) make px4_sitl gz_x500_mono_cam

detect: ## run the detection node only
	$(PY) src/perception/detector_node.py --config config/detector.yaml

e2e: ## closed loop: takeoff -> detect -> approach -> hover -> land
	bash scripts/run_e2e.sh

test: ## unit tests
	$(PY) -m pytest

lint: ## ruff check + format check
	$(PY) -m ruff check .
	$(PY) -m ruff format --check .

fmt: ## auto-format and auto-fix
	$(PY) -m ruff format .
	$(PY) -m ruff check --fix .

lock: ## freeze exact versions into requirements.lock.txt
	$(PY) -m pip freeze > requirements.lock.txt

clean: ## remove caches
	rm -rf .pytest_cache .ruff_cache
	find . -type d -name __pycache__ -prune -exec rm -rf {} +
