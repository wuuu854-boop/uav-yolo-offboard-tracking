# uav-yolo-offboard-tracking — one-command workflows
# Recipes use tabs (GNU make requirement). Do not reformat with spaces.

SHELL    := /bin/bash
PY       ?= python3
PX4_DIR  ?= $(HOME)/PX4-Autopilot
WORLD    ?= target_tracking
# Forward-looking `x500_mono_cam` would break the nadir projection: use _down.
PX4_MODEL ?= gz_x500_mono_cam_down
# Spawn 3.5 m west of target_0 so it is the only target in view at takeoff.
# Per-target poses are listed in docs/experiment-protocol.md.
POSE     ?= -3.5,0,0.05

.PHONY: help setup doctor demo trials qa patch-px4 install-world sim detect e2e preflight test lint fmt lock bench check clean

help: ## show available targets
	@grep -E '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) | awk 'BEGIN{FS=":.*?## "}{printf "  \033[36m%-8s\033[0m %s\n", $$1, $$2}'

setup: ## check environment, create venv, install dependencies
	bash scripts/setup.sh

doctor: ## check this machine against versions.yaml
	$(PY) scripts/doctor.py

bench: ## summarise results/landing_error.csv for the README
	$(PY) scripts/benchmark.py --csv results/landing_error.csv --markdown

demo: ## offline control-loop demo, no ROS or Gazebo needed
	$(PY) scripts/offline_demo.py

check: ## everything CI runs, locally
	$(PY) -m ruff check .
	$(PY) -m ruff format --check .
	$(PY) scripts/doctor.py --static
	$(PY) scripts/check_world_targets.py
	$(PY) scripts/check_links.py
	$(PY) scripts/check_doc_claims.py
	$(PY) scripts/check_dependencies.py
	$(PY) -m pytest -q --cov=uav_tracking --cov-report=term-missing

patch-px4: ## apply the PX4 v1.15 magnetometer patch and rebuild
	bash scripts/apply_px4_patch.sh

install-world: ## copy the world and the nadir camera model where PX4 looks
	@test -d "$(PX4_DIR)/Tools/simulation/gz/worlds" || { echo "PX4 gazebo tree not found at $(PX4_DIR); see sim/README.md"; exit 1; }
	cp sim/worlds/$(WORLD).sdf "$(PX4_DIR)/Tools/simulation/gz/worlds/$(WORLD).sdf"
	# PX4 v1.15's launcher runs `gz sim -r $(HOME)/.simulation-gazebo/worlds/$(WORLD).sdf`,
	# so a world copied only into the source tree is never opened.
	@mkdir -p "$(HOME)/.simulation-gazebo/worlds"
	cp sim/worlds/$(WORLD).sdf "$(HOME)/.simulation-gazebo/worlds/$(WORLD).sdf"
	@if [ ! -d "$(HOME)/.simulation-gazebo/models/x500_mono_cam_down" ]; then \
	  mkdir -p "$(HOME)/.simulation-gazebo/models"; \
	  cp -r sim/models/x500_mono_cam_down "$(HOME)/.simulation-gazebo/models/"; \
	  echo "installed the fallback nadir camera model"; \
	fi
	@echo "installed $(WORLD).sdf"

sim: install-world ## launch Gazebo + PX4 SITL with the target world
	cd $(PX4_DIR) && PX4_GZ_WORLD=$(WORLD) PX4_GZ_MODEL_POSE=$(POSE) make px4_sitl $(PX4_MODEL)

detect: ## run the detection node only
	$(PY) src/uav_tracking/perception/detector_node.py --config config/detector.yaml

e2e: ## closed loop: takeoff -> detect -> approach -> hover -> land
	bash scripts/run_e2e.sh

trials: ## run N closed-loop trials (N=3 default), restarting the sim each time
	bash scripts/run_trials.sh --count $(or $(N),3)

qa: ## data-quality report: make qa CAPTURE=<dir> [ASSET=east,north]
	@test -n "$(CAPTURE)" || (echo "set CAPTURE=<capture dir>; ASSET=<east,north> is optional" && exit 2)
	python3 scripts/qa_report.py --capture "$(CAPTURE)" $(if $(ASSET),--asset $(ASSET),)

preflight: ## check the trial environment without a simulator
	bash scripts/run_e2e.sh --dry-run

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
