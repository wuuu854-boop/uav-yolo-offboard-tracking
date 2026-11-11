# uav-yolo-offboard-tracking — one-command workflows
# Recipes use tabs (GNU make requirement). Do not reformat with spaces.

SHELL    := /bin/bash
PY       ?= python3
PX4_DIR  ?= $(HOME)/PX4-Autopilot
WORLD    ?= target_tracking
# Both worlds are installed: the tracking trials must keep the unobstructed one,
# and the avoidance flight needs the wall. See sim/worlds/target_tracking_obstacle.sdf.
WORLDS   := target_tracking target_tracking_obstacle target_tracking_detour target_tracking_blocked target_tracking_search
# Forward-looking `x500_mono_cam` would break the nadir projection: use _down.
PX4_MODEL ?= gz_x500_mono_cam_down
# Spawn 3.5 m west of target_0 so it is the only target in view at takeoff.
# Per-target poses are listed in docs/experiment-protocol.md.
POSE     ?= -3.5,0,0.05

.PHONY: help setup doctor demo trials qa shots pipeline patch-px4 install-world sim detect e2e avoid detour auto-detour blocked backoff backoff-sweep adaptive search global global-multi global-perception preflight test lint fmt lock bench check clean

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

install-world: ## copy the worlds and the depth-camera model where PX4 looks
	@test -d "$(PX4_DIR)/Tools/simulation/gz/worlds" || { echo "PX4 gazebo tree not found at $(PX4_DIR); see sim/README.md"; exit 1; }
	@for world in $(WORLDS); do \
	  cp "sim/worlds/$$world.sdf" "$(PX4_DIR)/Tools/simulation/gz/worlds/$$world.sdf"; \
	  echo "installed $$world.sdf"; \
	done
	# PX4 v1.15's launcher runs `gz sim -r $(HOME)/.simulation-gazebo/worlds/$(WORLD).sdf`,
	# so a world copied only into the source tree is never opened.
	@mkdir -p "$(HOME)/.simulation-gazebo/worlds" "$(HOME)/.simulation-gazebo/models/x500_mono_cam_down"
	@for world in $(WORLDS); do \
	  cp "sim/worlds/$$world.sdf" "$(HOME)/.simulation-gazebo/worlds/$$world.sdf"; \
	done
	# Always overwrite the installed vehicle model. Since v0.9.0 it carries the
	# forward depth camera; keeping an older copy silently removes the topic and
	# every avoidance run then fails at "topic not found" instead of at the feat.
	# PX4's source-tree copy is deliberately not touched: the model store shadows
	# it, and editing the PX4 checkout would make its git status dirty.
	cp sim/models/x500_mono_cam_down/model.sdf "$(HOME)/.simulation-gazebo/models/x500_mono_cam_down/model.sdf"
	cp sim/models/x500_mono_cam_down/model.config "$(HOME)/.simulation-gazebo/models/x500_mono_cam_down/model.config"
	@echo "installed x500_mono_cam_down (nadir + forward depth camera)"

sim: install-world ## launch Gazebo + PX4 SITL with the target world
	cd $(PX4_DIR) && PX4_GZ_WORLD=$(WORLD) PX4_GZ_MODEL_POSE=$(POSE) make px4_sitl $(PX4_MODEL)

detect: ## run the detection node only
	$(PY) src/uav_tracking/perception/detector_node.py --config config/detector.yaml

e2e: ## closed loop: takeoff -> detect -> approach -> hover -> land
	bash scripts/run_e2e.sh

avoid: install-world ## depth camera: takeoff -> stop before a wall -> land, with evidence
	bash scripts/run_avoidance.sh

detour: install-world ## depth camera: stop, fly around the wall, resume the waypoint
	MISSION=config/mission_detour.yaml PREFIX=detour MODE=detour bash scripts/run_avoidance.sh

auto-detour: install-world ## depth camera: choose the freer side, then fly around
	MISSION=config/mission_detour_auto.yaml WORLD=target_tracking_detour PREFIX=autod MODE=detour EXPECT_SIDE=left SIDE_OBSTACLE=config/side_obstacle.yaml bash scripts/run_avoidance.sh

blocked: install-world ## depth camera: both sides blocked -> hold, do not guess
	MISSION=config/mission_blocked.yaml WORLD=target_tracking_blocked PREFIX=blocked MODE=hold DEPTH_CONFIG=config/depth_blocked.yaml SIDE_OBSTACLE=config/blocked_obstacle.yaml bash scripts/run_avoidance.sh

backoff: install-world ## depth camera: both sides blocked -> back off, re-read, detour
	MISSION=config/mission_backoff.yaml WORLD=target_tracking_blocked PREFIX=backoff MODE=detour DEPTH_CONFIG=config/depth_backoff.yaml SIDE_OBSTACLE=config/blocked_obstacle.yaml bash scripts/run_avoidance.sh

backoff-sweep: install-world ## depth camera: one back-off is not enough -> step back again
	MISSION=config/mission_backoff_sweep.yaml WORLD=target_tracking_blocked PREFIX=sweep MODE=detour DEPTH_CONFIG=config/depth_backoff_sweep.yaml SIDE_OBSTACLE=config/blocked_obstacle.yaml bash scripts/run_avoidance.sh

adaptive: install-world ## depth camera: size each back-off from the measured side clearance
	MISSION=config/mission_adaptive.yaml WORLD=target_tracking_blocked PREFIX=adaptive MODE=detour DEPTH_CONFIG=config/depth_adaptive.yaml SIDE_OBSTACLE=config/blocked_obstacle.yaml bash scripts/run_avoidance.sh

search: install-world ## depth camera: back off, then side-step toward the freer side
	MISSION=config/mission_search.yaml WORLD=target_tracking_search PREFIX=search MODE=detour DEPTH_CONFIG=config/depth_search.yaml SIDE_OBSTACLE=config/search_obstacle.yaml bash scripts/run_avoidance.sh

global: install-world ## known-map A* around the wall (planner demo, no reactive detour)
	MISSION=config/mission_global.yaml WORLD=target_tracking_obstacle PREFIX=global MODE=global bash scripts/run_avoidance.sh

global-multi: install-world ## known-map A* around the wall and two side pillars
	MISSION=config/mission_global_multi.yaml WORLD=target_tracking_search PREFIX=global_multi MODE=global bash scripts/run_avoidance.sh

global-perception: install-world ## depth stop -> one measured box -> replan around it
	MISSION=config/mission_global_perception.yaml WORLD=target_tracking_obstacle PREFIX=global_perception MODE=global bash scripts/run_avoidance.sh

trials: ## run N closed-loop trials (N=3 default), restarting the sim each time
	bash scripts/run_trials.sh --count $(or $(N),3)

qa: ## data-quality report: make qa CAPTURE=<dir> [ASSET=east,north]
	@test -n "$(CAPTURE)" || (echo "set CAPTURE=<capture dir>; ASSET=<east,north> is optional" && exit 2)
	python3 scripts/qa_report.py --capture "$(CAPTURE)" $(if $(ASSET),--asset $(ASSET),)

shots: ## shot-sheet check: make shots CAPTURE=<dir> [ASSET=east,north]
	@test -n "$(CAPTURE)" || (echo "set CAPTURE=<capture dir>; ASSET=<east,north> is optional" && exit 2)
	python3 scripts/shot_sheet_check.py --capture "$(CAPTURE)" $(if $(ASSET),--asset $(ASSET),)

pipeline: ## QA + shot sheet + GeoJSON in one pass: make pipeline CAPTURE=<dir> [ASSET=east,north]
	@test -n "$(CAPTURE)" || (echo "set CAPTURE=<capture dir>; ASSET=<east,north> is optional" && exit 2)
	python3 scripts/pipeline.py --capture "$(CAPTURE)" $(if $(ASSET),--asset $(ASSET),) \
		--out results/pipeline.json --geojson-out results/pipeline.geojson

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
