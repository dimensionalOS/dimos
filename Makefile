# DimOS local developer targets (optional conveniences; CI does not depend on this file).

REPO_ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

.PHONY: plot-trajectory-ticks
plot-trajectory-ticks:
	cd "$(REPO_ROOT)" && uv run --with matplotlib python scripts/plot_trajectory_control_ticks.py \
		dimos/navigation/fixtures/trajectory_control_ticks_sample.jsonl
