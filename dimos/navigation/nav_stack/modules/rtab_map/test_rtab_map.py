# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for the RtabMap module wrapper.

These tests cover the wrapper contract — defaults, port declarations, config
validation, and integration with ``create_nav_stack`` — without driving the
runner end-to-end. Algorithmic behavior is validated in ``tests/``.
"""

from __future__ import annotations

from collections.abc import Generator

import pytest

from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.main import create_nav_stack
from dimos.navigation.nav_stack.modules.pgo.pgo import PGO
from dimos.navigation.nav_stack.modules.rtab_map.rtab_map import (
    RtabMap,
    RtabMapConfig,
)


@pytest.fixture()
def module() -> Generator[RtabMap, None, None]:
    """Construct a RtabMap with defaults; tear down after the test."""
    instance = RtabMap()
    try:
        yield instance
    finally:
        instance._close_module()


# ---------------------------------------------------------------------------
# Required defaults (locked by the task definition)
# ---------------------------------------------------------------------------


def test_defaults_match_spec() -> None:
    """The six OctoMap-relevant defaults required by the locked definition."""
    config = RtabMapConfig()
    assert config.grid_3d is True
    assert config.grid_ray_tracing is True
    assert config.grid_from_depth is False
    assert config.grid_cell_size == pytest.approx(0.1)
    assert config.grid_max_ground_angle == pytest.approx(45.0)
    assert config.grid_ground_is_obstacle is False
    assert config.grid_flat_obstacle_detected is True


def test_loop_closure_tunables_present() -> None:
    """Loop-closure tunables exist with sensible defaults (parity with PGO)."""
    config = RtabMapConfig()
    assert config.key_pose_delta_trans > 0
    assert config.key_pose_delta_deg > 0
    assert config.loop_search_radius > 0
    assert 0.0 < config.loop_score_thresh < 1.0
    assert config.submap_resolution > 0
    assert config.min_loop_detect_duration > 0


# ---------------------------------------------------------------------------
# Port declarations (the SLAM-swap contract)
# ---------------------------------------------------------------------------


def test_required_input_ports_declared(module: RtabMap) -> None:
    """Inputs match PGO's contract exactly."""
    inputs = module.inputs
    assert "registered_scan" in inputs
    assert "odometry" in inputs
    assert isinstance(inputs["registered_scan"], In)
    assert isinstance(inputs["odometry"], In)


def test_required_output_ports_declared(module: RtabMap) -> None:
    """Outputs include PGO parity (corrected_odometry, global_map, rtab_tf) plus
    the OctoMap-specific extras (octomap, projected_2d_grid)."""
    outputs = module.outputs
    for port_name in (
        "corrected_odometry",
        "global_map",
        "rtab_tf",
        "octomap",
        "projected_2d_grid",
    ):
        assert port_name in outputs, f"missing output port: {port_name}"
        assert isinstance(outputs[port_name], Out), f"{port_name} not an Out port"


def test_port_types_match_pgo_parity(module: RtabMap) -> None:
    """Streams shared with PGO use the same message types — required for
    autoconnect to consider RtabMap a drop-in replacement."""
    pgo_module = PGO()
    try:
        pgo_inputs = pgo_module.inputs
        pgo_outputs = pgo_module.outputs

        for port_name in ("registered_scan", "odometry"):
            assert module.inputs[port_name].type is pgo_inputs[port_name].type

        for port_name in ("corrected_odometry", "global_map"):
            assert module.outputs[port_name].type is pgo_outputs[port_name].type
    finally:
        pgo_module._close_module()


def test_octomap_outputs_are_pointcloud2(module: RtabMap) -> None:
    """The OctoMap-specific outputs are PointCloud2 so no new msg type is needed."""
    outputs = module.outputs
    assert outputs["octomap"].type is PointCloud2
    assert outputs["projected_2d_grid"].type is PointCloud2


def test_rtab_tf_is_odometry(module: RtabMap) -> None:
    """rtab_tf publishes the map->odom correction as an Odometry message, the
    same shape PGO uses for pgo_tf so the TF bridge can consume it identically."""
    assert module.outputs["rtab_tf"].type is Odometry


# ---------------------------------------------------------------------------
# Config validation
# ---------------------------------------------------------------------------


def test_config_accepts_overrides() -> None:
    """Per-field overrides work as expected (mypy-compatible kwargs path)."""
    config = RtabMapConfig(grid_cell_size=0.25, grid_max_ground_angle=30.0)
    assert config.grid_cell_size == pytest.approx(0.25)
    assert config.grid_max_ground_angle == pytest.approx(30.0)
    # Untouched values retain their spec defaults
    assert config.grid_3d is True
    assert config.grid_ray_tracing is True


def test_invalid_kwarg_rejected() -> None:
    """Unknown config keys raise — guards against silent typos."""
    with pytest.raises(Exception):
        RtabMapConfig(nonexistent_knob=True)  # type: ignore[call-arg]


# ---------------------------------------------------------------------------
# Blueprint wiring (the slam_choice swap)
# ---------------------------------------------------------------------------


def test_create_nav_stack_default_uses_pgo() -> None:
    """The locked spec requires PGO to remain the default slam provider so
    existing blueprints keep working without changes."""
    blueprint = create_nav_stack()
    module_classes = _module_classes(blueprint)
    assert PGO in module_classes
    assert RtabMap not in module_classes


def test_create_nav_stack_with_rtab_swaps_slam() -> None:
    """``slam_choice="rtab"`` swaps PGO for RtabMap. Both publish
    corrected_odometry, so the rest of the stack is unaffected."""
    blueprint = create_nav_stack(slam_choice="rtab")
    module_classes = _module_classes(blueprint)
    assert RtabMap in module_classes
    assert PGO not in module_classes


def test_create_nav_stack_rejects_invalid_slam_choice() -> None:
    """Unknown slam_choice values must error rather than silently fall through."""
    with pytest.raises(Exception, match="invalid slam_choice"):
        create_nav_stack(slam_choice="not_a_slam")  # type: ignore[arg-type]


def test_rtab_map_config_propagation_via_create_nav_stack() -> None:
    """Per-module config dict in ``create_nav_stack`` reaches RtabMap."""
    blueprint = create_nav_stack(
        slam_choice="rtab",
        rtab_map={"grid_cell_size": 0.2, "grid_max_ground_angle": 30.0},
    )
    rtab_atom = _find_atom_for(blueprint, RtabMap)
    assert rtab_atom is not None
    assert rtab_atom.kwargs["grid_cell_size"] == pytest.approx(0.2)
    assert rtab_atom.kwargs["grid_max_ground_angle"] == pytest.approx(30.0)


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _module_classes(blueprint: Blueprint) -> set[type[Module]]:
    """Walk a Blueprint's atoms and return the set of module classes present."""
    return {atom.module for atom in blueprint.blueprints}


def _find_atom_for(blueprint: Blueprint, module_class: type[Module]) -> BlueprintAtom | None:
    for atom in blueprint.blueprints:
        if atom.module is module_class:
            return atom
    return None
