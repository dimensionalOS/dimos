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

"""Marker-survey path of the loop-closure eval: the mirror-flip gate is on.

The survey builds the map_T_marker entries the fiducial prior relocalizes
against, so a mirror-ambiguous glimpse that slips through here is a confidently
wrong tag pose baked into the map. These tests pin the gate to the live
detector's value and prove the knob actually reaches ``detect_markers_in_image``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest
import typer
from typer.testing import CliRunner

from dimos.mapping.loop_closure import eval as eval_mod
from dimos.mapping.loop_closure.eval import (
    DEFAULT_AMBIGUITY_RATIO_MIN,
    corrected_marker_transforms,
)
from dimos.mapping.loop_closure.pgo import Keyframe, PoseGraph
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.fiducial.marker_detection_stream_module import (
    MarkerDetectionStreamModuleConfig,
)
from dimos.perception.fiducial.test_helpers import (
    camera_info,
    synthetic_marker_image,
    world_T_optical,
)

if TYPE_CHECKING:
    from collections.abc import Iterator
    from pathlib import Path

MARKER_ID = 7
MARKER_LENGTH_M = 0.18  # matches the tag rendered by synthetic_marker_image
FRAME_TS = 10.0
# Runner-up must reproject 1e6x worse to keep a view: no real IPPE pair ever
# does, so this rejects every glimpse — a knob-reached-the-detector probe.
REJECT_EVERYTHING_RATIO = 1e6


def _identity_graph(ts: float) -> PoseGraph:
    """PGO graph whose correction is identity, so surveyed poses pass through."""
    identity = Transform()
    return PoseGraph(keyframes=(Keyframe(ts=ts, local=identity, optimized=identity),))


@pytest.fixture
def survey_store(tmp_path: Path) -> Iterator[SqliteStore]:
    """Open store holding one head-on synthetic tag frame with a camera-in-world pose."""
    store = SqliteStore(path=str(tmp_path / "survey.db"))
    with store:
        store.stream("color_image", Image).append(
            synthetic_marker_image(MARKER_ID, ts=FRAME_TS),
            ts=FRAME_TS,
            pose=world_T_optical(FRAME_TS),
        )
        yield store


def _survey(store: SqliteStore, **kwargs: float) -> dict[int, list[Transform]]:
    """corrected_marker_transforms with the speed gate off (single frame)."""
    return corrected_marker_transforms(
        store,
        _identity_graph(FRAME_TS),
        camera_info=camera_info(),
        marker_size=MARKER_LENGTH_M,
        marker_max_speed=0.0,
        marker_max_rot_rate=0.0,
        marker_quality_window=0.1,
        marker_smoothing=0.0,
        **kwargs,
    )


def test_survey_gate_default_matches_the_live_detector() -> None:
    """Invariant: the survey gates mirror ambiguity at the same ratio the live
    detector does, and that ratio gates (> 1.0, which means off).

    Surveying ungated while the live path gates would put poses in the map that
    the runtime detector would have refused to publish.
    """
    live_default = MarkerDetectionStreamModuleConfig.model_fields["ambiguity_ratio_min"].default
    assert DEFAULT_AMBIGUITY_RATIO_MIN == live_default
    assert DEFAULT_AMBIGUITY_RATIO_MIN > 1.0


def test_survey_gates_by_default_and_ratio_reaches_the_detector(
    survey_store: SqliteStore,
) -> None:
    """Invariant: corrected_marker_transforms called with no ambiguity argument
    applies a gating ratio — one strict enough to reject a mirror-ambiguous view.

    Proven by construction rather than by inspecting the call: the same single
    head-on tag frame is surveyed at a reject-everything ratio (empty result) and
    ungated at 1.0 (the tag is found), so the argument demonstrably reaches
    ``detect_markers_in_image``; the default sits inside that range and is the
    live detector's value.
    """
    ungated = _survey(survey_store, ambiguity_ratio_min=1.0)
    assert set(ungated) == {MARKER_ID}

    all_rejected = _survey(survey_store, ambiguity_ratio_min=REJECT_EVERYTHING_RATIO)
    assert all_rejected == {}

    assert DEFAULT_AMBIGUITY_RATIO_MIN > 1.0
    defaulted = _survey(survey_store)
    assert defaulted == _survey(survey_store, ambiguity_ratio_min=DEFAULT_AMBIGUITY_RATIO_MIN)
    # An unambiguous, sharply-rendered view still survives the default gate.
    assert set(defaulted) == {MARKER_ID}


def test_cli_gates_by_default_and_exposes_the_ratio(monkeypatch: pytest.MonkeyPatch) -> None:
    """Invariant: `python -m dimos.mapping.loop_closure.eval` surveys with the
    gating default, and --ambiguity-ratio-min sets it — no recording needed,
    the per-recording eval is replaced by a recorder.
    """
    seen: list[float] = []

    def record(name: str, **kwargs: float) -> tuple[float, float]:
        seen.append(kwargs["ambiguity_ratio_min"])
        return (0.0, 0.0)

    monkeypatch.setattr(eval_mod, "_eval_recording", record)
    app = typer.Typer()
    app.command()(eval_mod.main)
    runner = CliRunner()

    assert runner.invoke(app, ["hk_village1"]).exit_code == 0
    assert runner.invoke(app, ["hk_village1", "--ambiguity-ratio-min", "3.5"]).exit_code == 0
    assert seen == [DEFAULT_AMBIGUITY_RATIO_MIN, 3.5]

    # Below 1.0 is not a weaker gate, it is nonsense (best beats runner-up by
    # construction); typer's min= rejects it before any recording is opened.
    assert runner.invoke(app, ["hk_village1", "--ambiguity-ratio-min", "0.5"]).exit_code != 0
    assert len(seen) == 2
