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

"""Command-source choice, declarations, and the tracker mount."""

from __future__ import annotations

import json

import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.sysid.ingest import (
    Declarations,
    command_coverage,
    mount_matrix,
    read_declarations,
    sidecar_path,
)


def test_the_command_source_is_chosen_by_time_coverage_not_presence():
    """A 60 s sport recording can carry an 850-message stub of policy/lowcmd
    beside 23520 on rt/lowcmd; picking the stub silently replays two seconds
    of a sixty-second file and reports a superb score for it."""
    stub = command_coverage(0.0, 2.1, 850, 60.0)  # the stub: present, tiny
    full = command_coverage(0.2, 59.8, 23520, 60.0)
    assert stub < 0.5 <= full


def test_a_single_message_covers_nothing():
    assert command_coverage(5.0, 5.0, 1, 60.0) == 0.0


def test_a_zero_span_does_not_divide_by_zero():
    assert command_coverage(0.0, 1.0, 10, 0.0) == pytest.approx(1.0)


def test_the_mount_matrix_is_a_rotation():
    m = mount_matrix()
    assert np.allclose(m @ m.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(m) == pytest.approx(1.0)


def test_flip_inverts_the_trackers_z():
    up = mount_matrix(90.0, flip=False)
    down = mount_matrix(90.0, flip=True)
    assert np.allclose(up[:, 2], [0, 0, 1])
    assert np.allclose(down[:, 2], [0, 0, -1])


def _write_mcap(path, metadata: dict[str, str] | None) -> None:
    mcap_writer = pytest.importorskip("mcap.writer")
    with open(path, "wb") as f:
        w = mcap_writer.Writer(f)
        w.start()
        if metadata is not None:
            w.add_metadata("go2sim", metadata)
        w.finish()


def test_declarations_ride_in_the_mcap_itself(tmp_path):
    p = tmp_path / "rec.mcap"
    _write_mcap(p, {"suspended": "true", "floor": "wood"})
    got = read_declarations(p)
    assert got == Declarations(suspended=True, floor="wood")


def test_a_sidecar_serves_recordings_that_predate_the_recorder_change(tmp_path):
    p = tmp_path / "rec.mcap"
    _write_mcap(p, None)
    sidecar_path(p).write_text(json.dumps({"suspended": True, "floor": "rubber_mat"}))
    got = read_declarations(p)
    assert got == Declarations(suspended=True, floor="rubber_mat")


def test_the_mcap_record_outranks_the_sidecar(tmp_path):
    """The metadata travels with the file and cannot be separated from it."""
    p = tmp_path / "rec.mcap"
    _write_mcap(p, {"suspended": "false"})
    sidecar_path(p).write_text(json.dumps({"suspended": True}))
    got = read_declarations(p)
    assert got.suspended is False


def test_nothing_declared_is_none_not_false(tmp_path):
    """None means 'nobody said': regimes may refuse on it, a False must not."""
    p = tmp_path / "rec.mcap"
    _write_mcap(p, None)
    assert read_declarations(p) == Declarations(suspended=None, floor=None)


def test_the_sidecar_is_named_after_the_recording(tmp_path):
    assert sidecar_path(tmp_path / "a" / "20260816-x.mcap").name == "20260816-x.meta.json"
