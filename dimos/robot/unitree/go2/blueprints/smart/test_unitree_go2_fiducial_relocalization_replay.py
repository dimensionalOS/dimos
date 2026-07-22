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

"""Replay integration test: the fiducial blueprint against a real recording.

The wiring tests next door prove composition on synthetic frames; this is the
next rung — the real CLI replaying hk_village3 end-to-end, asserted on the
live observables (module start, accepted relocalizations, /world_map_fix
traffic) rather than on wiring.

Opt-in only. Deselected by default (self_hosted is filtered out in addopts),
skipped on CI runners (skipif_in_ci), and skipped when the recording is not
already local — it never downloads. Exact command in the test docstring.

Both inputs are derived from the recording itself and cached under
data/replay_gate/ (gitignored): the premap via the same `dimos map global
--pgo --export` the deployment flow uses, the single-tag marker map via the
loop-closure eval pipeline (corrected_marker_transforms). Delete that dir to
force a rebuild; each build is ~10 s.
"""

from __future__ import annotations

from contextlib import suppress
import os
from pathlib import Path
import re
import signal
import subprocess

import pytest
import yaml

from dimos.core.transport import LCMTransport
from dimos.mapping.loop_closure.eval import corrected_marker_transforms
from dimos.mapping.loop_closure.pgo import PGO
from dimos.mapping.relocalization.module import Config as RelocConfig
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.robot.unitree.go2.connection import _camera_info_static
from dimos.utils.data import get_data_dir
from dimos.utils.testing.waiting import wait_until

pytestmark = [pytest.mark.self_hosted, pytest.mark.skipif_in_ci]

RECORDING = "hk_village3"
# The both-priors (RANSAC + fiducial) stack this test validates; renamed from
# unitree-go2-relocalization-fiducial (that name is now the fiducial-ONLY preset).
BLUEPRINT = "unitree-go2-relocalization-lidar-fiducial"
MARKER_LENGTH_M = 0.10  # hk_village3's printed tag edge; equals the blueprint default

# Measured on the Jul 2026 replay rehearsal of this exact command: module start
# at ~8 s, first accepted relocalization at ~42 s, first /world_map_fix fix
# inside the same minute. Budgets are ~2x those points to absorb slower CPUs.
START_DEADLINE_S = 90.0
OBSERVABLE_DEADLINE_S = 150.0
SHUTDOWN_DEADLINE_S = 30.0  # DimosCliCall's clean-shutdown budget
BUILD_TIMEOUT_S = 300.0  # premap CLI build; ~10 s measured, bounded hard

# Accept line only — rejects log as "relocalize rejected fitness=...". The
# accept line puts an optional source= right after "relocalize accepted" (the
# fiducial blueprint always tags one), so fitness is matched past an optional
# source token.
_ACCEPT_RE = re.compile(r"relocalize accepted(?: source=\S+)? fitness=([0-9.]+)")


def _cache_dir() -> Path:
    """Derived-input cache, keyed by recording stem. data/* is gitignored."""
    return get_data_dir() / "replay_gate"


@pytest.fixture(scope="module")
def recording_db() -> Path:
    """The recording, required already local — a test must never pull ~300 MB."""
    db = get_data_dir() / f"{RECORDING}.db"
    if not db.exists():
        pytest.skip(
            f"{db} not present; fetch it once with: uv run python -c "
            f"\"from dimos.utils.data import get_data; get_data('{RECORDING}.db')\""
        )
    return db


@pytest.fixture(scope="module")
def premap_file(recording_db: Path) -> Path:
    """PGO premap, built by the real `dimos map global` CLI once, then cached.

    Subprocess (not CliRunner) because --export writes ./<stem>.pc2.lcm into
    cwd and open3d state stays out of the pytest process. CPU device: this
    test must run without CUDA.
    """
    out = _cache_dir() / f"{RECORDING}.pc2.lcm"
    if out.exists():
        return out
    _cache_dir().mkdir(parents=True, exist_ok=True)
    log = _cache_dir() / f"{RECORDING}.premap_build.log"
    with log.open("wb") as fh:
        res = subprocess.run(
            [
                "dimos",
                "map",
                "global",
                str(recording_db),
                "--pgo",
                "--export",
                "--no-gui",
                "--device",
                "CPU:0",
            ],
            cwd=_cache_dir(),
            stdout=fh,
            stderr=subprocess.STDOUT,
            timeout=BUILD_TIMEOUT_S,
        )
    if res.returncode != 0 or not out.exists():
        pytest.fail(f"premap build failed (rc={res.returncode}), see {log}")
    return out


@pytest.fixture(scope="module")
def marker_map_file(recording_db: Path) -> Path:
    """Single-tag map_T_tag yaml derived from the recording itself, cached.

    Reuses the eval pipeline (corrected_marker_transforms): final smoothed
    track poses in the PGO-corrected frame — the frame the premap is exported
    in. One yaml entry per marker id: the medoid track (min summed distance
    to the others), robust to a single bad track. Knob values are eval.py's
    CLI defaults.
    """
    out = _cache_dir() / f"{RECORDING}.markers.yaml"
    if out.exists():
        return out
    store = SqliteStore(path=str(recording_db), must_exist=True)
    with store:
        graph = store.streams.lidar.transform(PGO()).last().data
        by_marker = corrected_marker_transforms(
            store,
            graph,
            camera_info=_camera_info_static(),
            marker_size=MARKER_LENGTH_M,
            marker_max_speed=0.5,
            marker_max_rot_rate=50.0,
            marker_quality_window=0.1,
            marker_smoothing=7.5,
        )
    if not by_marker:
        pytest.fail(f"no markers detected in {RECORDING}; cannot derive a marker map")

    def dist(a: Transform, b: Transform) -> float:
        da = a.translation
        db = b.translation
        return float(((da.x - db.x) ** 2 + (da.y - db.y) ** 2 + (da.z - db.z) ** 2) ** 0.5)

    entries: dict[int, dict[str, list[float]]] = {}
    for marker_id, tracks in sorted(by_marker.items()):
        best = min(tracks, key=lambda t: sum(dist(t, other) for other in tracks))
        entries[marker_id] = {
            "translation": [best.translation.x, best.translation.y, best.translation.z],
            "rotation": [best.rotation.x, best.rotation.y, best.rotation.z, best.rotation.w],
        }
    _cache_dir().mkdir(parents=True, exist_ok=True)
    out.write_text(yaml.safe_dump({"markers": entries}))
    return out


def test_replay_relocalizes_and_publishes_fixes(premap_file: Path, marker_map_file: Path) -> None:
    """Replaying hk_village3 through the fiducial blueprint relocalizes for real.

    Invariant: `dimos --replay --replay-db=hk_village3 run
    unitree-go2-relocalization-lidar-fiducial` with a premap derived from that
    same recording (a) starts RelocalizationModule with the premap,
    (b) accepts >= 1 relocalization whose fitness clears the configured
    threshold within the observation window, (c) publishes >= 1 world->map
    Transform on the /world_map_fix LCM stream (asserted by an in-process
    subscriber, not by log-scraping), (d) emits zero tracebacks, and (e)
    shuts down within 30 s of SIGTERM.

    Opt-in (needs data/hk_village3.db present; see module docstring):

        uv run pytest dimos/robot/unitree/go2/blueprints/smart/\
test_unitree_go2_fiducial_relocalization_replay.py -m self_hosted
    """
    log_path = _cache_dir() / f"{RECORDING}.replay_run.log"

    def log_text() -> str:
        return log_path.read_text(errors="replace") if log_path.exists() else ""

    fixes: list[Transform] = []
    fix_transport: LCMTransport[Transform] = LCMTransport("/world_map_fix", Transform)
    unsubscribe = fix_transport.subscribe(fixes.append)
    proc: subprocess.Popen[bytes] | None = None
    try:
        with log_path.open("wb") as fh:
            # start_new_session puts the CLI and its module workers in one
            # process group so teardown can kill them all by the exact pgid.
            #
            # Only map_file is a flat `-o` override now: the fiducial params moved
            # into the priors list (relocalizationmodule.priors[i]), which the
            # dotted override parser can't index. RANSAC (also enabled in
            # lidar-fiducial) carries the reloc for this test's assertions; a real
            # marker survey needs a preset that bakes it in (follow-up). marker_map_
            # file is still built above to exercise its derivation pipeline.
            overrides = [
                f"relocalizationmodule.map_file={premap_file}",
            ]
            proc = subprocess.Popen(
                ["dimos", "--replay", f"--replay-db={RECORDING}", "run", BLUEPRINT]
                + [arg for override in overrides for arg in ("-o", override)],
                stdout=fh,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )

        # (a) the module came up with our premap, not disabled/no-op.
        wait_until(
            lambda: "relocalization module started" in log_text(),
            timeout=START_DEADLINE_S,
            interval=0.5,
            message=f"RelocalizationModule never started; see {log_path}",
        )
        assert f"map_file={premap_file}" in log_text()

        # (b) + (c) an accepted relocalization in the log AND a fix on the wire.
        wait_until(
            lambda: bool(_ACCEPT_RE.search(log_text())) and len(fixes) >= 1,
            timeout=OBSERVABLE_DEADLINE_S,
            interval=0.5,
            message=(
                f"no accepted relocalization + published fix within "
                f"{OBSERVABLE_DEADLINE_S}s; see {log_path}"
            ),
        )

        # (e) clean shutdown: SIGTERM the group, gone within the budget.
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=SHUTDOWN_DEADLINE_S)
    finally:
        if proc is not None and proc.poll() is None:
            with suppress(ProcessLookupError):
                os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        unsubscribe()
        fix_transport.stop()

    text = log_text()
    # (b) every accepted fitness clears the module's own configured threshold.
    fitnesses = [float(m) for m in _ACCEPT_RE.findall(text)]
    assert fitnesses, "accept line vanished from the log after shutdown"
    # priors is a required field now, so read the documented default off the field
    # rather than constructing a bare Config (the blueprint leaves the threshold at
    # its default).
    assert min(fitnesses) >= RelocConfig.model_fields["fitness_threshold"].default
    # (c) the fix stream carries the world->map convention end to end.
    assert (fixes[0].frame_id, fixes[0].child_frame_id) == ("world", "map")
    # (d) nothing raised anywhere in the run, shutdown included.
    assert "Traceback (most recent call last)" not in text, f"traceback in {log_path}"
