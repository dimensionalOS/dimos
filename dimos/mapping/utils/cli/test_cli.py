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

"""End-to-end tests for the `dimos map` verbs, run in-process.

Each test invokes the real CLI against the `go2_short` recording (60s, auto-
pulled via LFS) and asserts on the artifact it produces. A short `--duration`
snippet keeps every invocation to a few seconds, and `--no-gui` stops the rrd
verbs from spawning a rerun viewer.

The CLI is invoked in-process via Typer's CliRunner (not a subprocess per case),
so the heavy dimos import is paid once for the whole module instead of per case.
"""

from __future__ import annotations

from pathlib import Path
import traceback
from types import SimpleNamespace

import pytest
from typer.testing import CliRunner

# These drive the real CLI against an LFS recording (CPU voxel accumulation,
# multi-second runs) — self-hosted runner only.
pytestmark = pytest.mark.self_hosted

# A few seconds in, then a couple seconds long — small enough to stay fast,
# long enough that the robot moves (so dedup/PGO/markers have something to do).
SEEK = 4.0
DURATION = 3.0

# go2_short has 461 lidar frames over ~60s; a 3s snippet must be far fewer.
FULL_LIDAR_COUNT = 461

_runner = CliRunner()


def _turbojpeg_available() -> bool:
    """True if the native libturbojpeg can be loaded (decodes the color_image stream)."""
    try:
        from turbojpeg import TurboJPEG

        TurboJPEG()
    except Exception:
        return False
    return True


# The color_image stream is JPEG; decoding it needs the native libturbojpeg, which
# is missing from stale CI images. Skip the verbs that touch images when it's absent.
requires_turbojpeg = pytest.mark.skipif(
    not _turbojpeg_available(),
    reason="native libturbojpeg unavailable; cannot decode the JPEG color_image stream",
)


def _run(*args: str, timeout: float = 300.0) -> SimpleNamespace:
    """Invoke `dimos map <args>` in-process and capture its result.

    Uses Typer's CliRunner so the dimos import cost is paid once (module import)
    rather than per case. `timeout` is kept for call-site compatibility but is a
    no-op here — pytest-timeout covers hangs. Returns a result with the
    .returncode/.stdout/.stderr fields the test bodies use.
    """
    from dimos.robot.cli.dimos import main as cli_app

    res = _runner.invoke(cli_app, ["map", *args])
    err = res.output
    if res.exception is not None and not isinstance(res.exception, SystemExit):
        err += "\n" + "".join(traceback.format_exception(res.exception))
    return SimpleNamespace(returncode=res.exit_code, stdout=res.output, stderr=err)


def _stream_counts(db_path: Path) -> dict[str, int]:
    from dimos.memory2.store.sqlite import SqliteStore

    store = SqliteStore(path=str(db_path), must_exist=True)
    try:
        return {name: store.stream(name).count() for name in store.list_streams()}
    finally:
        store.stop()


@pytest.fixture(scope="module")
def dataset() -> str:
    """Ensure the go2_short recording is present locally; return its bare name."""
    from dimos.utils.data import get_data

    get_data("go2_short.db")
    return "go2_short"


def test_summary(dataset: str) -> None:
    res = _run("summary", dataset)
    assert res.returncode == 0, res.stderr
    assert "lidar" in res.stdout
    assert "odom" in res.stdout


@requires_turbojpeg
def test_rename_snippet(dataset: str, tmp_path: Path) -> None:
    out = tmp_path / "renamed.db"
    res = _run(
        "rename",
        dataset,
        "--out",
        str(out),
        "--rename",
        "odom=odometry",
        "--duration",
        str(DURATION),
    )
    assert res.returncode == 0, res.stderr
    assert out.exists()

    counts = _stream_counts(out)
    # Stream was renamed, not duplicated.
    assert "odometry" in counts
    assert "odom" not in counts
    assert "lidar" in counts
    # --duration actually clipped the copy.
    assert 0 < counts["lidar"] < FULL_LIDAR_COUNT


def test_pose_fill_snippet(dataset: str, tmp_path: Path) -> None:
    out = tmp_path / "posed.db"
    res = _run(
        "pose-fill",
        dataset,
        "--out",
        str(out),
        "--streams",
        "lidar,odom",
        "--duration",
        str(DURATION),
    )
    assert res.returncode == 0, res.stderr
    assert out.exists()

    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    store = SqliteStore(path=str(out), must_exist=True)
    try:
        lidar = store.stream("lidar", PointCloud2)
        assert 0 < lidar.count() < FULL_LIDAR_COUNT
        # The target stream carries a baked pose after pose-fill.
        assert lidar.first().pose is not None
    finally:
        store.stop()


@requires_turbojpeg
def test_replay_snippet(dataset: str, tmp_path: Path) -> None:
    out = tmp_path / "replay.rrd"
    res = _run("replay", dataset, "--out", str(out), "--no-gui", "--duration", str(DURATION))
    assert res.returncode == 0, res.stderr
    assert out.exists() and out.stat().st_size > 0


@requires_turbojpeg
def test_replay_marker_snippet(dataset: str, tmp_path: Path) -> None:
    out = tmp_path / "markers.rrd"
    res = _run("replay-marker", dataset, "--out", str(out), "--no-gui", "--duration", str(DURATION))
    assert res.returncode == 0, res.stderr
    assert out.exists() and out.stat().st_size > 0


@requires_turbojpeg
def test_replay_map_final_snippet(dataset: str, tmp_path: Path) -> None:
    out = tmp_path / "replay_map.rrd"
    # CPU device keeps the voxel accumulation runnable without a GPU.
    res = _run(
        "replay",
        dataset,
        "--out",
        str(out),
        "--no-gui",
        "--duration",
        str(DURATION),
        "--map-final",
        "--map-carve-columns",
        "--map-device",
        "CPU:0",
    )
    assert res.returncode == 0, res.stderr
    assert out.exists() and out.stat().st_size > 0


def test_global_snippet(dataset: str, tmp_path: Path) -> None:
    out = tmp_path / "global.rrd"
    # CPU device + small block budget keeps this runnable without a GPU.
    # --seek exercises the snippet offset alongside --duration.
    res = _run(
        "global",
        dataset,
        "--out",
        str(out),
        "--no-gui",
        "--seek",
        str(SEEK),
        "--duration",
        str(DURATION),
        "--device",
        "CPU:0",
        "--block-count",
        "100000",
        timeout=600.0,
    )
    assert res.returncode == 0, res.stderr
    assert out.exists() and out.stat().st_size > 0
