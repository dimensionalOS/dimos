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

"""Exercise raw sensor export and the real shell isolation boundary."""

import json
from pathlib import Path
import socket
import subprocess
from typing import Any

import numpy as np
from PIL import Image as PILImage
import pytest

from dimos.evals.agents.lib.plain_recording import plain_recording
from dimos.evals.agents.lib.sandbox import sandbox_command
from dimos.evals.agents.pi import PiAdapter
from dimos.evals.types import RunningEnvironment
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def test_export_preserves_selected_sensor_values_without_framework(tmp_path: Path) -> None:
    pixels = np.array([[[10, 20, 30], [40, 50, 60]]], dtype=np.uint8)
    points = np.array([[1.125, -2.5, 3.0], [4.0, 5.0, 6.0]])
    with SqliteStore(path=tmp_path / "source.db") as store:
        lidar = store.stream("lidar", PointCloud2)
        lidar.append(PointCloud2.from_numpy(points, frame_id="world", timestamp=1.0), ts=1.0)
        lidar.append(PointCloud2.from_numpy(points * 2, timestamp=2.0), ts=2.0)
        images = store.stream("camera", Image)
        images.append(Image.from_numpy(pixels, format=ImageFormat.RGB), ts=3.0)
        store.stream("secret", str).append("grader-only")
        stored_pixels = images.first().data.to_rgb().data
        manifest = plain_recording((lidar.limit(1), images), tmp_path / "input")
    records = json.loads(manifest.read_text())["observations"]
    assert [record["stream"] for record in records] == ["lidar", "camera"]
    np.testing.assert_array_equal(
        np.loadtxt(manifest.parent / records[0]["file"], delimiter=",", skiprows=1), points
    )
    np.testing.assert_array_equal(
        PILImage.open(manifest.parent / records[1]["file"]), stored_pixels
    )
    assert not any(path.suffix in {".db", ".pkl", ".py"} for path in manifest.parent.iterdir())


def test_baseline_does_not_expose_artifacts_or_mcp(tmp_path: Path) -> None:
    agent = PiAdapter(allowed_tools=("bash",), sandbox=True)
    with pytest.raises(ValueError, match="MCP"):
        agent._prepare_files(
            RunningEnvironment(mcp_url="http://localhost:9990", streams=(), artifacts={}), tmp_path
        )
    with SqliteStore(path=tmp_path / "source.db") as store:
        selected = store.stream("observed", str)
        selected.append("a fact")
        files = agent._prepare_files(
            RunningEnvironment(
                mcp_url="", streams=(selected,), artifacts={"hidden": tmp_path / "source.db"}
            ),
            tmp_path,
        )
    assert files == {"manifest": Path("/input/manifest.json")}
    assert agent.available_tools(("move", "render_pointcloud")) == ("bash",)
    assert "hidden" not in (tmp_path / "input" / "manifest.json").read_text()


@pytest.mark.parametrize(
    "config",
    [
        dict(allowed_tools=("bash", "read")),
        dict(skills=("x",)),
        dict(modules=("mcp-server",)),
    ],
)
def test_policy_rejects_extra_capabilities(config: dict[str, Any]) -> None:
    with pytest.raises(ValueError):
        PiAdapter(sandbox=True, **config)


@pytest.mark.skipif(not Path("/usr/bin/bwrap").is_file(), reason="requires Linux bubblewrap")
def test_shell_can_work_but_cannot_reach_host_or_dimos(tmp_path: Path) -> None:
    inputs, workspace = tmp_path / "input", tmp_path / "workspace"
    inputs.mkdir()
    workspace.mkdir()
    (inputs / "facts.txt").write_text("alpha\nbeta\n")
    secret = tmp_path / "host-only.txt"
    secret.write_text("private")
    with socket.socket() as server:
        server.bind(("127.0.0.1", 0))
        server.listen()
        port = server.getsockname()[1]
        script = f"""
set -eu
grep beta /input/facts.txt > /workspace/found.txt
test ! -e {secret}
test ! -e /home/stash
test ! -e /usr/local/bin/dimos
test -z "${{OPENAI_API_KEY:-}}"
test -z "${{DIMOS_ROBOT_IP:-}}"
! command -v dimos
! touch /input/forbidden
python3 -I - <<'PY'
import importlib.util, socket
assert importlib.util.find_spec('dimos') is None
s = socket.socket()
s.settimeout(0.1)
assert s.connect_ex(('127.0.0.1', {port})) != 0
PY
"""
        result = subprocess.run(
            [*sandbox_command(inputs, workspace), script],
            text=True,
            capture_output=True,
            timeout=10,
            env={"OPENAI_API_KEY": "test-secret", "DIMOS_ROBOT_IP": "hidden"},
        )
    assert result.returncode == 0, result.stderr
    assert (workspace / "found.txt").read_text() == "beta\n"
    assert not (inputs / "forbidden").exists()


def test_shell_environment_excludes_dimos(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("DIMOS_ROBOT_IP", "hidden")
    env = PiAdapter(allowed_tools=("bash",), sandbox=True)._build_process_env(tmp_path)
    assert "DIMOS_ROBOT_IP" not in env
