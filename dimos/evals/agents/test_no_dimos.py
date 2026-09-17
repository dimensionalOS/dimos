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

"""What a Pi agent without dimOS is handed, and what it is not."""

import json
import os
from pathlib import Path

import numpy as np
from PIL import Image as PILImage
import pytest

from dimos.evals.agents.lib.pi_config import RunPaths
from dimos.evals.agents.lib.plain_recording import plain_recording
from dimos.evals.agents.pi import NO_DIMOS_KEYWORDS, PiAdapter
from dimos.evals.types import RunningEnvironment
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def test_no_dimos_defaults_the_excluded_keywords_but_keeps_explicit_ones() -> None:
    assert PiAdapter(no_dimos=True).config.excluded_keywords == NO_DIMOS_KEYWORDS
    assert PiAdapter(no_dimos=True, excluded_keywords=("Acme",)).config.excluded_keywords == (
        "acme",
    )
    assert PiAdapter().config.excluded_keywords == ()
    with pytest.raises(ValueError, match="nonempty words"):
        PiAdapter(excluded_keywords=("dim os",))
    with pytest.raises(ValueError, match="modules or Pi skills"):
        PiAdapter(no_dimos=True, modules=("mcp-server",))
    with pytest.raises(ValueError, match="modules or Pi skills"):
        PiAdapter(no_dimos=True, skills=("dimensional/SKILL.md",))


def test_robot_environment_needs_raw_topics(tmp_path: Path) -> None:
    agent = PiAdapter(no_dimos=True)
    live = RunningEnvironment(mcp_url="http://localhost:9990/mcp", streams=(), artifacts={})
    with pytest.raises(ValueError, match="raw_bridge"):
        agent._prepare_case(live, tmp_path)
    prompt = agent._prepare_case(
        RunningEnvironment(
            mcp_url="http://localhost:9990/mcp",
            streams=(),
            artifacts={"recording": tmp_path / "memory.db"},
            raw_endpoint="tcp/127.0.0.1:7448",
        ),
        tmp_path,
    )
    readme = (tmp_path / "ROBOT.md").read_text()
    assert "tcp/127.0.0.1:7448" in readme and "robot/cmd_vel/json" in readme
    assert "ROBOT.md" in prompt and "memory.db" not in prompt
    assert "dimos mcp call" not in prompt and "dimensionalOS" in prompt
    # No MCP tool listing was fetched: there is nothing for the agent to call through dimOS.
    assert "Tools:" not in prompt
    assert agent.available_tools(("move_to", "wait")) == PiAdapter.default_tools


def test_recorded_observations_are_exported_as_plain_files(tmp_path: Path) -> None:
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


def test_prepared_case_hides_the_memory_store(tmp_path: Path) -> None:
    agent = PiAdapter(no_dimos=True)
    with SqliteStore(path=tmp_path / "source.db") as store:
        selected = store.stream("observed", str)
        selected.append("a fact")
        prompt = agent._prepare_case(
            RunningEnvironment(
                mcp_url="",
                streams=(selected,),
                artifacts={"recording": tmp_path / "source.db", "notes": tmp_path / "notes.txt"},
            ),
            tmp_path,
        )
    assert "input/manifest.json" in prompt and "notes.txt" in prompt
    assert "source.db" not in prompt and "SqliteStore" not in prompt
    assert not (tmp_path / "recording.db").exists()


def test_process_env_drops_dimos_paths_and_variables(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    venv_bin = tmp_path / "checkout" / ".venv" / "bin"
    venv_bin.mkdir(parents=True)
    (venv_bin / "dimos").touch()
    named = tmp_path / "dimos" / "bin"
    named.mkdir(parents=True)
    plain = tmp_path / "tools"
    plain.mkdir()
    monkeypatch.setenv("PATH", os.pathsep.join(map(str, (venv_bin, named, plain, "/usr/bin"))))
    monkeypatch.setenv("PYTHONPATH", str(tmp_path / "checkout"))
    monkeypatch.setenv("VIRTUAL_ENV", str(tmp_path / "checkout" / ".venv"))
    monkeypatch.setenv("DIMOS_TRANSPORT", "lcm")
    paths = RunPaths.for_run(tmp_path / "run")

    env = PiAdapter(no_dimos=True)._build_process_env(paths)
    assert env["PATH"].split(os.pathsep) == [str(plain), "/usr/bin"]
    assert "PYTHONPATH" not in env and "VIRTUAL_ENV" not in env and "DIMOS_TRANSPORT" not in env
    assert env["DIMOS_EVAL_RUN_ID"] == str(paths.workspace)

    kept = PiAdapter()._build_process_env(paths)
    assert str(venv_bin) in kept["PATH"] and kept["DIMOS_TRANSPORT"] == "lcm"


def test_pi_is_launched_by_absolute_path_when_it_shares_a_dir_with_dimos(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    shared = tmp_path / "bin"
    shared.mkdir()
    for name in ("pi", "dimos"):
        (shared / name).write_text("#!/bin/sh\n")
        (shared / name).chmod(0o755)
    monkeypatch.setenv("PATH", os.pathsep.join((str(shared), "/usr/bin")))
    agent = PiAdapter(no_dimos=True, cli="pi")
    paths = RunPaths.for_run(tmp_path / "run")
    command = agent._build_pi_command("inputs", "prompt", paths)
    assert command[0] == str(shared / "pi")
    assert str(shared) not in agent._build_process_env(paths)["PATH"]
