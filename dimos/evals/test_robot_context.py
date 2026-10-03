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

import hashlib
import json
from pathlib import Path

import pytest

from dimos.evals.agents.pi import PiAdapter
from dimos.evals.robot_context import robot_context_files, stage_robot_context
from dimos.evals.types import RunningEnvironment


def bundle(root: Path) -> Path:
    root.mkdir()
    contents = {
        "README.md": "Robot-only context",
        "robot.urdf": '<robot name="arm"><link name="base"/></robot>',
        "gripper.urdf": '<robot name="gripper"><link name="gripper"/></robot>',
        "robot_info.json": '{"gripper":{"command_unit":"normalized"}}',
        "meshes/finger.stl": "robot mesh",
    }
    for name, content in contents.items():
        target = root / name
        target.parent.mkdir(exist_ok=True, parents=True)
        target.write_text(content)
    manifest = {
        "files": {
            name: hashlib.sha256(content.encode()).hexdigest() for name, content in contents.items()
        }
    }
    (root / "manifest.json").write_text(json.dumps(manifest))
    return root


def test_only_declared_robot_assets_are_staged(tmp_path):
    source = bundle(tmp_path / "source")
    (source / "scene.xml").write_text("private object positions")
    files = stage_robot_context(source, tmp_path / "case" / "robot")
    assert set(files) == {"robot_context", "robot_urdf", "gripper_urdf", "robot_info"}
    assert (tmp_path / "case/robot/meshes/finger.stl").read_text() == "robot mesh"
    assert not (tmp_path / "case/robot/scene.xml").exists()


def test_changed_asset_rejected_before_copy(tmp_path):
    source = bundle(tmp_path / "source")
    (source / "meshes/finger.stl").write_text("different geometry")
    target = tmp_path / "target"
    with pytest.raises(ValueError, match="checksum mismatch"):
        stage_robot_context(source, target)
    assert not target.exists()


@pytest.mark.parametrize("name", ["../secret", "/tmp/secret", "scene.xml"])
def test_manifest_cannot_name_scene_or_external_files(tmp_path, name):
    source = bundle(tmp_path / "source")
    manifest = json.loads((source / "manifest.json").read_text())
    manifest["files"][name] = "0" * 64
    (source / "manifest.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError):
        robot_context_files(source)


def test_agent_receives_context_inside_its_workspace_without_recording_access(tmp_path):
    source = bundle(tmp_path / "source")
    run_dir = tmp_path / "case"
    run_dir.mkdir()
    agent = PiAdapter(no_dimos=True)
    prompt = agent._prepare_case(
        RunningEnvironment(
            mcp_url="unused",
            streams=(),
            artifacts={"recording": tmp_path / "private.db"},
            raw_endpoint="tcp/127.0.0.1:12345",
            raw_interface="manipulation",
            robot_context=source,
        ),
        run_dir,
    )
    assert str(run_dir / "robot/robot.urdf") in prompt
    assert str(run_dir / "robot/gripper.urdf") in prompt
    assert str(source) not in prompt
    assert "private.db" not in prompt
    assert (run_dir / "robot/robot_info.json").is_file()
    assert "read robot/README.md and robot/robot_info.json" in (run_dir / "ROBOT.md").read_text()
