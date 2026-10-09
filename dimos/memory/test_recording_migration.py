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

"""Public migration CLI never overwrites or silently skips unsupported recordings."""

import json

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import pytest
from typer.testing import CliRunner

from dimos.memory.cli.app import mem_app
from dimos.memory.test_convert_recording import read_mcap, write_mcap
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped as LegacyPoseStamped


def source_file(path, payload=b'"hello"'):
    path.parent.mkdir(parents=True, exist_ok=True)
    write_mcap(path, [("text", "dimos.msgs.std_msgs.String.String", "json", [payload])])


def test_cli_single_and_batch_dry_run_then_conversion(tmp_path):
    source = tmp_path / "input"
    source_file(source / "nested" / "run.mcap")
    destination = tmp_path / "output"
    runner = CliRunner()
    dry = runner.invoke(mem_app, ["convert", str(source), str(destination), "--dry-run"])
    assert dry.exit_code == 0, dry.output
    assert json.loads(dry.stdout)["files"][0]["total"] == 1
    assert not destination.exists()
    done = runner.invoke(mem_app, ["convert", str(source), str(destination)])
    assert done.exit_code == 0, done.output
    output = destination / "nested" / "run.mcap.cdr.mcap"
    assert len(read_mcap(output)) == 1
    summary = json.loads((destination / "migration-summary.json").read_text())
    assert summary["files"][0]["status"] == "converted"
    single = runner.invoke(mem_app, ["convert", str(output), str(tmp_path / "single.db")])
    assert single.exit_code == 0, single.output
    assert (tmp_path / "single.db").is_file()
    repeat = runner.invoke(mem_app, ["convert", str(source), str(destination)])
    assert repeat.exit_code == 2
    assert len(read_mcap(output)) == 1


@pytest.mark.parametrize(
    "blocked", ["unknown.pickle", "archive.tar.gz", "capture.pcap", "pointer.mcap"]
)
def test_batch_reports_blockers_and_publishes_nothing(tmp_path, blocked):
    source = tmp_path / "input"
    source_file(source / "good.mcap")
    (source / blocked).write_bytes(b"version https://git-lfs.github.com/spec/v1\n")
    destination = tmp_path / "output"
    result = CliRunner().invoke(mem_app, ["convert", str(source), str(destination)])
    assert result.exit_code == 2, result.output
    assert json.loads(result.stdout)["blocked"] == 1
    assert not destination.exists()


def test_batch_payload_failure_reports_partial_results(tmp_path):
    source = tmp_path / "input"
    source_file(source / "a.mcap")
    source_file(source / "b.mcap", b"not json")
    source_file(source / "c.mcap")
    destination = tmp_path / "output"
    result = CliRunner().invoke(mem_app, ["convert", str(source), str(destination)])
    assert result.exit_code == 2, result.output
    summary = json.loads((destination / "migration-summary.json").read_text())
    assert [entry["status"] for entry in summary["files"]] == [
        "converted",
        "failed",
        "not_attempted",
    ]
    assert not (destination / "b.mcap.cdr.mcap").exists()
    assert not (destination / "c.mcap.cdr.mcap").exists()


def test_overlap_symlinks_and_invalid_single_output_are_rejected(tmp_path):
    source = tmp_path / "input"
    source_file(source / "good.mcap")
    runner = CliRunner()
    overlap = runner.invoke(mem_app, ["convert", str(source), str(source / "out")])
    assert overlap.exit_code == 2
    (source / "link").symlink_to(tmp_path, target_is_directory=True)
    linked = runner.invoke(mem_app, ["convert", str(source), str(tmp_path / "out")])
    assert linked.exit_code == 2
    assert json.loads(linked.stdout)["blocked"] == 1
    invalid = runner.invoke(
        mem_app, ["convert", str(source / "good.mcap"), str(tmp_path / "out.txt"), "--dry-run"]
    )
    assert invalid.exit_code == 2


def test_retained_legacy_interface_is_explicit_and_cdr_remains_distinct():
    old = LegacyPoseStamped(ts=1.5, frame_id="world", position=(3.25, 0, 0))
    restored = LegacyPoseStamped.lcm_decode(old.lcm_encode())
    assert (restored.ts, restored.frame_id, restored.position.x) == (1.5, "world", 3.25)
    current = PoseStamped(
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        pose=Pose(
            position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        ),
    )
    assert cdr_decode(cdr_encode(current), PoseStamped) == current
    with pytest.raises(ValueError):
        cdr_decode(old.lcm_encode(), PoseStamped)
