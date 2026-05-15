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

"""CLI input-validation tests for ``scripts/datasets/rrd_to_lerobot.py``.

These tests exercise the validation path only — they never reach the lerobot
or rerun.dataframe imports, so they pass without the ``datasets`` extra
installed.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest

# The script lives outside the dimos package, so import it via spec to keep
# the test self-contained.
_SCRIPT_PATH = Path(__file__).resolve().parents[3] / "scripts" / "datasets" / "rrd_to_lerobot.py"


def _load_script():
    spec = importlib.util.spec_from_file_location("rrd_to_lerobot", _SCRIPT_PATH)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules["rrd_to_lerobot"] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def script():
    return _load_script()


def test_missing_both_task_flags_fails(script, tmp_path):
    rrd = tmp_path / "episode_001.rrd"
    rrd.write_bytes(b"")
    parser = script._build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(["--input", str(rrd)])


def test_both_task_flags_fails(script, tmp_path):
    rrd = tmp_path / "episode_001.rrd"
    rrd.write_bytes(b"")
    tasks = tmp_path / "t.jsonl"
    tasks.write_text('{"episode": "episode_001", "task": "x"}\n')
    parser = script._build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(
            [
                "--input",
                str(rrd),
                "--task",
                "x",
                "--task-per-episode",
                str(tasks),
            ]
        )


def test_threshold_above_one_fails(script):
    with pytest.raises(ValueError, match=r"\[0\.0, 1\.0\]"):
        script._validate_threshold(1.5)


def test_threshold_below_zero_fails(script):
    with pytest.raises(ValueError, match=r"\[0\.0, 1\.0\]"):
        script._validate_threshold(-0.01)


def test_threshold_in_range_ok(script):
    script._validate_threshold(0.0)
    script._validate_threshold(1.0)
    script._validate_threshold(0.7)
    script._validate_threshold(None)  # None means use contract default


def test_missing_input_path_fails(script):
    with pytest.raises(FileNotFoundError):
        script._resolve_input_files(Path("/does/not/exist"))


def test_input_file_wrong_extension_fails(script, tmp_path):
    not_rrd = tmp_path / "session.tar"
    not_rrd.write_bytes(b"")
    with pytest.raises(ValueError, match="not a .rrd"):
        script._resolve_input_files(not_rrd)


def test_input_dir_with_no_rrd_files_fails(script, tmp_path):
    with pytest.raises(FileNotFoundError, match="no .rrd"):
        script._resolve_input_files(tmp_path)


def test_input_single_rrd_file(script, tmp_path):
    rrd = tmp_path / "episode_001.rrd"
    rrd.write_bytes(b"")
    name, files = script._resolve_input_files(rrd)
    assert name == "episode_001"
    assert files == [rrd]


def test_input_session_dir_sorted_episodes(script, tmp_path):
    (tmp_path / "episode_001.rrd").write_bytes(b"")
    (tmp_path / "episode_003.rrd").write_bytes(b"")
    (tmp_path / "episode_002.rrd").write_bytes(b"")
    name, files = script._resolve_input_files(tmp_path)
    assert name == tmp_path.name
    assert [f.name for f in files] == ["episode_001.rrd", "episode_002.rrd", "episode_003.rrd"]


def test_task_per_episode_loads_jsonl(script, tmp_path):
    f = tmp_path / "tasks.jsonl"
    f.write_text(
        '{"episode": "episode_001", "task": "pick"}\n{"episode": "episode_002", "task": "place"}\n'
    )
    tasks = script._load_task_per_episode(f, ["episode_001", "episode_002"])
    assert tasks == {"episode_001": "pick", "episode_002": "place"}


def test_task_per_episode_missing_entry_fails(script, tmp_path):
    f = tmp_path / "tasks.jsonl"
    f.write_text('{"episode": "episode_001", "task": "pick"}\n')
    with pytest.raises(KeyError, match="episode_002"):
        script._load_task_per_episode(f, ["episode_001", "episode_002"])


def test_task_per_episode_malformed_json_fails(script, tmp_path):
    f = tmp_path / "tasks.jsonl"
    f.write_text("not json\n")
    with pytest.raises(ValueError, match="malformed JSON"):
        script._load_task_per_episode(f, [])


def test_task_per_episode_missing_fields_fails(script, tmp_path):
    f = tmp_path / "tasks.jsonl"
    f.write_text('{"task": "no episode key"}\n')
    with pytest.raises(ValueError, match="'episode' and 'task'"):
        script._load_task_per_episode(f, [])


def test_apply_gripper_overrides_does_not_mutate_contract(script):
    from dimos.manipulation.policy.contracts.registry import get_contract

    original = get_contract("piper")
    original_threshold = original.gripper_binarization.threshold
    original_enabled = original.gripper_binarization.enabled

    overridden = script._apply_gripper_overrides(original, binarize=False, threshold=0.42)

    assert overridden.gripper_binarization.threshold == 0.42
    assert overridden.gripper_binarization.enabled is False
    # Original contract is untouched.
    assert original.gripper_binarization.threshold == original_threshold
    assert original.gripper_binarization.enabled == original_enabled
