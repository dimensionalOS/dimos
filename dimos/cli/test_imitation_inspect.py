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


from io import StringIO

import pytest
from rich.console import Console

from dimos.cli.imitation_inspect import print_inspection


@pytest.fixture
def recording_info():
    return {
        "format": "recording",
        "path": "/recordings/session-001/recording.mcap",
        "streams": {"coordinator_joint_state": 6206, "status": 2, "wrist_image": 1953},
        "status_stream": "status",
        "episodes": 1,
        "saved_episodes": 1,
        "discarded_episodes": 0,
        "incomplete_episodes": [],
        "quality": [
            {
                "episode_id": "ep_000000",
                "valid": True,
                "mode": "strict",
                "expected_frames": 673,
                "emitted_frames": 673,
                "filled_frames": 0,
                "source_rates_hz": {"observation.images.wrist": 30.000210029462686},
                "max_gaps_ms": {"observation.images.wrist": 34.532785415649414},
                "max_alignment_error_ms": 5.251407623291016,
                "rejection_reasons": [],
            }
        ],
    }


def render(info, *, verbose=False, width=100):
    output = StringIO()
    print_inspection(
        info, console=Console(file=output, width=width, color_system=None), verbose=verbose
    )
    return output.getvalue()


def test_recording_summary_rounds_metrics_and_omits_successful_episode_details(recording_info):
    output = render(recording_info)
    for value in (
        "Recording · session-001",
        "1 saved",
        "6,206",
        "1,953",
        "PASS",
        "673 emitted / 673 expected",
        "0 filled",
        "30.00 Hz",
        "34.53 ms",
        "5.25 ms",
    ):
        assert value in output
    assert "ep_000000" not in output
    assert "30.000210029462686" not in output


def test_summary_aggregates_metrics_and_always_shows_issues(recording_info):
    failed = {
        **recording_info["quality"][0],
        "episode_id": "ep_000001",
        "valid": False,
        "emitted_frames": 600,
        "filled_frames": 5,
        "source_rates_hz": {"observation.images.wrist": 20.0},
        "max_gaps_ms": {"observation.images.wrist": 100.0},
        "max_alignment_error_ms": 25.0,
        "rejection_reasons": ["Missing action samples"],
    }
    recording_info.update(saved_episodes=2, episodes=3, discarded_episodes=1)
    recording_info["quality"].append(failed)
    recording_info["incomplete_episodes"] = [{"start_ts": 123.456, "task_label": "pick [cube]"}]
    output = render(recording_info)
    for value in (
        "2 saved · 1 discarded · 1 incomplete",
        "FAIL",
        "1/2 assessed",
        "1,273 emitted / 1,346 expected",
        "5 filled",
        "20.00-30.00 Hz",
        "100.00 ms",
        "25.00 ms",
        "ep_000001",
        "Missing action samples",
        "pick [cube]",
        "123.46 s",
    ):
        assert value in output
    assert "ep_000000" not in output


def test_verbose_shows_successful_episode_and_mode(recording_info):
    output = render(recording_info, verbose=True)
    assert "ep_000000 · PASS · strict" in output
    assert output.count("673 emitted / 673 expected") == 2


@pytest.mark.parametrize("empty", [False, True])
def test_unassessed_recordings_never_report_pass(recording_info, empty):
    recording_info.pop("quality")
    if empty:
        recording_info.update(episodes=0, saved_episodes=0, streams={}, status_stream=None)
    output = render(recording_info)
    assert "Not assessed" in output
    assert "PASS" not in output
    if empty:
        assert "No episodes" in output
        assert "No recorded streams" in output
        assert "Episode markers: unavailable" in output


@pytest.mark.parametrize("kind", ["hdf5", "lerobot"])
def test_dataset_summary(kind):
    info = {
        "format": kind,
        "path": "/datasets/pick",
        "version": "v3.0",
        "robot": "openyam",
        "episodes": 2,
        "frames": 1234,
        "fps": 30.0,
        "episode_lengths": {"min": 600, "max": 634, "mean": 617.0, "uniform": False},
        "shapes_uniform": True,
        "has_stats": False,
        "observation": {"wrist": {"shape": [480, 640, 3], "dtype": "uint8"}},
        "action": {"action": {"shape": [7], "dtype": "float32"}},
    }
    output = render(info)
    for value in (
        kind.upper(),
        "v3.0",
        "openyam",
        "1,234",
        "30.00 Hz",
        "600-634 frames",
        "617.00",
        "480 x 640 x 3",
        "float32",
        "Not available",
    ):
        assert value in output


def test_narrow_output_preserves_literal_paths_and_reasons(recording_info):
    recording_info["path"] = "/recordings/[bold]literal[/bold]/recording.mcap"
    recording_info["quality"][0].update(
        valid=False, rejection_reasons=["Missing [red]camera[/red] samples at recording end"]
    )
    output = render(recording_info, width=40)
    compact = "".join(output.split())
    assert "/recordings/[bold]literal[/bold]/recording.mcap" in compact
    assert "Missing[red]camera[/red]samplesatrecordingend" in compact
    assert "\x1b[" not in output
