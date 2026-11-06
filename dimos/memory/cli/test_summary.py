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

from typer.testing import CliRunner

from dimos.core.stream import Transport
from dimos.memory.cli.app import mem_app
from dimos.memory.codecs.lcm import LcmCodec
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tap import TransportRecorder
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


def test_summary_reports_missing_poses_on_fresh_recording(tmp_path, mocker):
    path = tmp_path / "memory.db"
    transport = mocker.Mock(spec=Transport)
    with SqliteStore(path=str(path)) as store:
        recorder = TransportRecorder(store)
        try:
            recorder.tap("odom", PoseStamped, transport)
            on_message = transport.subscribe.call_args.args[0]
            on_message(PoseStamped(ts=42.25, position=(1, 2, 3)))
        finally:
            recorder.close()

    # Summary must inspect metadata without decoding recorded sensor payloads.
    mocker.patch.object(LcmCodec, "decode", side_effect=AssertionError("unexpected payload read"))
    result = CliRunner().invoke(mem_app, ["summary", str(path)])

    assert result.exit_code == 0, result.output
    assert "odom" in result.output
    assert "no attached poses (run pose-fill)" in result.output


def test_summary_omits_missing_poses_hint_when_any_stream_has_poses(tmp_path):
    path = tmp_path / "memory.db"
    with SqliteStore(path=str(path)) as store:
        store.stream("unposed", float).append(1.0, ts=1.0)
        store.stream("posed", float).append(1.0, ts=1.0, pose=(1, 2, 3))

    result = CliRunner().invoke(mem_app, ["summary", str(path)])

    assert result.exit_code == 0, result.output
    assert "no attached poses" not in result.output
