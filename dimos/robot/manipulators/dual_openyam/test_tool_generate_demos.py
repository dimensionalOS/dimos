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

from types import SimpleNamespace

import pytest

from dimos.robot.manipulators.dual_openyam.tool_generate_demos import (
    planning_model_sha256,
    recording_episode,
)


def test_verified_episode_is_saved(mocker):
    monitor = mocker.Mock()
    monitor.get_status.return_value = SimpleNamespace(state="idle")
    monitor.command.side_effect = [
        SimpleNamespace(state="recording", ts=123.0),
        SimpleNamespace(state="idle"),
    ]

    with recording_episode(monitor) as start_ts:
        assert start_ts == 123.0

    assert [call.args[0] for call in monitor.command.call_args_list] == ["start", "save"]


@pytest.mark.parametrize("failure", [RuntimeError, KeyboardInterrupt])
def test_failed_or_interrupted_episode_is_discarded_and_propagates(mocker, failure):
    monitor = mocker.Mock()
    monitor.get_status.return_value = SimpleNamespace(state="idle")
    monitor.command.side_effect = [
        SimpleNamespace(state="recording", ts=123.0),
        SimpleNamespace(state="idle"),
    ]

    with pytest.raises(failure), recording_episode(monitor):
        raise failure("Interrupted take")

    assert [call.args[0] for call in monitor.command.call_args_list] == ["start", "discard"]


def test_collection_does_not_autosave_an_existing_take(mocker):
    monitor = mocker.Mock()
    monitor.get_status.return_value = SimpleNamespace(state="recording")

    with pytest.raises(RuntimeError, match="already recording"), recording_episode(monitor):
        pass

    monitor.command.assert_not_called()


@pytest.mark.parametrize("second_mesh,same_model", [(b"mesh", True), (b"changed", False)])
def test_recording_model_identity_uses_mesh_contents_instead_of_checkout_path(
    tmp_path, second_mesh, same_model
):
    first = tmp_path / "first.stl"
    second = tmp_path / "second.stl"
    first.write_bytes(b"mesh")
    second.write_bytes(second_mesh)
    first_xml = f'<robot><link name="arm"><mesh filename="{first}"/></link></robot>'
    second_xml = f'<robot><link name="arm"><mesh filename="{second}"/></link></robot>'

    assert (planning_model_sha256(first_xml) == planning_model_sha256(second_xml)) is same_model
