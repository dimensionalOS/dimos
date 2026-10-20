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


import pytest
from pytest_mock import MockerFixture

from dimos.imitation.policy.dreamlabs.probe import main


def test_probe_requires_explicit_live_flag(
    mocker: MockerFixture, monkeypatch: pytest.MonkeyPatch
) -> None:
    constructor = mocker.patch("dimos.imitation.policy.dreamlabs.probe.create_client")
    monkeypatch.setattr("sys.argv", ["probe"])
    main()
    constructor.assert_not_called()


def test_probe_sends_only_one_synthetic_inference(
    mocker: MockerFixture, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    constructor = mocker.patch("dimos.imitation.policy.dreamlabs.probe.create_client")
    client = constructor.return_value.__enter__.return_value
    client.get_action.return_value = {"q_l_chunk": [[0.0] * 7] * 48, "q_r_chunk": [[0.0] * 7] * 48}
    monkeypatch.setattr("sys.argv", ["probe", "--live"])
    main()
    client.get_action.assert_called_once_with()
    assert client.policy.call_args.args[0]["_record"] is False
    frames, state = client.set_observation.call_args.args
    assert set(frames) == {"video.overhead", "video.wrist_l", "video.wrist_r"}
    assert state["state.left_arm"].tolist() == [0.0, 0.5, 0.5, pytest.approx(-0.35), 0.0, 0.0]
    assert state["prompt"] == "hold position"
    assert (
        capsys.readouterr().out
        == "One synthetic inference returned two finite 48 by 7 chunks. No hardware connected.\n"
    )
    constructor.return_value.__exit__.assert_called_once()
