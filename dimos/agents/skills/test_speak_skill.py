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

from collections.abc import Iterator

import pytest
from pytest_mock import MockerFixture

from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.stream.audio.tts.node_openai import Voice


@pytest.fixture
def speak_skill() -> Iterator[SpeakSkill]:
    module = SpeakSkill()
    yield module
    module.stop()


@pytest.mark.parametrize("api_key", [None, ""], ids=["unset", "empty"])
@pytest.mark.parametrize("blocking", [True, False])
def test_start_without_api_key_disables_speech(
    speak_skill: SpeakSkill,
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
    api_key: str | None,
    blocking: bool,
) -> None:
    if api_key is None:
        monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    else:
        monkeypatch.setenv("OPENAI_API_KEY", api_key)
    audio_output = mocker.patch("dimos.agents.skills.speak_skill.SounddeviceAudioOutput")
    warning = mocker.patch("dimos.agents.skills.speak_skill.logger.warning")

    speak_skill.start()

    assert speak_skill.speak("Hello", blocking=blocking) == "Error: TTS not initialized"
    audio_output.assert_not_called()
    warning.assert_called_once_with("OPENAI_API_KEY is not set. Speech is disabled.")


def test_start_with_api_key_connects_tts_to_audio_output(
    speak_skill: SpeakSkill, monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture
) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "test-api-key")
    tts_node = mocker.patch("dimos.agents.skills.speak_skill.OpenAITTSNode", autospec=True)
    audio_output = mocker.patch(
        "dimos.agents.skills.speak_skill.SounddeviceAudioOutput", autospec=True
    )

    speak_skill.start()

    tts_node.assert_called_once_with(speed=1.2, voice=Voice.ONYX)
    audio_output.return_value.consume_audio.assert_called_once_with(
        tts_node.return_value.emit_audio.return_value
    )
