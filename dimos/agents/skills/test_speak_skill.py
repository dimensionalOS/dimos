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

from __future__ import annotations

from dimos.agents.skills.speak_skill import SpeakSkill


def test_start_disables_tts_when_audio_initialization_fails(mocker):
    mocker.patch(
        "dimos.agents.skills.speak_skill.OpenAITTSNode",
        side_effect=RuntimeError("missing api key"),
    )
    audio_output = mocker.patch("dimos.agents.skills.speak_skill.SounddeviceAudioOutput")

    skill = SpeakSkill()
    try:
        skill.start()

        assert skill._tts_node is None
        audio_output.assert_not_called()
        assert skill.speak("hello") == "Error: TTS not initialized"
    finally:
        skill.stop()
