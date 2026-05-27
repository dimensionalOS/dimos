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

import json

from dimos.agents.skills.speak_skill import SpeakSkill


def test_speak_skill_start_is_noop_without_openai_api_key(monkeypatch) -> None:
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    skill = SpeakSkill()

    try:
        skill.start()
        assert (
            skill.speak("hello", blocking=False) == "Speech unavailable: OPENAI_API_KEY is not set"
        )
        assert skill.speech_status() == (
            "SpeakSkill status: tts=unavailable; reason=OPENAI_API_KEY is not set; "
            "audio_output=missing; background_speech_threads=0."
        )
    finally:
        skill.stop()


def test_speak_skill_status_reports_ready_when_tts_is_initialized() -> None:
    skill = SpeakSkill()
    try:
        skill._tts_node = object()
        skill._audio_output = object()

        assert skill.speech_status() == (
            "SpeakSkill status: tts=ready; audio_output=connected; "
            "background_speech_threads=0."
        )
    finally:
        skill._close_module()


def test_speak_skill_exposes_status_schema() -> None:
    skill = SpeakSkill()
    try:
        skill_infos = {info.func_name: json.loads(info.args_schema) for info in skill.get_skills()}
    finally:
        skill._close_module()

    assert "speech_status" in skill_infos
    status_schema = skill_infos["speech_status"]
    assert "text-to-speech readiness" in status_schema["description"]
    assert status_schema.get("properties", {}) == {}
