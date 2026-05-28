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
from types import SimpleNamespace

from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.agents.skills.unitree_speak_skill import UnitreeSpeakSkill, _generate_tone_wav
from dimos.robot.unitree.go2.blueprints.agentic._seat_guide_agentic import _seat_guide_agentic


class FakeGo2Connection:
    def __init__(self) -> None:
        self.requests = []

    def publish_request(self, topic, data):
        self.requests.append((topic, data))
        if data["api_id"] == 1001:
            filename = None
            for _, request_data in self.requests:
                if request_data["api_id"] == 2001:
                    filename = json.loads(request_data["parameter"])["file_name"]
                    break
            return {
                "data": {
                    "data": json.dumps(
                        {"audio_list": [{"CUSTOM_NAME": filename, "UNIQUE_ID": "audio-1"}]}
                    )
                }
            }
        return {"data": {"data": "{}"}}


class FakeOpenAIClient:
    audio = SimpleNamespace(
        speech=SimpleNamespace(create=lambda **kwargs: SimpleNamespace(content=b"RIFF-wav-data"))
    )


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


def test_unitree_speak_skill_start_is_noop_without_openai_api_key(monkeypatch) -> None:
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    skill = UnitreeSpeakSkill()

    try:
        skill.start()
        assert (
            skill.speak("hello", blocking=False)
            == "Robot speech unavailable: OPENAI_API_KEY is not set"
        )
        assert skill.speech_status() == (
            "UnitreeSpeakSkill status: tts=unavailable; "
            "reason=OPENAI_API_KEY is not set; robot_audio=missing; "
            "background_speech_threads=0."
        )
    finally:
        skill.stop()


def test_unitree_speak_skill_uploads_and_plays_audio_on_robot() -> None:
    skill = UnitreeSpeakSkill.__new__(UnitreeSpeakSkill)
    skill._connection = FakeGo2Connection()
    skill._openai_client = FakeOpenAIClient()
    skill._speech_unavailable_reason = None
    skill._bg_threads = []
    skill._bg_threads_lock = __import__("threading").Lock()

    result = skill.speak("我已经到了, 请坐。", blocking=True)

    api_ids = [data["api_id"] for _, data in skill._connection.requests]
    assert result == "Spoke on robot: 我已经到了, 请坐。"
    assert api_ids == [2001, 1001, 1007, 1004, 1002]
    assert json.loads(skill._connection.requests[0][1]["parameter"])["file_type"] == "wav"
    assert json.loads(skill._connection.requests[-1][1]["parameter"]) == {
        "unique_id": "audio-1"
    }


def test_unitree_speak_skill_audio_test_does_not_call_openai() -> None:
    skill = UnitreeSpeakSkill.__new__(UnitreeSpeakSkill)
    skill._connection = FakeGo2Connection()
    skill._openai_client = None
    skill._speech_unavailable_reason = "OPENAI_API_KEY is not set"
    skill._bg_threads = []
    skill._bg_threads_lock = __import__("threading").Lock()

    result = skill.play_robot_audio_test()

    api_ids = [data["api_id"] for _, data in skill._connection.requests]
    assert result.startswith("Robot audio test sent: filename=audio_test_")
    assert "bytes=" in result
    assert "unique_id=audio-1" in result
    assert api_ids[-4:] == [1001, 1007, 1004, 1002]
    assert api_ids[:-4] == [2001] * len(api_ids[:-4])
    upload_parameter = json.loads(skill._connection.requests[0][1]["parameter"])
    assert upload_parameter["file_type"] == "wav"
    assert upload_parameter["file_size"] > 200000


def test_unitree_speak_skill_megaphone_test_does_not_call_openai() -> None:
    skill = UnitreeSpeakSkill.__new__(UnitreeSpeakSkill)
    skill._connection = FakeGo2Connection()
    skill._openai_client = None
    skill._speech_unavailable_reason = "OPENAI_API_KEY is not set"
    skill._bg_threads = []
    skill._bg_threads_lock = __import__("threading").Lock()

    result = skill.play_robot_megaphone_test()

    api_ids = [data["api_id"] for _, data in skill._connection.requests]
    assert result == "Robot megaphone audio test sent: bytes=220544."
    assert api_ids[0] == 4001
    assert api_ids[-1] == 4002
    assert all(api_id == 4003 for api_id in api_ids[1:-1])


def test_generate_tone_wav_returns_wav_bytes() -> None:
    wav_data = _generate_tone_wav()

    assert wav_data.startswith(b"RIFF")
    assert b"WAVE" in wav_data[:16]


def test_seat_guide_blueprint_uses_unitree_speaker() -> None:
    module_names = {atom.module.__name__ for atom in _seat_guide_agentic.blueprints}

    assert "UnitreeSpeakSkill" in module_names
    assert "SpeakSkill" not in module_names
