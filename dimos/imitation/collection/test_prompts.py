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

from dimos.imitation.collection.prompts import CollectionSpeech
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.stream.audio.tts.kokoro import KokoroTTSConfig


def test_confirmed_transitions_use_prepared_audio_and_ignore_duplicate_polls(speech):
    prompts, engine = speech
    engine.close.assert_called_once()
    engine.synthesize.reset_mock()
    events = [
        ("init", "idle", 0, 0, None),
        ("save", "idle", 0, 0, None),
        ("discard", "idle", 0, 0, None),
        ("start", "recording", 0, 0, b"Recording started"),
        ("save", "idle", 1, 0, b"Episode saved"),
        ("start", "recording", 1, 0, b"Recording started"),
        ("discard", "idle", 1, 1, b"Recording canceled"),
        ("start", "recording", 1, 1, b"Recording started"),
        ("start", "recording", 2, 1, b"Recording started"),
    ]
    for ts, (event, state, saved, discarded, expected) in enumerate(events):
        status = EpisodeStatus(
            ts=ts, last_event=event, state=state, episodes_saved=saved, episodes_discarded=discarded
        )
        assert prompts.update(status) == expected
        assert prompts.update(status) is None
        # RPC polling refreshes timestamps even when the episode has not changed.
        assert prompts.update(status.model_copy(update={"ts": status.ts + 0.5})) is None
    engine.synthesize.assert_not_called()


@pytest.fixture
def speech(mocker):
    engine = mocker.patch(
        "dimos.imitation.collection.prompts.KokoroTTS", autospec=True
    ).return_value
    engine.synthesize.side_effect = lambda phrase: phrase.encode()
    speech = CollectionSpeech(KokoroTTSConfig(enabled=True))
    speech.prepare()
    return speech, engine


def test_failed_preparation_releases_engine(mocker):
    engine = mocker.patch(
        "dimos.imitation.collection.prompts.KokoroTTS", autospec=True
    ).return_value
    engine.synthesize.side_effect = RuntimeError("synthesis failed")
    speech = CollectionSpeech(KokoroTTSConfig(enabled=True))
    with pytest.raises(RuntimeError, match="synthesis failed"):
        speech.prepare()
    engine.close.assert_called_once_with()
    assert (
        speech.update(
            EpisodeStatus(
                ts=1, state="recording", last_event="start", episodes_saved=0, episodes_discarded=0
            )
        )
        is None
    )
