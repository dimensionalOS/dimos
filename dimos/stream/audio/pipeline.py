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

"""The one owned speech-to-text chain: normalize -> local Whisper.

Every audio-to-text transport adapter (the legacy WebInput HTTP upload, the
cockpit VoiceInput chunks) feeds this same pipeline rather than growing its
own transcription implementation.
"""

from typing import TYPE_CHECKING

import reactivex as rx
import reactivex.operators as ops

from dimos.stream.audio.node_normalizer import AudioNormalizer

if TYPE_CHECKING:
    from reactivex import Observable

    from dimos.stream.audio.base import AudioEvent


def whisper_pipeline() -> "tuple[rx.subject.Subject[AudioEvent], Observable[str]]":
    """AudioEvents in, transcribed text out.

    Loads the Whisper model (blocking); call from a module's start(), not
    its constructor.
    """
    # Here to prevent unwanted imports in the file: whisper backends belong
    # to the optional [agents] extra and load a model at construction.
    from dimos.stream.audio.stt.node_whisper import WhisperNode

    audio_subject: rx.subject.Subject[AudioEvent] = rx.subject.Subject()
    normalizer = AudioNormalizer()
    stt_node = WhisperNode()
    normalizer.consume_audio(audio_subject.pipe(ops.share()))
    stt_node.consume_audio(normalizer.emit_audio())
    return audio_subject, stt_node.emit_text()
