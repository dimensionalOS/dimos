# Copyright 2025 Dimensional Inc.
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

from dimos.stream.audio.node_key_recorder import KeyRecorder
from dimos.stream.audio.node_microphone import SounddeviceAudioSource
from dimos.stream.audio.node_normalizer import AudioNormalizer
from dimos.stream.audio.node_output import SounddeviceAudioOutput
from dimos.stream.audio.node_volume_monitor import monitor
from dimos.stream.audio.stt.node_whisper import WhisperNode
from dimos.stream.audio.text.node_stdout import TextPrinterNode
from dimos.stream.audio.tts.node_openai import OpenAITTSNode, Voice


def stt():  # type: ignore[no-untyped-def]
    # Create microphone source, recorder, and audio output
    mic = SounddeviceAudioSource()
    normalizer = AudioNormalizer()
    recorder = KeyRecorder(always_subscribe=True)
    whisper_node = WhisperNode()  # Assign to global variable

    # Connect audio processing pipeline
    normalizer.consume_audio(mic.emit_audio())
    recorder.consume_audio(normalizer.emit_audio())
    monitor(recorder.emit_audio())
    whisper_node.consume_audio(recorder.emit_recording())

    user_text_printer = TextPrinterNode(prefix="USER: ")
    user_text_printer.consume_text(whisper_node.emit_text())

    return whisper_node


def tts():  # type: ignore[no-untyped-def]
    tts_node = OpenAITTSNode(speed=1.2, voice=Voice.ONYX)
    agent_text_printer = TextPrinterNode(prefix="AGENT: ")
    agent_text_printer.consume_text(tts_node.emit_text())

    response_output = SounddeviceAudioOutput(sample_rate=24000)
    response_output.consume_audio(tts_node.emit_audio())

    return tts_node
