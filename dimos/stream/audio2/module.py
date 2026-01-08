# Copyright 2025-2026 Dimensional Inc.
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

from dimos.agents2 import skill
from dimos.core import Module
from dimos.stream.audio2.input.tts_oai import openai_tts
from dimos.stream.audio2.operators import normalizer, robotize
from dimos.stream.audio2.output.network import network_output
from dimos.stream.audio2.output.soundcard import speaker


class SpeechModule(Module):
    syntesizer = staticmethod(openai_tts)
    output = staticmethod(speaker)

    @skill()
    def say(self, text: str) -> str:
        """Speak the given text out loud. using TTS."""
        self.syntesizer(text).pipe(robotize(), normalizer(), self.output()).run()
        return f"said {text} out loud"


class RemoteSpeechModule(SpeechModule):
    def __init__(self, host: str):
        super().__init__()
        self.output = lambda: network_output(host=host, port=5002, codec="opus")
