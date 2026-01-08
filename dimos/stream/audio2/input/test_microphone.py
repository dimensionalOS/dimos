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

import threading
import time

import pytest
from reactivex import operators as ops

from dimos.stream.audio2.input import microphone
from dimos.stream.audio2.operators import normalizer, vumeter
from dimos.stream.audio2.output import speaker
from dimos.stream.audio2.types import AudioFormat, AudioSpec


@pytest.mark.tool
def test_mic_to_speaker():
    microphone().pipe(normalizer(), vumeter(), speaker()).run()
