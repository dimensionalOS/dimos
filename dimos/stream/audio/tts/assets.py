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

"""Pinned Kokoro assets shared by setup and runtime configuration."""

from dimos.constants import CACHE_DIR

TTS_CACHE_DIR = CACHE_DIR / "tts"
MODEL_FILENAME = "kokoro-v1.0.int8.onnx"
VOICES_FILENAME = "voices-v1.0.bin"
RELEASE_URL = "https://github.com/thewh1teagle/kokoro-onnx/releases/download/model-files-v1.1"
ASSET_SHA256 = {
    MODEL_FILENAME: "ae315a79b623f244700e4afb9246c46a26066782e049ba174bf3ba433970ee9c",
    VOICES_FILENAME: "bca610b8308e8d99f32e6fe4197e7ec01679264efed0cac9140fe9c29f1fbf7d",
}
SETUP_COMMAND = "uv run --extra manipulation --extra tts python -m dimos.stream.audio.tts.setup"
