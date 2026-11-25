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

"""unitree_webrtc_connect, importable on a machine with no audio server.

Any import of unitree_webrtc_connect imports sounddevice, which it does not
use. Import its names from here, so that sounddevice_loader runs first.
"""

import dimos.stream.audio.sounddevice_loader  # noqa: F401

# isort: split
