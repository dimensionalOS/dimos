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

"""Fixed topic profiles. Importing this table never loads a media dependency."""

from dataclasses import dataclass


@dataclass(frozen=True)
class MediaProfile:
    kind: str
    max_hz: float
    width: int = 0
    height: int = 0
    quality: int = 75
    keep_every: int = 1


MEDIA_PROFILES = {
    "image.jpeg.low.v1": MediaProfile("jpeg", 30, 640, 360, 60),
    "image.jpeg.balanced.v1": MediaProfile("jpeg", 30, 1280, 720, 75),
    "image.jpeg.high.v1": MediaProfile("jpeg", 30, 1920, 1080, 85),
    "depth.lossless.v1": MediaProfile("depth", 30),
    "cloud.q16.low.v1": MediaProfile("cloud", 5, keep_every=4),
    "cloud.q16.balanced.v1": MediaProfile("cloud", 20),
    "cloud.q16.full.v1": MediaProfile("cloud", 30),
}
