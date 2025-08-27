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

from dimos.perception.detection2d.module import (
    better_detection_format,
    build_bbox,
    build_detection2d,
    build_detection2d_array,
    build_imageannotations,
)

array_sample = better_detection_format(
    [
        [[246.2418670654297, 315.33331298828125, 371.5143127441406, 387.5533752441406]],
        [10],
        [28],
        [0.6393297910690308],
        ["suitcase"],
    ]
)

import time


class FakeImage:
    ts: float

    def __init__(self):
        self.ts = time.time()


detections = (FakeImage(), array_sample)


def test_build_detectionarray():
    print(build_detection2d_array(detections).lcm_encode())


def test_build_imageannotations():
    annotations = build_imageannotations(detections)
    print(annotations, annotations.texts)
    print(annotations.lcm_encode())
