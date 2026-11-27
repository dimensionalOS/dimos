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

from collections.abc import Callable
import sys
from typing import Any

import pytest

from dimos.models.segmentation.yoloe import YoloeBoxSegmenter
from dimos.perception.detection.detectors.person.yolo import YoloPersonDetector
from dimos.perception.detection.detectors.yolo import Yolo2DDetector
from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector


@pytest.mark.parametrize(
    "factory", [Yolo2DDetector, YoloPersonDetector, Yoloe2DDetector, YoloeBoxSegmenter]
)
def test_without_ultralytics_the_error_names_the_extra(
    factory: Callable[[], Any], monkeypatch: pytest.MonkeyPatch
) -> None:
    # None makes `import ultralytics` fail, as if the package were not installed.
    monkeypatch.setitem(sys.modules, "ultralytics", None)

    with pytest.raises(ImportError, match="Install the yolo extra"):
        factory()
