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

from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import torch

from dimos.models.segmentation import edge_tam
from dimos.msgs.image import image_from_array
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg


def test_generated_tracker_frame_and_initial_jpeg(monkeypatch):
    monkeypatch.setattr(
        edge_tam, "_build_model", lambda: SimpleNamespace(image_size=8, device="cpu")
    )
    processor = edge_tam.EdgeTAMProcessor()
    pixels = np.zeros((12, 16, 3), dtype=np.uint8)
    pixels[:, :, 0] = 255
    image = image_from_array(pixels, encoding="rgb8")
    frame = processor._prepare_frame(image)
    assert frame.shape == (3, 8, 8)
    torch.testing.assert_close(
        frame[:, 0, 0], torch.tensor([(1 - 0.485) / 0.229, -0.456 / 0.224, -0.406 / 0.225])
    )
    with edge_tam._temp_dir_context(image) as directory:
        path = Path(directory)
        decoded = cv2.imread(str(path / "00000.jpg"))
        assert decoded.shape == (12, 16, 3)
        assert decoded[0, 0, 2] > 240
    assert not path.exists()
    mask = torch.zeros((12, 16))
    mask[2:8, 3:10] = 1
    detection = Detection2DSeg.from_sam2_result(mask, 7, image)
    assert detection.bbox == (3, 2, 9, 7)
    assert detection.to_ros_detection2d().header == image.header
