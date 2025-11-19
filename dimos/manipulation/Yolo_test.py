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

# Quick YOLO test
import cv2
from ultralytics import YOLO

# Load model directly
model = YOLO("yolov8n.pt")  # Use a known good model

# Test on your camera image
img = cv2.imread("/tmp/test_rgb.jpg")  # Use your saved test image
results = model(img, conf=0.25)

print(f"Detected {len(results[0].boxes)} objects")
for box in results[0].boxes[:5]:
    print(f"  Class: {model.names[int(box.cls)]}, Conf: {box.conf:.2f}")
