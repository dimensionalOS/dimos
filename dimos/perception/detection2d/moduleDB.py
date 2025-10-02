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
from typing import List


from dimos.core import rpc
from dimos.msgs.vision_msgs import Detection3DArray
from dimos.perception.detection2d.module3D import Detection3DModule
from dimos.perception.detection2d.type import Detection3D


class DetectionDBModule(Detection3DModule):
    @rpc
    def start(self):
        super().start()
        unsub = self.pointcloud_stream().subscribe(self.add_detections)
        self._disposables.add(unsub)

    @rpc
    def stop(self):
        super().stop()

    def add_detections(self, detections: List[Detection3DArray]):
        for det in detections:
            self.add_detection(det)

    def add_detection(self, detection: Detection3D):
        print("DETECTION", detection)

    def lookup(self, label: str) -> List[Detection3D]:
        """Look up a detection by label."""
        return []
