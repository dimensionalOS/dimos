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

from dimos_lcm.foxglove_msgs.ImageAnnotations import (  # type: ignore[import-untyped]
    ImageAnnotations as FoxgloveImageAnnotations,
)


class ImageAnnotations(FoxgloveImageAnnotations):  # type: ignore[misc]
    def __add__(self, other: "ImageAnnotations") -> "ImageAnnotations":
        points = self.points + other.points
        texts = self.texts + other.texts
        circles = self.circles + other.circles

        return ImageAnnotations(
            texts=texts,
            texts_length=len(texts),
            points=points,
            points_length=len(points),
            circles=circles,
            circles_length=len(circles),
        )

    def agent_encode(self) -> str:
        if len(self.texts) == 0:
            return None  # type: ignore[return-value]
        return list(map(lambda t: t.text, self.texts))  # type: ignore[return-value]
