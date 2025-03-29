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

from dimos.hardware.sensor import AbstractSensor

class Camera(AbstractSensor):
    def __init__(self, resolution=None, focal_length=None, sensor_size=None, sensor_type='Camera'):
        super().__init__(sensor_type)
        self.resolution = resolution  # (width, height) in pixels
        self.focal_length = focal_length  # in millimeters
        self.sensor_size = sensor_size  # (width, height) in millimeters

    def get_sensor_type(self):
        return self.sensor_type

    def calculate_intrinsics(self):
        if not self.resolution or not self.focal_length or not self.sensor_size:
            raise ValueError("Resolution, focal length, and sensor size must be provided")

        # Calculate pixel size
        pixel_size_x = self.sensor_size[0] / self.resolution[0]
        pixel_size_y = self.sensor_size[1] / self.resolution[1]

        # Calculate the principal point (assuming it's at the center of the image)
        principal_point_x = self.resolution[0] / 2
        principal_point_y = self.resolution[1] / 2

        # Calculate the focal length in pixels
        focal_length_x = self.focal_length / pixel_size_x
        focal_length_y = self.focal_length / pixel_size_y

        return {
            'focal_length_x': focal_length_x,
            'focal_length_y': focal_length_y,
            'principal_point_x': principal_point_x,
            'principal_point_y': principal_point_y
        }

    def get_intrinsics(self):
        return self.calculate_intrinsics()
