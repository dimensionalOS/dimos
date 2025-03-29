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

from dimos.hardware.end_effector import EndEffector
from dimos.hardware.camera import Camera
from dimos.hardware.stereo_camera import StereoCamera
from dimos.hardware.ufactory import UFactoryEndEffector, UFactory7DOFArm

class HardwareInterface:
    def __init__(self, end_effector: EndEffector = None, sensors: list = None, arm_architecture: UFactory7DOFArm = None):
        self.end_effector = end_effector
        self.sensors = sensors if sensors is not None else []
        self.arm_architecture = arm_architecture

    def get_configuration(self):
        """Return the current hardware configuration."""
        return {
            'end_effector': self.end_effector,
            'sensors': [sensor.get_sensor_type() for sensor in self.sensors],
            'arm_architecture': self.arm_architecture
        }

    def set_configuration(self, configuration):
        """Set the hardware configuration."""
        self.end_effector = configuration.get('end_effector', self.end_effector)
        self.sensors = configuration.get('sensors', self.sensors)
        self.arm_architecture = configuration.get('arm_architecture', self.arm_architecture)

    def add_sensor(self, sensor):
        """Add a sensor to the hardware interface."""
        if isinstance(sensor, (Camera, StereoCamera)):
            self.sensors.append(sensor)
        else:
            raise ValueError("Sensor must be a Camera or StereoCamera instance.")
