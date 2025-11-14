# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import yaml
import logging
from sensor_msgs.msg import CameraInfo
from ament_index_python.packages import get_package_share_directory

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


def load_camera_info():
    yaml_file = get_package_share_directory('go2_robot_sdk') + "/calibration/front_camera.yaml"

    logger.info("Loading camera info from file: {}".format(yaml_file))

    # Load the camera info from the YAML file
    with open(yaml_file, "r") as file_handle:
        camera_info_data = yaml.safe_load(file_handle)

    # Create a CameraInfo message
    camera_info_msg = CameraInfo()

    # Fill in the CameraInfo fields from the YAML data
    camera_info_msg.width = camera_info_data["image_width"]
    camera_info_msg.height = camera_info_data["image_height"]
    camera_info_msg.k = camera_info_data["camera_matrix"]["data"]
    camera_info_msg.d = camera_info_data["distortion_coefficients"]["data"]
    camera_info_msg.r = camera_info_data["rectification_matrix"]["data"]
    camera_info_msg.p = camera_info_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = camera_info_data["distortion_model"]

    return camera_info_msg
