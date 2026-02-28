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

"""Default Livox SDK2 network port constants.

These match the defaults in ``common/livox_sdk_config.hpp`` (``SdkPorts``).
Both the Mid-360 driver and FAST-LIO2 modules reference this single source
so port numbers are defined in one place on the Python side.
"""

SDK_CMD_DATA_PORT = 56100
SDK_PUSH_MSG_PORT = 56200
SDK_POINT_DATA_PORT = 56300
SDK_IMU_DATA_PORT = 56400
SDK_LOG_DATA_PORT = 56500
SDK_HOST_CMD_DATA_PORT = 56101
SDK_HOST_PUSH_MSG_PORT = 56201
SDK_HOST_POINT_DATA_PORT = 56301
SDK_HOST_IMU_DATA_PORT = 56401
SDK_HOST_LOG_DATA_PORT = 56501
