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

import os

from dimos.simulation.isaac import IsaacSimulator, IsaacStream


def main():
    # Initialize simulator
    sim = IsaacSimulator(headless=True)

    # Create stream with custom settings
    stream = IsaacStream(
        simulator=sim,
        width=1920,
        height=1080,
        fps=60,
        camera_path="/World/alfred_parent_prim/alfred_base_descr/chest_cam_rgb_camera_frame/chest_cam",
        annotator_type="rgb",
        transport="tcp",
        rtsp_url="rtsp://mediamtx:8554/stream",
        usd_path=f"{os.getcwd()}/assets/TestSim3.usda",
    )

    # Start streaming
    stream.stream()


if __name__ == "__main__":
    main()
