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

from dimos import agents2
from dimos.agents2.skills.navigation import NavigationSkillContainer
from dimos.core import DimosCluster, start, wait_exit
from dimos.perception import spatial_perception
from dimos.robot.unitree_webrtc.modular import g1detector


def deploy(dimos: DimosCluster, ip: str) -> None:
    g1 = g1detector.deploy(dimos, ip)

    nav = g1.get("nav")
    camera = g1.get("camera")
    detector3d = g1.get("detector3d")
    connection = g1.get("connection")

    spatialmem = spatial_perception.deploy(dimos, camera)

    navskills = dimos.deploy(
        NavigationSkillContainer,
        spatialmem,
        nav,
        detector3d,
    )
    navskills.start()

    agent = agents2.deploy(
        dimos,
        "You are controling a humanoid robot",
        skill_containers=[connection, nav, camera, spatialmem, navskills],
    )
    agent.run_implicit_skill("current_position")
    agent.run_implicit_skill("video_stream")

    return {"agent": agent, "spatialmem": spatialmem} + g1


if __name__ == "__main__":
    import argparse
    import os

    from dotenv import load_dotenv

    load_dotenv()

    parser = argparse.ArgumentParser(description="Unitree G1 Humanoid Robot Control")
    parser.add_argument("--ip", default=os.getenv("ROBOT_IP"), help="Robot IP address")

    args = parser.parse_args()

    dimos = start(8)
    deploy(dimos, args.ip)
    wait_exit()
    dimos.close_all()
