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

""" 
Given a policy server tuned for a particular robot, this script will get observations from camera and joint positions, 
query the policy server for actions, and send the actions to the robot.
"""

import os
import threading
import time

import numpy as np
from openpi_client import websocket_client_policy
from xarm.wrapper import XArmAPI

from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs import Image, JointCommand, JointState
from dimos.msgs.sensor_msgs.image_impls.AbstractImage import ImageFormat
from dimos.msgs.sensor_msgs import JointCommand

ACTION_HORIZON = 15
ACTION_CHUNK = None
GRIPPER_CHUNK = None

# xArm7 joint limits in degrees (lower, upper)
XARM7_JOINT_LIMITS_DEG = [
    (-360.0, 360.0),
    (-118.0, 118.0),
    (-360.0, 360.0),
    (-233.0, 11.0),
    (-360.0, 360.0),
    (-97.0, 180.0),
    (-360.0, 360.0),
]


def get_camera_image(timeout: float = 5.0, topic: str = "/camera/color") -> np.ndarray:
    event = threading.Event()
    image_data: dict[str, np.ndarray] = {}

    def on_img(msg: Image) -> None:
        if event.is_set():
            return
        image_data["image"] = msg.to_rgb().to_opencv()
        os.makedirs("captures", exist_ok=True)
        filename = f"camera_color_{time.time()}.png"
        Image.from_numpy(image_data["image"], format=ImageFormat.RGB).save(
            os.path.join("captures", filename)
        )
        event.set()

    transport = LCMTransport(topic, Image)
    transport.subscribe(on_img)

    if not event.wait(timeout=timeout):
        raise TimeoutError(f"No image received on {topic} within {timeout} seconds.")

    return image_data["image"]

def get_xarm_joint_positions(timeout: float = 5.0, topic: str = "/xarm/joint_states"):
    event = threading.Event()
    joint_positions: dict[str, np.ndarray] = {}

    def on_joint_state(msg: JointState) -> None:
        if event.is_set():
            return
        joint_positions["joint_positions"] = msg.position
        event.set()

    transport = LCMTransport(topic, JointState)
    transport.subscribe(on_joint_state)

    if not event.wait(timeout=timeout):
        raise TimeoutError(f"No joint states received on {topic} within {timeout} seconds.")

    return joint_positions["joint_positions"]


def get_observation():
    return {
        "observation/exterior_image_1_left": get_camera_image(),  # ADD SECOND CAMERA IN BLUEPRINT DEFINED WITH SERIAL NUMBER
        "observation/wrist_image_left": get_camera_image(),
        "observation/joint_position": get_xarm_joint_positions(),
        "observation/gripper_position": 0.0,
        "prompt": "move the arm slightly to the left",
    }


def run_inference():
    """
    Run inference loop until user interrupts
    """
    actions_from_chunk_completed = 0
    joint_cmd_pub = LCMTransport("/xarm/joint_position_command", JointCommand)

    while True:
        if actions_from_chunk_completed == 0 or actions_from_chunk_completed >= ACTION_HORIZON:
            actions_from_chunk_completed = 0
            observation = get_observation()

            result = policy.infer(observation)
            action_chunk = result["actions"]  # Shape: (15, 8) - these are VELOCITY COMMANDS

            dt = 1.0 / 15.0
            action_chunk = action_chunk.copy()
            action_chunk[:, :-1] *= dt
            action_chunk[:, :-1] = np.cumsum(
                action_chunk[:, :-1], axis=0
            )  # integrate to get delta position in radians, franka
            current_joint_positions = np.array(get_xarm_joint_positions())
            action_chunk[:, :-1] += current_joint_positions
            action_chunk[:, :-1] *= 360 / (2 * np.pi)  # convert to degrees
            ACTION_CHUNK = action_chunk[:, :-1]
            GRIPPER_CHUNK = action_chunk[:, 7]
            GRIPPER_CHUNK = np.where(GRIPPER_CHUNK > 0.5, 0.0, GRIPPER_CHUNK)

        action = ACTION_CHUNK[actions_from_chunk_completed]
        limits = np.array(XARM7_JOINT_LIMITS_DEG[: len(action)])
        lower = limits[:, 0]
        upper = limits[:, 1]
        action = np.clip(action, lower, upper)
        gripper_xarm = (1.0 - GRIPPER_CHUNK[actions_from_chunk_completed]) * 850
        actions_from_chunk_completed += 1

        print(f"Setting joint positions: {action} and gripper position: {gripper_xarm}")
        joint_positions_rad = np.deg2rad(action).tolist()
        joint_cmd_pub.broadcast(None, JointCommand(positions=joint_positions_rad))

        time.sleep(0.2)


if __name__ == "__main__":
    # connect to policy server
    policy = websocket_client_policy.WebsocketClientPolicy(
        host="localhost",
        port=8000,
    )

    # connect to xArm
    arm = XArmAPI("192.168.2.235")
    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(1)
    arm.set_state(state=0)
    arm.move_gohome(wait=True)

    print("Starting inference loop...")
    run_inference()

    arm.disconnect()
