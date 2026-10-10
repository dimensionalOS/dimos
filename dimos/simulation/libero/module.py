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

"""A LIBERO(-PRO) task with LIBERO's own Panda, run by LIBERO itself in its own environment.

The native process (``server.py``) builds the BDDL task in LIBERO, robot included, steps
it in real time, and publishes the streams ``MujocoSimModule`` gives an arm sim, plus
LIBERO's own goal check on ``task_status``. The coordinator drives the arm over
``sim_state`` / ``sim_command`` through the ``sim_transport`` adapter.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from pydantic import Field

from dimos.core.native_module import LogFormat, NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.simulation.libero.benchmark import (
    LIBERO_DEPS,
    LIBERO_PYTHON,
    bddl_root,
    libero_config_dir,
    libero_root,
)


def _default_bddl() -> str:
    return str(bddl_root() / "libero_goal" / "open_the_middle_drawer_of_the_cabinet.bddl")


class LiberoSimConfig(NativeModuleConfig):
    """The task and the hand camera; everything else is LIBERO's."""

    executable: str = str(Path(__file__).parent / "libero-native")
    stdin_config: bool = True
    log_format: LogFormat = LogFormat.TEXT

    # A LIBERO / LIBERO-PRO task file; defaults to a LIBERO-Goal task.
    bddl: str = Field(default_factory=_default_bddl)
    # Picks one of LIBERO's recorded initial states for the task, or, for a task without
    # them (generated perturbations), seeds LIBERO's own placement sampling.
    seed: int = 0

    width: int = 640
    height: int = 480
    fps: float = Field(default=15.0, gt=0.0)
    enable_depth: bool = True
    camera_info_fps: float = Field(default=1.0, gt=0.0)
    joint_state_hz: float = Field(default=100.0, gt=0.0)
    status_hz: float = Field(default=5.0, gt=0.0)


class LiberoSim(NativeModule):
    """Run a LIBERO task live; takes ``MujocoSimModule``'s place in an arm sim stack."""

    config: LiberoSimConfig

    sim_command: In[JointState]

    sim_state: Out[JointState]
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    tf: Out[TFMessage]
    task_status: Out[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # The checkout must exist before the native imports LIBERO from it.
        self.config.extra_env = {
            "PYTHONPATH": str(libero_root()),
            "LIBERO_CONFIG_PATH": str(libero_config_dir()),
            "LIBERO_PYTHON": LIBERO_PYTHON,
            "LIBERO_DEPS": " ".join(LIBERO_DEPS),
            "MUJOCO_GL": "egl",
            **self.config.extra_env,
        }
