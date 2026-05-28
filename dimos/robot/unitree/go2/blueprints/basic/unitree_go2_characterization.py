#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Unitree Go2 FOPDT characterization — one-terminal HW flow.

Bundles GO2Connection + ControlCoordinator + (publish-only-when-active)
pygame keyboard teleop with the :class:`Characterizer` module so the
operator runs a single command:

    dimos run unitree-go2-characterization

instead of the two-terminal flow (``dimos run
unitree-go2-webrtc-keyboard-teleop`` + ``python -m
dimos.utils.benchmarking.characterization``). All operator input goes
through the pygame window:

  * **WASD/QE** — reposition the robot between steps (existing teleop).
  * **ENTER** — advance to the next SI step.
  * **K** — skip the current amplitude.
  * **Backspace** — quit (no artifact written).

Why the gate stream: ``dimos run`` deploys modules into forkserver
worker subprocesses that don't share the parent CLI's TTY, so
``input()`` inside the Characterizer would EOF immediately. Routing the
operator's ENTER/K/Backspace through KeyboardTeleop's pygame event loop
(which already owns its own X11 window for movement keys) and an
``Out[str]`` -> ``In[str]`` stream avoids stdin entirely.

For ``--mode self-test`` (pure in-process math) keep using the CLI
entrypoint. For end-to-end sim characterization, bring up
``coordinator-sim-fopdt`` in one terminal and run the CLI with
``--mode hw`` in another.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Int8 import Int8
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_coordinator import (
    unitree_go2_coordinator,
)
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.utils.benchmarking.characterization import Characterizer
from dimos.utils.benchmarking.characterization_recorder import CharacterizationRecorder

unitree_go2_characterization = autoconnect(
    unitree_go2_coordinator,
    KeyboardTeleop.blueprint(publish_only_when_active=True),
    Characterizer.blueprint(robot="go2", mode="hw", gate_source="stream"),
    CharacterizationRecorder.blueprint(robot_id="go2"),
).transports(
    {
        # Operator gate events from the pygame window -> Characterizer.
        # autoconnect pairs KeyboardTeleop.gate (Out) with
        # Characterizer.gate (In) by name+type; the LCM transport gives
        # them a wire since the modules live in different worker
        # subprocesses. Int8 carries the gate code (0=advance, 1=skip,
        # 2=quit) — see GATE_* constants in keyboard_teleop.
        ("gate", Int8): LCMTransport("/characterizer/gate", Int8),
        # CharacterizationRecorder taps the LCM topics the rest of the
        # stack already publishes on. LCM is multicast so additional
        # subscribers are free. The recorder's `odom: In[PoseStamped]`
        # port is named plain `odom` — GO2Connection's outbound odom is
        # remapped to `go2_odom` on the bus, so we explicitly route the
        # recorder's port to /go2/odom here.
        ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("odom", PoseStamped): LCMTransport("/go2/odom", PoseStamped),
    }
)

__all__ = ["unitree_go2_characterization"]
