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

"""B601-DM coordinator: the shared Damiao connection coordinator."""

from __future__ import annotations

from dimos.robot.manipulators.damiao.damiao_blueprint import DamiaoCoordinator


class SeeedB601DMCoordinator(DamiaoCoordinator):
    """Drive one B601-DM connection with trajectories and its gripper task.

    The CAN selector lives on the connection module, so ``--can-port`` reaches
    the bus directly; ``--simulation`` composes ``MockJointConnection`` instead.
    """
