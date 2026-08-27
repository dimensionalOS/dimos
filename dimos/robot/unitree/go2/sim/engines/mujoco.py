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

"""The menagerie Go2 behind the seam: the shared CPU engine on the Go2 plant."""

from __future__ import annotations

from typing import ClassVar, cast

from dimos.robot.unitree.go2.sim.engines import model as go2_model
from dimos.simulation.sysid.engines.model import Plant
from dimos.simulation.sysid.engines.mujoco import (
    MujocoBackend as _MujocoBackend,
    MujocoSession as MujocoSession,
)


class MujocoBackend(_MujocoBackend):
    """The Go2 plant module on the shared MuJoCo engine."""

    plant: ClassVar[Plant] = cast(
        "Plant", go2_model
    )  # a module satisfies the protocol; mypy will not say so
