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

"""Loop-1 identifiability on the G1; the method is :mod:`dimos.simulation.sysid.identify`.

python -m dimos.robot.unitree.g1.sim.sysid.identify REC.db --preset stock --channel joint
"""

from __future__ import annotations

from collections.abc import Sequence

from dimos.simulation.sysid.identify import main as _main


def main(argv: Sequence[str] | None = None) -> None:
    from dimos.robot.unitree.g1.sim.engines.mujoco import MujocoBackend
    from dimos.robot.unitree.g1.sim.ranges import (
        DEFAULT_PRESET,
        ENGINE_DEFAULTS,
        SOLVER_KEYS,
        load_preset,
    )
    from dimos.robot.unitree.g1.sim.sysid.ingest import G1_READER

    _main(
        argv,
        reader=G1_READER,
        backend_cls=MujocoBackend,
        load_preset=load_preset,
        default_preset=DEFAULT_PRESET,
        defaults=ENGINE_DEFAULTS,
        exclude=SOLVER_KEYS,
    )


if __name__ == "__main__":
    main()
