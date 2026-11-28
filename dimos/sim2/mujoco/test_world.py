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


import pytest

# linux-aarch64 has no usd-core wheel, and every scene here is a USD layer.
pytest.importorskip("pxr")


def test_a_spawn_points_volume_neither_collides_nor_renders() -> None:
    from dimos.sim2.mujoco.world import compile_world
    from dimos.sim2.scene import LOGISTICS, layer_file, scene_usd

    model = compile_world(layer_file(scene_usd(LOGISTICS).usd, "logistics"), ids=()).compile()
    names = [model.geom(geom).name for geom in range(model.ngeom)]
    assert names and not [name for name in names if "spawns" in name]
