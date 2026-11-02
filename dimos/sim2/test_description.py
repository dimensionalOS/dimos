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

from pydantic import TypeAdapter

from dimos.sim2.types import World


def test_cli_configuration_roundtrip_keeps_sensor_profiles_and_model_variants(world):
    adapter = TypeAdapter(World)
    restored = adapter.validate_python(adapter.dump_python(world))
    assert [type(s) for s in restored.sensors] == [type(s) for s in world.sensors]
    assert restored.revision == world.revision
    assert restored.instances[0].model.load().root_link == "left/base_link"
