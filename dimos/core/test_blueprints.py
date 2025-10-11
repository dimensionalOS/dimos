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

from dimos.core.blueprints import ModuleBlueprint, make_module_blueprint
from dimos.core.module import Module
from dimos.core.stream import In, Out


class Scratch:
    pass


class Petting:
    pass


class CatModule(Module):
    pet_cat: In[Petting]
    scratches: Out[Scratch]


def test_get_connection_set():
    assert make_module_blueprint(CatModule, args=(), kwargs={}) == ModuleBlueprint(
        module=CatModule,
        incoming={"pet_cat": Petting},
        outgoing={"scratches": Scratch},
        args=(),
        kwargs={},
    )
