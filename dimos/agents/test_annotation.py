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


from types import SimpleNamespace
from unittest.mock import Mock

from dimos.agents.annotation import skill


def test_skill_bare_form_has_empty_uses_and_instant_lifecycle():
    @skill
    def s() -> str:
        return "ok"

    assert s.__skill__ is True
    assert list(s.__skill_uses__) == []
    assert s.__skill_lifecycle__ == "instant"


def test_skill_parametrized_form_stores_uses_and_lifecycle():
    @skill(uses=["movement"], lifecycle="background")
    def s() -> str:
        return "ok"

    assert list(s.__skill_uses__) == ["movement"]
    assert s.__skill_lifecycle__ == "background"


def test_skill_parametrized_with_no_uses_defaults_to_empty():
    @skill(lifecycle="background")
    def s() -> str:
        return "ok"

    assert list(s.__skill_uses__) == []
    assert s.__skill_lifecycle__ == "background"


def test_skill_native_is_a_skill_that_forwards_to_the_native_server() -> None:
    @skill(native=True, uses=["movement"])
    def go(self, x: float) -> str:
        """Served by the native process."""

    assert go.__skill__ is True
    assert go.__rpc__ is True
    assert go.__rpc_native__ is True
    assert list(go.__skill_uses__) == ["movement"]

    module = SimpleNamespace(
        rpc_name="Nav", rpc=Mock(call_sync=Mock(return_value=("done", lambda: None)))
    )
    assert go(module, 2.0) == "done"
    module.rpc.call_sync.assert_called_once_with("Nav/go", ((2.0,), {}))
