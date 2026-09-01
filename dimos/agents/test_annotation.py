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
