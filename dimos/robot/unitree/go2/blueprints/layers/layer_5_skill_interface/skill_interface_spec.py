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

from typing import Any, Protocol

from dimos.spec.utils import Spec


class SkillInterfaceSpec(Spec, Protocol):
    def get_skill_interface_snapshot(self, domain: str = "") -> dict[str, Any]: ...
    def get_skill_contract(self, skill_name: str) -> dict[str, Any] | None: ...
    def validate_skill_request(self, skill_name: str, args_json: str = "{}") -> dict[str, Any]: ...


__all__ = ["SkillInterfaceSpec"]
