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

from importlib.metadata import requires

from packaging.requirements import Requirement


def test_linux_python310_mapping_extra_selects_one_gtsam_provider() -> None:
    target = {
        "extra": "mapping",
        "sys_platform": "linux",
        "platform_system": "Linux",
        "platform_machine": "x86_64",
        "python_version": "3.10",
        "python_full_version": "3.10.12",
    }
    providers = {
        req.name
        for req in map(Requirement, requires("dimos") or [])
        if req.name in {"gtsam", "gtsam-extended"}
        and (not req.marker or req.marker.evaluate(target))
    }
    assert providers == {"gtsam"}
