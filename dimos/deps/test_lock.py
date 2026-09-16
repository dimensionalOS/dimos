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

import sys

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.deps.lock import LockIndex

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

LOCK = """
[[package]]
name = "dimos"
version = "0.0.14"
source = { editable = "." }
dependencies = [
    { name = "numpy", version = "2.2.6", marker = "python_full_version < '3.11'" },
    { name = "numpy", version = "2.3.5", marker = "python_full_version >= '3.11'" },
    { name = "open3d", marker = "platform_machine != 'aarch64'" },
]

[package.optional-dependencies]
perception = [{ name = "transformers", extra = ["torch"] }]
web = [{ name = "fastapi" }]
base = [{ name = "fastapi" }, { name = "transformers", extra = ["torch"] }]
unitree = [{ name = "fastapi" }, { name = "transformers", extra = ["torch"] }, { name = "webrtc" }]

[[package]]
name = "numpy"
version = "2.2.6"

[[package]]
name = "numpy"
version = "2.3.5"

[[package]]
name = "open3d"
version = "0.19.0"
dependencies = [{ name = "pillow" }]

[[package]]
name = "pillow"
version = "12.2.0"

[[package]]
name = "transformers"
version = "4.53.3"
dependencies = [{ name = "numpy", version = "2.3.5" }, { name = "requests" }]

[package.optional-dependencies]
torch = [{ name = "torch" }]

[[package]]
name = "torch"
version = "2.7.1"

[[package]]
name = "requests"
version = "2.32.0"

[[package]]
name = "fastapi"
version = "0.115.0"
dependencies = [{ name = "starlette" }]

[[package]]
name = "starlette"
version = "0.46.0"

[[package]]
name = "webrtc"
version = "1.0.0"
"""

PYPROJECT = """
[project]
name = "dimos"
dependencies = ["numpy>=1.26", "open3d>=0.18; platform_machine != 'aarch64'"]

[project.optional-dependencies]
perception = ["transformers[torch]>=4.53"]
web = ["fastapi>=0.115"]
base = ["dimos[web,perception]"]
unitree = ["dimos[base]", "webrtc>=1"]
"""


@pytest.fixture
def index() -> LockIndex:
    return LockIndex.from_documents(tomllib.loads(PYPROJECT), tomllib.loads(LOCK))


def test_direct_declarations_decide_ownership(index: LockIndex) -> None:
    assert index.core_direct == {"numpy", "open3d"}
    assert index.owner_of("numpy") == "core" and index.owner_of("Open3D") == "core"
    assert index.owner_of("pillow") is None and index.owner_of("torch") is None
    perception = index.extras["perception"]
    assert perception.own_direct == {"transformers"}
    assert perception.own_edges == {"transformers[torch]"}
    assert not perception.is_pure_aggregate
    assert index.owner_of("transformers") == "perception"


def test_self_references_become_includes(index: LockIndex) -> None:
    base = index.extras["base"]
    assert base.own_direct == frozenset()
    assert base.includes == {"web", "perception"}
    assert base.is_pure_aggregate
    unitree = index.extras["unitree"]
    assert unitree.own_direct == {"webrtc"}
    assert unitree.includes == {"base"}
    assert not unitree.is_pure_aggregate


def test_providers_are_direct_declarers_only(index: LockIndex) -> None:
    assert index.providers("torch") == ()
    assert index.providers("starlette") == ()
    assert index.providers("webrtc") == ("unitree",)
    assert index.providers("fastapi") == ("web",)
    assert index.providers("numpy") == ()


def test_chain_explains_transitive_availability(index: LockIndex) -> None:
    assert index.chain("pillow") == ("core", ("open3d", "pillow"))
    assert index.chain("numpy") == ("core", ("numpy",))
    assert index.chain("torch") == ("perception", ("transformers[torch]", "torch"))
    assert index.chain("starlette") == ("web", ("fastapi", "starlette"))
    assert index.chain("nothing") is None


def test_pyproject_alone_is_enough_for_ownership() -> None:
    index = LockIndex.from_documents(tomllib.loads(PYPROJECT))
    assert index.core_direct == {"numpy", "open3d"} and index.packages == frozenset()
    assert index.providers("webrtc") == ("unitree",)
    assert index.chain("pillow") is None


def test_real_lock_smoke() -> None:
    index = LockIndex.load(DIMOS_PROJECT_ROOT)
    assert "torch" in index.extras["perception"].own_direct
    assert "torch" not in index.core_direct
    assert "pygame" in index.extras["control"].own_direct
    assert "pygame" in index.extras["sim"].own_direct
    assert index.providers("unitree-webrtc-connect") == ("unitree",)
    assert index.extras["base"].is_pure_aggregate
    assert "opencv-contrib-python" in index.core_direct
    assert "pydantic-core" in index.core_direct and "lcm-dimos-fork" in index.core_direct
    assert index.chain("lcm-dimos-fork") == ("core", ("lcm-dimos-fork",))
