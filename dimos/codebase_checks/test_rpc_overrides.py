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

"""An override of an @rpc method must carry the decorator itself.

``Module.rpcs`` reads the ``__rpc__`` marker off each attribute, so a plain override
hides the base method from the module's proxy. The host then fetches it with a
GetAttrRequest, and the worker cannot send a bound method back: ``start()`` on such a
module fails at deploy time.
"""

import pytest

from dimos.core.global_config import global_config
from dimos.core.module import ModuleBase
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.get_all_blueprints import OptionalDependencyError, load_blueprint
from dimos.robot.test_all_blueprints import SELF_HOSTED_BLUEPRINTS


def _plain_overrides(module: type[ModuleBase]) -> list[str]:
    """Public methods without @rpc that some base class declares with it."""
    return sorted(
        name
        for name in dir(module)
        if not name.startswith("_")
        and callable(getattr(module, name, None))
        and not hasattr(getattr(module, name), "__rpc__")
        and any(hasattr(vars(base).get(name), "__rpc__") for base in module.__mro__)
    )


@pytest.mark.parametrize(
    "blueprint_name",
    [
        pytest.param(name, marks=pytest.mark.self_hosted)
        if name in SELF_HOSTED_BLUEPRINTS
        else name
        for name in sorted(all_blueprints)
    ],
)
def test_rpc_overrides_keep_the_decorator(
    blueprint_name: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    # The multi-robot blueprints read ROBOT_IPS at import time.
    monkeypatch.setattr(global_config, "robot_ips", "192.0.2.10,192.0.2.11")
    try:
        blueprint = load_blueprint(blueprint_name)
    except OptionalDependencyError as e:
        pytest.skip(f"Skipping due to missing optional dependency: {e}")

    violations = sorted(
        {
            f"{atom.module.__module__}.{atom.module.__name__}.{name}"
            for atom in blueprint.blueprints
            for name in _plain_overrides(atom.module)
        }
    )
    assert not violations, (
        f"Blueprint {blueprint_name!r} has @rpc methods overridden without the decorator; "
        f"add @rpc to: {violations}"
    )
