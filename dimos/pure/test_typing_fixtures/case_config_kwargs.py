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

"""T2 seed (shared fixture): flat config fields become typed constructor
kwargs; nested classes, resources, and step never become fields.

Stand-in PureModule until config.py/module.py land; T2 adopts this case.
Origin: design-session config_fields.py, verbatim modulo spelling.

Static-typing fixture — never imported at runtime.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any, Generic, TypeVar

from typing_extensions import dataclass_transform

from dimos.pure.typing import EngineSurface

T = TypeVar("T")


class Resource(Generic[T]):
    def __init__(self, fn: Callable[[Any], T]) -> None:
        raise NotImplementedError

    def __get__(self, obj: object, owner: type | None = None) -> T:
        raise NotImplementedError


def resource(fn: Callable[[Any], T]) -> Resource[T]:
    raise NotImplementedError


@dataclass_transform(kw_only_default=True)
class PureModule(EngineSurface):
    def __init_subclass__(cls, **kw: object) -> None:
        raise NotImplementedError


class Odometry:
    pass


class Pose:
    pass


class Grid:
    pass


class Go2Connection(PureModule):
    prefix: str = ""
    robot_ip: str | None = None
    odom_timeout: float = 0.5

    class In:
        ts: float
        odom: Odometry

    class Out:
        ts: float
        pose: Pose

    @resource
    def grid(self) -> Grid:
        raise NotImplementedError

    def step(self, i: In) -> Out:
        raise NotImplementedError


c = Go2Connection(prefix="go2a", robot_ip="192.168.12.1")
reveal_type(c.prefix)  # R: builtins.str
reveal_type(c.odom_timeout)  # R: builtins.float
reveal_type(c.grid)  # R: Grid

# config + step protocols compose — the engine surface still infers:
reveal_type(next(c.over()))  # R: Go2Connection.Out

Go2Connection(prefx="go2a")  # E[call-arg]: did you mean "prefix"?
Go2Connection(robot_ip=0.5)  # E[arg-type]: incompatible type
Go2Connection(In=3)  # E[call-arg]: Unexpected keyword argument
