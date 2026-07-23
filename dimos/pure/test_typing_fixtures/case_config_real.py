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

"""T2 statics against the REAL PureModule.

Unlike case_config_kwargs.py (stand-in PureModule), this fixture imports the
shipped dimos.pure.module/config so the synthesized-__init__, read-only config
property, ClassVar exclusion, required fields, and frozen_default read-only
errors are pinned against production.

Static-typing fixture — never imported at runtime.
"""

from __future__ import annotations

from typing import ClassVar

from dimos.pure.config import PureModuleConfig
from dimos.pure.module import PureModule


class Go2Connection(PureModule):
    LIMIT: ClassVar[int] = 3
    prefix: str = ""
    robot_ip: str | None = None
    odom_timeout: float = 0.5


class NeedsIp(PureModule):
    robot_ip: str  # no default -> required kwarg


class Sub(Go2Connection):
    extra_field: int = 7


c = Go2Connection(prefix="go2a", robot_ip="192.168.12.1")
reveal_type(c.prefix)  # R: builtins.str
reveal_type(c.odom_timeout)  # R: builtins.float
reveal_type(c.config)  # R: dimos.pure.config.PureModuleConfig
reveal_type(c.config.model_dump())  # R: builtins.dict[builtins.str, Any]

Go2Connection(prefx="go2a")  # E[call-arg]: did you mean "prefix"?
Go2Connection(robot_ip=0.5)  # E[arg-type]: incompatible type
Go2Connection(LIMIT=5)  # E[call-arg]: Unexpected keyword argument "LIMIT"
Go2Connection("go2a")  # E[misc]: Too many positional arguments
NeedsIp()  # E[call-arg]: Missing named argument "robot_ip"
Sub(prefix="a", extra_field=1)  # inherited + own fields: ok

# frozen at the static surface (frozen_default + read-only config property):
c.odom_timeout = 1.0  # E[misc]: is read-only
c.config = c.config  # E[misc]: is read-only

_cfg: PureModuleConfig = c.config  # BaseConfig-compatible currency: ok
