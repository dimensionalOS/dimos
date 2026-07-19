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

"""The ``pm`` surface: pure-module rows, config, classification, and typing.

Canonical use: ``from dimos import pure as pm`` — then ``pm.In`` / ``pm.Out``
bundles, ``pm.PureModule`` bases, ``pm.tick()`` / ``pm.latest()`` field
specifiers, ``pm.step_spec()`` introspection, and the T4 protocols
(``pm.Stateless`` et al.) for structural aliases.

House rules (t4-typing.md §6): re-export *names* only — never submodules, no
star imports. The ``dimos.pure.typing`` submodule stays package-qualified and
must never shadow stdlib ``typing``. Everything here is the zero-engine data
layer; drivers, alignment, and the rim import *us*, never the reverse.

Export lists per owning spec: T1 ``rows.__all__``; T2 t2-config.md §2.3;
T3 t3-validation.md §1 (recommended set); T4 t4-typing.md §6 ``__all__``.
"""

from dimos.pure.config import (
    ConfigFieldError,
    FrozenModuleError,
    PureModuleConfig,
)
from dimos.pure.module import PureModule
from dimos.pure.rows import (
    UNSTAMPED,
    BundleDefinitionError,
    ContractSpec,
    FieldSpec,
    In,
    InterpolateSpec,
    LatestSpec,
    Out,
    PlainSpec,
    TickSpec,
    contract,
    interpolate,
    latest,
    tick,
)
from dimos.pure.stepspec import (
    PureModuleDefinitionError,
    StepKind,
    StepSpec,
    step_spec,
)
from dimos.pure.typing import (
    AsyncStateless,
    EngineSurface,
    Fold,
    InPort,
    InPorts,
    Mealy,
    OutPort,
    OutPorts,
    Stateless,
)

__all__ = [
    "UNSTAMPED",
    "AsyncStateless",
    "BundleDefinitionError",
    "ConfigFieldError",
    "ContractSpec",
    "EngineSurface",
    "FieldSpec",
    "Fold",
    "FrozenModuleError",
    "In",
    "InPort",
    "InPorts",
    "InterpolateSpec",
    "LatestSpec",
    "Mealy",
    "Out",
    "OutPort",
    "OutPorts",
    "PlainSpec",
    "PureModule",
    "PureModuleConfig",
    "PureModuleDefinitionError",
    "Stateless",
    "StepKind",
    "StepSpec",
    "TickSpec",
    "contract",
    "interpolate",
    "latest",
    "step_spec",
    "tick",
]
