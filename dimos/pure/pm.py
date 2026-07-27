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

Canonical use: ``from dimos.pure import pm`` — then ``pm.In`` / ``pm.Out``
bundles, ``pm.PureModule`` bases, ``pm.tick()`` / ``pm.latest()`` field
specifiers, ``pm.step_spec()`` introspection, and the T4 protocols
(``pm.Stateless`` et al.) for structural aliases.

A plain module, not a package ``__init__`` (house rule: no ``__init__.py``), so
importing a submodule never drags the surface in. Every name below uses the
``X as X`` re-export form, which is what lets other modules import it from here.

House rules (t4-typing.md §6): re-export *names* only — never submodules, no
star imports. The ``dimos.pure.typing`` submodule stays package-qualified and
must never shadow stdlib ``typing``. Everything here is the zero-engine data
layer; drivers, alignment, and the rim import *us*, never the reverse.

Export lists per owning spec: T1 rows; T2 t2-config.md §2.3;
T3 t3-validation.md §1 (recommended set); T4 t4-typing.md §6.
"""

from dimos.pure.align import (
    Aligner as Aligner,
    AlignmentError as AlignmentError,
    Interpolatable as Interpolatable,
    align as align,
    register_interpolator as register_interpolator,
)
from dimos.pure.config import (
    ConfigFieldError as ConfigFieldError,
    FrozenModuleError as FrozenModuleError,
    PureModuleConfig as PureModuleConfig,
)
from dimos.pure.debugrec import (
    Debug as Debug,
    latest as debug_latest,  # noqa: F401  — renamed re-export
    load as debug_load,  # noqa: F401  — renamed re-export
)
from dimos.pure.drivers import (
    PureModuleRunError as PureModuleRunError,
    RunHooks as RunHooks,
    StepError as StepError,
    run_over as run_over,
)
from dimos.pure.health import Health as Health
from dimos.pure.module import PureModule as PureModule
from dimos.pure.resources import resource as resource
from dimos.pure.rows import (
    UNSTAMPED as UNSTAMPED,
    BundleDefinitionError as BundleDefinitionError,
    ContractSpec as ContractSpec,
    FieldSpec as FieldSpec,
    In as In,
    InterpolateSpec as InterpolateSpec,
    LatestSpec as LatestSpec,
    Out as Out,
    PlainSpec as PlainSpec,
    TfOutSpec as TfOutSpec,
    TfSpec as TfSpec,
    TickSpec as TickSpec,
    contract as contract,
    interpolate as interpolate,
    latest as latest,
    tf as tf,
    tf_out as tf_out,
    tick as tick,
)
from dimos.pure.state import State as State
from dimos.pure.stepspec import (
    PureModuleDefinitionError as PureModuleDefinitionError,
    StepKind as StepKind,
    StepSpec as StepSpec,
    step_spec as step_spec,
)
from dimos.pure.typing import (
    AsyncStateless as AsyncStateless,
    EngineSurface as EngineSurface,
    Fold as Fold,
    InPort as InPort,
    InPorts as InPorts,
    Mealy as Mealy,
    OutPort as OutPort,
    OutPorts as OutPorts,
    Stamped as Stamped,
    Stateless as Stateless,
)
