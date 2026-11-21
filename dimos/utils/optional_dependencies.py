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

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from importlib.metadata import PackageNotFoundError, distribution
from importlib.util import find_spec
import re
import sys

from packaging.requirements import Requirement
from packaging.utils import canonicalize_name

from dimos.constants import DIMOS_PROJECT_ROOT

# Imports whose distribution cannot be inferred from its normalized name.
_DISTRIBUTIONS = {
    "PIL": "pillow",
    "bosdyn": "bosdyn-client",
    "bosdyn.api": "bosdyn-api",
    "bosdyn.client": "bosdyn-client",
    "can": "python-can",
    "gtsam": "gtsam-extended",
    "hydra": "hydra-core",
    "mujoco_playground": "playground",
    "pydrake": "drake",
    "pxr": "usd-core",
    "rerun": "rerun-sdk",
    "sam2": "edgetam-dimos",
    "scservo_sdk": "ftservo-python-sdk",
    "socketio": "python-socketio",
    "unitree_sdk2py": "unitree-sdk2py-dimos",
    "warp": "warp-lang",
    "yaml": "pyyaml",
    "xarm": "xarm-python-sdk",
}


class OptionalDependencyError(ImportError):
    """A blueprint could not import because an optional dependency is missing."""


@contextmanager
def explain_missing_extra() -> Iterator[None]:
    """Identify a missing top-level package owned by an extra on this platform."""
    try:
        yield
    except ModuleNotFoundError as error:
        module = error.name
        if not module or ("." in module and module not in _DISTRIBUTIONS):
            raise
        try:
            package = distribution("dimos")
        except PackageNotFoundError:
            package = None
        if package is None:
            raise

        name = canonicalize_name(_DISTRIBUTIONS.get(module, module))
        extras: set[str] = set()
        declared: set[str] = set()
        environments: set[str] = set()
        for requirement in map(Requirement, package.requires or []):
            if canonicalize_name(requirement.name) != name:
                continue
            marker = requirement.marker
            if marker is None or marker.evaluate({"extra": ""}):
                raise
            # Setuptools appends an extra equality to each optional Requires-Dist marker.
            environments.add(re.sub(r' and extra == "[^"]+"$', "", str(marker)))
            declared.update(re.findall(r'\bextra == "([^"]+)"', str(marker)))
            extras.update(
                extra
                for extra in package.metadata.get_all("Provides-Extra") or []
                if marker.evaluate({"extra": extra})
            )
        if not (extras or declared):
            raise

        choices = " or ".join(f"'dimos[{extra}]'" for extra in sorted(extras or declared))
        if not extras:
            raise OptionalDependencyError(
                f"Missing {module!r}. Provided by {choices}, with no compatible build "
                f"for this Python or platform. Create a supported dimOS environment matching "
                f"{' or '.join(sorted(environments))} before installing the extra."
            ) from error
        hint = f"Install {choices}"
        if (DIMOS_PROJECT_ROOT / ".git").exists():
            # 'dimos[extra]' names the published package; a checkout installs from its lock.
            commands = " or ".join(
                f"`uv sync --extra {extra} --inexact`" for extra in sorted(extras)
            )
            hint += f" with {commands}"
        raise OptionalDependencyError(f"Missing {module!r}. {hint}.") from error


def check_imports(*packages: str) -> None:
    """Check that the selected implementation's packages are installed, without importing them."""
    with explain_missing_extra():
        for package in packages:
            # Finding a submodule imports only its parents, never the package itself.
            if sys.modules.get(package) is None and find_spec(package) is None:
                raise ModuleNotFoundError(f"No module named {package!r}", name=package)
