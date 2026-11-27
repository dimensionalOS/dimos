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
from importlib.metadata import PackageNotFoundError, distribution, packages_distributions
from importlib.util import find_spec
import re
import sys

from packaging.requirements import Requirement
from packaging.utils import canonicalize_name

from dimos.constants import DIMOS_PROJECT_ROOT


class OptionalDependencyError(ImportError):
    """A blueprint could not import because an optional dependency is missing."""


def _matches_requirement(req_name: str, root: str, installed: set[str]) -> bool:
    return (
        req_name in installed
        or req_name in {root, f"python-{root}"}
        or req_name.startswith(f"{root}-")
    )


@contextmanager
def explain_missing_extra() -> Iterator[None]:
    """Rewrite a missing package import into an install hint naming its dimOS extra."""
    try:
        yield
    except ModuleNotFoundError as error:
        module = error.name
        top = (module or "").partition(".")[0]
        if not module or top == "dimos":
            raise
        if "." in module and sys.modules.get(module, True) is not None:
            try:
                top_installed = sys.modules.get(top) is not None or find_spec(top) is not None
            except ModuleNotFoundError:
                top_installed = False
            if top_installed:
                raise
        try:
            package = distribution("dimos")
        except PackageNotFoundError:
            raise error from None

        root = canonicalize_name(top)
        installed = {canonicalize_name(d) for d in packages_distributions().get(top, ())}
        extras: set[str] = set()
        declared: set[str] = set()
        environments: set[str] = set()
        provided_extras = package.metadata.get_all("Provides-Extra") or []
        for requirement in map(Requirement, package.requires or ()):
            req_name = canonicalize_name(requirement.name)
            if req_name == "dimos" or not _matches_requirement(req_name, root, installed):
                continue
            marker = requirement.marker
            if marker is None or marker.evaluate({"extra": ""}):
                raise
            environments.add(re.sub(r' and extra == "[^"]+"$', "", str(marker)))
            declared.update(re.findall(r'\bextra == "([^"]+)"', str(marker)))
            extras.update(extra for extra in provided_extras if marker.evaluate({"extra": extra}))
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
            commands = " or ".join(
                f"`uv sync --extra {extra} --inexact`" for extra in sorted(extras)
            )
            hint += f" with {commands}"
        raise OptionalDependencyError(f"Missing {module!r}. {hint}.") from error


def check_imports(*packages: str) -> None:
    """Check that optional packages are installed without importing them."""
    with explain_missing_extra():
        for package in packages:
            if package in sys.modules:
                if sys.modules[package] is None:
                    raise ModuleNotFoundError(f"No module named {package!r}", name=package)
                continue
            if find_spec(package) is None:
                raise ModuleNotFoundError(f"No module named {package!r}", name=package)
