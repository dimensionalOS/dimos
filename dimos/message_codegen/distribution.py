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

"""Package generated Python source and schema/native-source resources without compilation."""

from __future__ import annotations

import json
from pathlib import Path
import re
import shutil

from .ownership import Dependency


def write_distribution(
    output: Path,
    module: str,
    names: tuple[str, ...],
    version: str = "0.1.0",
    *,
    dependencies: tuple[Dependency, ...] = (),
) -> None:
    project = output / "python"
    support = module + "_schemas"
    package = project / support
    package.mkdir(parents=True, exist_ok=True)
    (package / "__init__.py").write_text('"""Generated message/schema provider."""\n')
    for filename in ["schemas", "package"]:
        if (package / filename).exists():
            shutil.rmtree(package / filename)
    shutil.copytree(output / "schemas", package / "schemas")
    shutil.copyfile(output / "schemas.json", package / "schemas.json")
    resources = package / "package"
    resources.mkdir()
    for directory in ["schemas", "cpp", "include", "lib", "rust"]:
        if (output / directory).is_dir():
            shutil.copytree(
                output / directory,
                resources / directory,
                ignore=shutil.ignore_patterns("target", "Cargo.lock"),
            )
    cargo_manifest = resources / "rust/Cargo.toml"
    if cargo_manifest.is_file():
        cargo_manifest.write_text(re.sub(r', path = "[^"\n]*"', "", cargo_manifest.read_text()))
    if (output / "message-package.json").is_file():
        shutil.copyfile(output / "message-package.json", resources / "message-package.json")
    shutil.copyfile(output / "schemas.json", resources / "schemas.json")
    manifest = (
        json.loads((output / "message-package.json").read_text())
        if (output / "message-package.json").is_file()
        else {"owned": names}
    )
    owned = manifest["owned"]
    (package / "provider.py").write_text(
        "\n".join(Path(__file__).read_text().splitlines()[:13])
        + "\n\n"
        + "from importlib import import_module\nfrom pathlib import Path\n\n"
        "def schema_root():\n    return Path(__file__).with_name('schemas')\n\n"
        "def package_root():\n    return Path(__file__).with_name('package')\n\n"
        "def message_types():\n"
        f"    values = import_module({module!r})\n"
        f"    names = {list(owned)!r}\n"
        "    return {name: getattr(getattr(values, name.split('/')[0]).msg, name.split('/')[-1]) for name in names}\n"
    )
    requirements = ["numpy>=1.26.4", "rosbags==0.11.0", "dimos-message-build==0.1.0"] + [
        dep.module.replace("_", "-") + "==" + dep.version for dep in dependencies
    ]
    (project / "pyproject.toml").write_text(
        '[build-system]\nrequires = ["setuptools>=70", "wheel"]\nbuild-backend = "setuptools.build_meta"\n'
    )
    (project / "setup.py").write_text(
        "from setuptools import find_packages, setup\n"
        f"setup(name={module.replace('_', '-')!r}, version={version!r}, packages=find_packages(),\n"
        f"      install_requires={requirements!r},\n"
        f"      package_data={{{module!r}: ['py.typed', '**/*.pyi', '*.pyi'], {support!r}: ['schemas.json', 'schemas/**/*', 'package/**/*']}},\n"
        f"      entry_points={{'dimos.messages': [{(module + '=' + support + '.provider')!r}]}})\n"
    )
    (project / "MANIFEST.in").write_text(
        f"graft {module}\ngraft {support}\ninclude pyproject.toml setup.py\nglobal-exclude __pycache__ *.pyc\n"
    )
    (project / "message-package.json").write_text(
        json.dumps({"module": module, "types": list(owned)}, indent=2) + "\n"
    )
