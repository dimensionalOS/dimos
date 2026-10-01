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

"""Emit a standard setuptools source package around generated native messages."""

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
    shared: bool = False,
    source_output: Path | None = None,
) -> None:
    manifest = json.loads((output / "message-package.json").read_text())
    names = tuple(manifest["owned"])
    shared = shared or manifest["shared"]
    project = output / "python"
    support = module + "_schemas"
    package = project / support
    package.mkdir(parents=True, exist_ok=True)
    (package / "__init__.py").write_text("")
    if (package / "schemas").exists():
        shutil.rmtree(package / "schemas")
    shutil.copytree(output / "schemas", package / "schemas")
    shutil.copyfile(output / "schemas.json", package / "schemas.json")
    resources = package / "package"
    if resources.exists():
        shutil.rmtree(resources)
    resources.mkdir()
    for directory in ("schemas", "include", "lib", "rust"):
        shutil.copytree(
            output / directory,
            resources / directory,
            ignore=shutil.ignore_patterns("target", "Cargo.lock"),
        )
    cargo_manifest = resources / "rust" / "Cargo.toml"
    cargo_manifest.write_text(re.sub(r', path = "[^"\n]*"', "", cargo_manifest.read_text()))
    shutil.copyfile(output / "message-package.json", resources / "message-package.json")
    shutil.copyfile(output / "schemas.json", resources / "schemas.json")
    (package / "provider.py").write_text(
        "from importlib import import_module\nimport json\nfrom pathlib import Path\n\n"
        "def schema_root():\n    return Path(__file__).with_name('schemas')\n\n"
        "def package_root():\n    return Path(__file__).with_name('package')\n\n"
        "def message_types():\n"
        f"    extension = import_module({module!r})\n"
        "    names = json.loads((package_root() / 'message-package.json').read_text())['owned']\n"
        "    return {name: getattr(getattr(extension, name.split('/')[0]).msg, name.split('/')[-1]) for name in names}\n"
    )
    toolkit = project / "_codegen" / "dimos" / "message_codegen"
    shutil.copytree(
        Path(__file__).parent,
        toolkit,
        dirs_exist_ok=True,
        ignore=shutil.ignore_patterns(
            "__pycache__", "test_*.py", "build", "dist", "*.egg-info", "target"
        ),
    )
    # A namespace directory loses to an installed regular `dimos` package,
    # even with _codegen first on sys.path. Build with the bundled generator.
    (toolkit.parent / "__init__.py").write_text("")
    (project / "pyproject.toml").write_text(
        "[build-system]\nrequires = "
        + json.dumps(
            [
                "setuptools>=70",
                "wheel",
                "pybind11==3.0.1",
                "cmake>=3.20",
                *[dep.module.replace("_", "-") + "==" + dep.version for dep in dependencies],
            ]
        )
        + "\n"
        + 'build-backend = "setuptools.build_meta"\n'
    )
    (project / "setup.py").write_text(
        "from pathlib import Path\nimport os\nimport sys\nfrom setuptools import setup\n"
        "sys.path.insert(0, str(Path(__file__).parent / '_codegen'))\n"
        "from dimos.message_codegen.generate import generate\n"
        "from dimos.message_codegen.ownership import Dependency\n"
        "from importlib import import_module\n"
        "from dimos.message_codegen.native_build import MessageExtension, MessageBuildExt\n"
        "root = Path(__file__).parent\n"
        f"dependencies = tuple(Dependency.load(Path(import_module(name + '_schemas').__file__).parent / 'package') for name in {[dep.module for dep in dependencies]!r})\n"
        + "os.environ['CMAKE_PREFIX_PATH'] = os.pathsep.join([str(dep.root) for dep in dependencies] + [os.environ.get('CMAKE_PREFIX_PATH', '')])\n"
        + (
            f"generate([root / {support!r} / 'schemas'], root / 'build/messages', {list(names)!r}, {module!r}, version={version!r}, dependencies=dependencies, shared={shared!r})\nmessage_source = root / 'build/messages/cpp'\n"
            if source_output is None
            else f"message_source = Path({str(source_output / 'cpp')!r})\n"
        )
        + f"setup(name={module.replace('_', '-')!r}, version={version!r}, packages=[{support!r}],\n"
        f"      package_data={{{support!r}: ['schemas.json', 'schemas/**/*', 'package/**/*']}},\n"
        f"      install_requires={[dep.module.replace(chr(95), chr(45)) + '==' + dep.version for dep in dependencies]!r},\n"
        f"      entry_points={{'dimos.messages': [{(module + '=' + support + '.provider')!r}]}},\n"
        f"      ext_modules=[MessageExtension({module!r}, message_source)],\n"
        "      cmdclass={'build_ext': MessageBuildExt})\n"
    )
    (project / "MANIFEST.in").write_text(
        f"graft _codegen\ngraft {support}\ninclude pyproject.toml setup.py\n"
        "global-exclude __pycache__ *.pyc\n"
    )
    shutil.copyfile(output / "message-package.json", project / "message-package.json")
