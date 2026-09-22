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
import shutil


def write_distribution(
    output: Path, module: str, names: tuple[str, ...], version: str = "0.1.0"
) -> None:
    project = output / "python"
    support = module + "_schemas"
    package = project / support
    package.mkdir(parents=True, exist_ok=True)
    (package / "__init__.py").write_text("")
    if (package / "schemas").exists():
        shutil.rmtree(package / "schemas")
    shutil.copytree(output / "schemas", package / "schemas")
    shutil.copyfile(output / "schemas.json", package / "schemas.json")
    (package / "provider.py").write_text(
        "from importlib import import_module\nimport json\nfrom pathlib import Path\n\n"
        "def schema_root():\n    return Path(__file__).with_name('schemas')\n\n"
        "def message_types():\n"
        f"    extension = import_module({module!r})\n"
        "    names = json.loads(Path(__file__).with_name('schemas.json').read_text())\n"
        "    return {name: getattr(getattr(extension, name.split('/')[0]).msg, name.split('/')[-1]) for name in names}\n"
    )
    toolkit = project / "_codegen" / "dimos" / "message_codegen"
    shutil.copytree(
        Path(__file__).parent,
        toolkit,
        dirs_exist_ok=True,
        ignore=shutil.ignore_patterns("__pycache__", "test_*.py"),
    )
    # A namespace directory loses to an installed regular `dimos` package,
    # even with _codegen first on sys.path. Build with the bundled generator.
    (toolkit.parent / "__init__.py").write_text("")
    (project / "pyproject.toml").write_text(
        '[build-system]\nrequires = ["setuptools>=70", "wheel", "pybind11==3.0.1"]\n'
        'build-backend = "setuptools.build_meta"\n'
    )
    (project / "setup.py").write_text(
        "from pathlib import Path\nimport sys\nfrom setuptools import setup\n"
        "sys.path.insert(0, str(Path(__file__).parent / '_codegen'))\n"
        "from dimos.message_codegen.generate import generate\n"
        "from dimos.message_codegen.native_build import MessageExtension, MessageBuildExt\n"
        "root = Path(__file__).parent\n"
        f"generate([root / {support!r} / 'schemas'], root / 'build/messages', {list(names)!r}, {module!r}, version={version!r})\n"
        f"setup(name={module.replace('_', '-')!r}, version={version!r}, packages=[{support!r}],\n"
        f"      package_data={{{support!r}: ['schemas.json', 'schemas/**/*']}},\n"
        f"      entry_points={{'dimos.messages': [{(module + '=' + support + '.provider')!r}]}},\n"
        f"      ext_modules=[MessageExtension({module!r}, root / 'build/messages/cpp')],\n"
        "      cmdclass={'build_ext': MessageBuildExt})\n"
    )
    (project / "MANIFEST.in").write_text(
        f"graft _codegen\ngraft {support}\ninclude pyproject.toml setup.py\n"
        "global-exclude __pycache__ *.pyc\n"
    )
    (project / "message-package.json").write_text(
        json.dumps({"module": module, "types": names}, indent=2) + "\n"
    )
