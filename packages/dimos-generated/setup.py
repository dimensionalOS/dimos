# Copyright 2025-2026 Dimensional Inc.
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

"""Maintainer source build for the independently distributed built-in messages."""

from pathlib import Path
import shutil
import sys

from setuptools import setup
from setuptools.command.sdist import sdist as _sdist

ROOT = Path(__file__).resolve().parent
BUNDLED = ROOT / "_codegen"
REPOSITORY = ROOT.parents[1]
sys.path.insert(0, str(BUNDLED if BUNDLED.is_dir() else REPOSITORY))
from dimos.message_codegen.distribution import write_distribution
from dimos.message_codegen.generate import generate
from dimos.message_codegen.native_build import MessageBuildExt, MessageExtension

OUTPUT = ROOT / "build" / "messages"
generate([], OUTPUT, shared=True)
write_distribution(OUTPUT, "dimos_generated", (), shared=True)


class SourceDistribution(_sdist):
    def make_release_tree(self, base_dir, files):
        super().make_release_tree(base_dir, files)
        target = Path(base_dir) / "_codegen" / "dimos" / "message_codegen"
        source = (
            BUNDLED / "dimos" / "message_codegen"
            if BUNDLED.is_dir()
            else REPOSITORY / "dimos" / "message_codegen"
        )
        shutil.copytree(
            source,
            target,
            ignore=shutil.ignore_patterns(
                "build", "dist", "*.egg-info", "__pycache__", "test_*.py", "target"
            ),
        )
        (target.parent / "__init__.py").write_text("")


setup(
    packages=["dimos_generated_schemas"],
    package_dir={"": "build/messages/python"},
    package_data={"dimos_generated_schemas": ["schemas.json", "schemas/**/*", "package/**/*"]},
    ext_modules=[
        MessageExtension(
            "dimos_generated",
            OUTPUT / "cpp",
            REPOSITORY / "build/message-codegen/install" if not BUNDLED.is_dir() else None,
        )
    ],
    cmdclass={"build_ext": MessageBuildExt, "sdist": SourceDistribution},
)
