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

from collections.abc import Callable
import fnmatch
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tarfile

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import find_packages, setup
from setuptools.command.build_py import build_py as _build_py
from setuptools.command.sdist import sdist as _sdist


def python_is_macos_universal_binary(executable: str | None = None) -> bool:
    """
    Returns True if the given executable is a macOS universal (fat) binary.
    """
    FAT_MAGIC = 0xCAFEBABE  # big-endian fat
    FAT_CIGAM = 0xBEBAFECA  # little-endian fat
    FAT_MAGIC_64 = 0xCAFEBABF  # big-endian fat 64
    FAT_CIGAM_64 = 0xBFBAFECA  # little-endian fat 64

    if executable is None:
        executable = sys.executable

    path = Path(executable)
    if not path.exists():
        return False

    try:
        with path.open("rb") as f:
            header = f.read(4)
            if len(header) < 4:
                return False

            magic = struct.unpack(">I", header)[0]
            return magic in {
                FAT_MAGIC,
                FAT_CIGAM,
                FAT_MAGIC_64,
                FAT_CIGAM_64,
            }
    except OSError:
        return False


TEST_MODULE_PATTERNS = ("test_*.py", "conftest.py")

# The Deno relay (repo-root web/) ships inside the wheel so a pip-installed
# dimos can run it without a checkout. Copied into build_lib below; editable
# installs skip the copy and locate.find_web_dir() resolves the checkout.
# MANIFEST.in grafts web/ so sdist->wheel builds can reproduce this.
# cockpit/sdk deno.json + package.json must ship even without their sources:
# the workspace root lists ./cockpit and ./sdk as members and Deno refuses to
# run the relay when a member's config file is missing.
RELAY_DIST_SOURCES = (
    "deno.json",
    "deno.lock",
    "relay",
    "shared",
    "cockpit/deno.json",
    "cockpit/package.json",
    "cockpit/dist",
    "sdk/deno.json",
    "sdk/package.json",
    "sdk/dist",
    # the gateway's /msgs.js bundles its generated decoders against this one source (experimental/gateway/msgs.py)
    "sdk/src/decoders/lcm.ts",
)
RELAY_DIST_TARGET = os.path.join("dimos", "web", "relay_bridge", "_relay_dist")


def bundle_native_sources(output_root: str) -> None:
    """Keep the native build trees together, with the same paths and modes as the release."""
    bundle = Path("dimos/native_sources.tar")
    destination = Path(output_root) / bundle
    if bundle.is_file() and not Path(".git").exists():
        # Building a wheel from an sdist uses the exact archive it shipped.
        shutil.copyfile(bundle, destination)
        return

    roots = ("dimos", "examples/native-modules")
    if Path(".git").exists():
        # Like bin/native-modules, use tracked flakes. Package working-tree
        # contents so a local wheel includes edits, but never its build outputs.
        tracked = subprocess.check_output(
            ["git", "-c", f"safe.directory={Path.cwd()}", "ls-files", "-z", "--", *roots]
        )
        paths = [Path(os.fsdecode(name)) for name in tracked.split(b"\0") if name]
    else:
        # A downloaded source tree need not have Git metadata.
        paths = []
        for root in roots:
            for directory, dirs, names in os.walk(root):
                here = Path(directory)
                dirs[:] = [
                    name
                    for name in dirs
                    if name
                    not in {".git", ".venv", "__pycache__", "target", "build", "node_modules"}
                    and not name.startswith("result")
                    and here / name != Path("dimos/data")
                ]
                paths.extend(here / name for name in names if not name.endswith((".pyc", ".o")))

    projects = {path.parent for path in paths if path.name == "flake.nix"}
    # The Habitat installer starts ../server.py in its own Python environment.
    inputs = {
        Path("dimos/simulation/habitat/server.py"),
        Path("dimos/simulation/habitat/frames.py"),
    }
    inputs.update(path for path in paths if projects.intersection(path.parents))
    with tarfile.open(destination, "w") as archive:
        for path in sorted(inputs):
            with path.open("rb") as stream:
                # Location, mtime and package-install permissions are not build inputs.
                info = tarfile.TarInfo(path.as_posix())
                info.size = path.stat().st_size
                info.mode = 0o755 if path.stat().st_mode & 0o111 else 0o644
                archive.addfile(info, stream)


class sdist(_sdist):
    def make_release_tree(self, base_dir: str, files: list[str]) -> None:
        super().make_release_tree(base_dir, files)
        bundle_native_sources(base_dir)


class build_py(_build_py):
    def find_package_modules(self, package: str, package_dir: str) -> list[tuple[str, str, str]]:
        find_modules: Callable[[str, str], list[tuple[str, str, str]]] = (
            super().find_package_modules
        )
        return [
            (pkg, mod, filepath)
            for pkg, mod, filepath in find_modules(package, package_dir)
            if not any(
                fnmatch.fnmatch(os.path.basename(filepath), pat) for pat in TEST_MODULE_PATTERNS
            )
        ]

    def run(self) -> None:
        super().run()
        if not getattr(self, "editable_mode", False):
            self._copy_relay_dist()
            bundle_native_sources(self.build_lib)

    def _copy_relay_dist(self) -> None:
        src = Path(__file__).parent / "web"
        if not (src / "relay" / "main.ts").is_file():
            raise RuntimeError(f"relay sources missing at {src}; refusing to build the wheel")
        missing = [
            rel
            for rel in ("cockpit/dist/index.html", "sdk/dist/sdk.js")
            if not (src / rel).is_file()
        ]
        if missing:
            # Wheels must carry the Cockpit and the SDK bundle: there is no
            # fallback debug page, so a dist-less wheel's relay serves no UI
            # (and /sdk.js only a build hint). Building a deliberate
            # python-only wheel (e.g. where deno is unavailable) requires the
            # explicit env-var opt-out, which covers both products.
            if os.environ.get("DIMOS_ALLOW_MISSING_COCKPIT") != "1":
                raise RuntimeError(
                    f"web dist missing ({', '.join(missing)}); run `deno task build` "
                    "in web/sdk and web/cockpit (or `dimos run --local-relay` from a "
                    "checkout builds them), or set DIMOS_ALLOW_MISSING_COCKPIT=1 to "
                    "build a UI-less wheel anyway"
                )
            self.warn(
                f"web dist missing ({', '.join(missing)}); this wheel's relay will have no UI"
            )
        dst = Path(self.build_lib) / RELAY_DIST_TARGET
        for name in RELAY_DIST_SOURCES:
            entry = src / name
            if not entry.exists():  # only the dists may be absent (env-var opt-out above)
                continue
            for path in sorted(entry.rglob("*")) if entry.is_dir() else [entry]:
                # Filter on the path below src: matching path.parts would also
                # hit checkout ancestors (e.g. a build under /work/testdata/).
                rel = path.relative_to(src)
                if path.is_dir() or path.name.endswith("_test.ts") or "testdata" in rel.parts:
                    continue
                target = dst / rel
                self.mkpath(str(target.parent))
                self.copy_file(str(path), str(target))
        # An over-eager exclusion filter would otherwise ship a broken wheel silently.
        if not (dst / "relay" / "main.ts").is_file():
            raise RuntimeError(f"relay copy did not produce {dst / 'relay' / 'main.ts'}")


extra_compile_args = [
    "-O3",  # Maximum optimization
    "-ffast-math",  # Fast floating point
]
# when the python exe is a universal binary, this option fails because the compiler
# call tries to build a matching (e.g. universal) binary, clang doesn't support this option for universal binaries
# if the user is using an arm64 specific binary (ex: nix build) then the optimization exists and is useful
# CIBUILDWHEEL=1 marks the release-wheel build; drop -march=native there so wheels are portable across customer CPUs
if not python_is_macos_universal_binary() and os.environ.get("CIBUILDWHEEL") != "1":
    extra_compile_args.append("-march=native")

# C++ extensions
ext_modules = [
    Pybind11Extension(
        "dimos.navigation.go2.replanning_a_star.min_cost_astar_ext",
        [os.path.join("dimos", "navigation", "go2", "replanning_a_star", "min_cost_astar_cpp.cpp")],
        extra_compile_args=extra_compile_args,
        define_macros=[
            ("NDEBUG", "1"),
        ],
    ),
]

setup(
    packages=find_packages(),
    package_dir={"": "."},
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext, "build_py": build_py, "sdist": sdist},
)
