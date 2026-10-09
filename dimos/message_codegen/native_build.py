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

"""Linux source preparation using upstream CMake projects and a writable cache.

Nothing calls this module from package installation, schema discovery or import.
No package-manager implementation lives here: CMake owns upstream source builds.
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from hashlib import sha256
from importlib import metadata
from importlib.util import find_spec
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
from typing import Any

if sys.platform == "linux":
    import fcntl

from . import cpp
from .definitions import Definitions
from .ownership import Dependency

TOOLKIT = Path(__file__).parent


def _run(command: list[str], environment: dict[str, str]) -> None:
    subprocess.run(command, env=environment, check=True)


@contextmanager
def _locked(directory: Path) -> Iterator[None]:
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / "build.lock").open("w") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        yield


def _toolchain_key() -> str:
    if sys.platform != "linux":
        raise NotImplementedError(
            "Native ROSIDL source preparation is currently validated on Linux only"
        )
    cxx = shlex.split(os.environ.get("CXX") or "c++")
    cc = shlex.split(os.environ.get("CC") or "cc")
    if not cxx or not cc:
        raise RuntimeError("CC and CXX must name a compiler command")
    for command in ["cmake", cxx[0], cc[0]]:
        if shutil.which(command) is None:
            raise RuntimeError(
                f"Missing native build tool: {command}; install your platform toolchain first"
            )
    missing = [
        name
        for name in ["pip", "setuptools", "wheel", "em", "lark", "catkin_pkg", "yaml"]
        if find_spec(name) is None
    ]
    if missing:
        raise RuntimeError(
            f"Missing native build Python prerequisites: {missing}; install dimos-message-build[native] in the build environment"
        )
    compiler = subprocess.check_output([*cxx, "--version"], text=True)
    target = subprocess.check_output([*cxx, "-dumpmachine"], text=True)
    c_compiler = subprocess.check_output([*cc, "--version"], text=True)
    cmake = subprocess.check_output(["cmake", "--version"], text=True)
    flags = {
        name: os.environ.get(name, "")
        for name in ["CC", "CXX", "CFLAGS", "CXXFLAGS", "LDFLAGS", "CMAKE_TOOLCHAIN_FILE"]
    }
    versions = {
        name: metadata.version(name)
        for name in ["pip", "empy", "lark", "catkin-pkg", "PyYAML", "setuptools", "wheel"]
    }
    return sha256(
        (
            compiler
            + c_compiler
            + cmake
            + target
            + sys.version
            + sys.executable
            + json.dumps([flags, versions], sort_keys=True)
        ).encode()
    ).hexdigest()


def build_environment(prefixes: list[Path], base: dict[str, str] | None = None) -> dict[str, str]:
    """Expose upstream ament Python resources to ordinary CMake consumers."""
    environment = os.environ if base is None else base
    python = f"lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages"
    return {
        **environment,
        "PATH": os.pathsep.join([str(Path(sys.executable).parent), environment.get("PATH", "")]),
        "CMAKE_PREFIX_PATH": os.pathsep.join(
            [*map(str, prefixes), environment.get("CMAKE_PREFIX_PATH", "")]
        ),
        "AMENT_PREFIX_PATH": os.pathsep.join(map(str, prefixes)),
        "PYTHONPATH": os.pathsep.join(
            [*(str(prefix / python) for prefix in prefixes), environment.get("PYTHONPATH", "")]
        ),
        "PIP_DISABLE_PIP_VERSION_CHECK": "1",
        "PIP_NO_INDEX": "1",  # Python upstream sources build without resolving/downloading dependencies.
    }


def prepare_support(
    cache: Path, *, offline: bool = False, source_dirs: dict[str, Path] | None = None
) -> Path:
    """Build pinned support libraries locally; source overrides support offline preparation."""
    key = _toolchain_key()
    lock = (TOOLKIT / "native_sources.json").read_bytes()
    recipe = (TOOLKIT / "templates/native-support.cmake").read_bytes()
    snapshots = {name: source_digest(path) for name, path in sorted((source_dirs or {}).items())}
    digest = sha256(
        key.encode() + lock + recipe + json.dumps(snapshots, sort_keys=True).encode()
    ).hexdigest()
    directory = cache / "support" / digest
    prefix = directory / "install"
    with _locked(directory):
        marker = directory / "complete.json"
        if complete(prefix, marker):
            return prefix
        if marker.exists() and (directory / "build").exists():
            shutil.rmtree(directory / "build")
        source = directory / "source"
        source.mkdir(exist_ok=True)
        (source / "CMakeLists.txt").write_bytes(recipe)
        (source / "native_sources.json").write_bytes(lock)
        overrides = source_dirs or {}
        expected = {*json.loads(lock)["repositories"], "fastcdr"}
        if overrides.keys() - expected:
            raise ValueError(
                f"Unknown upstream source override: {sorted(overrides.keys() - expected)}"
            )
        if offline and any(
            name not in overrides and not (directory / "upstream" / (name + "-src")).is_dir()
            for name in expected
        ):
            raise RuntimeError(
                "Offline native preparation requires every pinned source in the CMake cache or source_dirs"
            )
        env = build_environment([prefix])
        _run(
            [
                "cmake",
                "-S",
                str(source),
                "-B",
                str(directory / "build"),
                "-DPython3_EXECUTABLE=" + sys.executable,
                "-DFETCHCONTENT_BASE_DIR=" + str(directory / "upstream"),
                "-DDIMOS_SUPPORT_PREFIX=" + str(prefix),
                "-DFETCHCONTENT_FULLY_DISCONNECTED=" + ("ON" if offline else "OFF"),
                *[
                    "-DFETCHCONTENT_SOURCE_DIR_" + name.upper() + "=" + str(path.resolve())
                    for name, path in sorted(overrides.items())
                ],
            ],
            env,
        )
        _run(["cmake", "--build", str(directory / "build"), "--parallel", "2"], env)
        save_complete(prefix, marker)
    return prefix


def validate_cpp_closure(root: Path) -> None:
    """Reject incompatible installed schemas before invoking a toolchain or fetching sources."""
    manifests: dict[str, dict[str, Any]] = {}
    owners: dict[str, str] = {}
    schemas: dict[str, str] = {}
    active: set[str] = set()

    def visit(package_root: Path) -> Dependency:
        package = Dependency.load(package_root)
        manifest = json.loads((package_root / "message-package.json").read_text())
        name = package.module
        if name in active:
            raise ValueError(f"Cyclic message dependency: {name}")
        if name in manifests:
            if manifests[name] != manifest:
                raise ValueError(f"Conflicting message package identity/version: {name}")
            return package
        if manifest.get("unsupported", {}).get("cpp"):
            raise NotImplementedError(
                f"Unsupported native C++ semantics: {manifest['unsupported']['cpp']}"
            )
        active.add(name)
        for dependency, version in sorted(manifest["dependencies"].items()):
            spec = find_spec(dependency + "_schemas")
            if spec is None or spec.origin is None:
                raise ValueError(f"Missing installed message dependency: {dependency}")
            found = visit(Path(spec.origin).parent / "package")
            if found.module != dependency or found.version != version:
                raise ValueError(f"Dependency version/identity mismatch: {dependency}=={version}")
        definitions = Definitions([package_root / "schemas"])
        for message in definitions.resolve(list(package.owned)):
            digest = sha256(definitions.schema(message.name).encode()).hexdigest()
            if package.schemas.get(message.name) != digest:
                raise ValueError(f"Dependency schema mismatch: {message.name}")
            if schemas.setdefault(message.name, digest) != digest:
                raise ValueError(f"Dependency schema mismatch: {message.name}")
        for message_name in package.owned:
            if owners.setdefault(message_name, name) != name:
                raise ValueError(f"Multiple owners for message {message_name}")
        active.remove(name)
        manifests[name] = manifest
        return package

    visit(root)
    if schemas.keys() - owners.keys():
        raise ValueError(f"Missing message owners: {sorted(schemas.keys() - owners.keys())}")


def prepare_cpp(
    root: Path,
    *,
    cache: Path | None = None,
    offline: bool = False,
    source_dirs: dict[str, Path] | None = None,
) -> Path:
    """Prepare a message source package and its dependency-owned C++ types."""
    cache = (
        cache or Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "dimos/native"
    ).resolve()
    validate_cpp_closure(root.resolve())
    support = prepare_support(cache, offline=offline, source_dirs=source_dirs)
    completed: dict[str, tuple[str, Path]] = {}
    active: set[str] = set()

    def build(package_root: Path) -> Path:
        manifest = json.loads((package_root / "message-package.json").read_text())
        package = Dependency.load(package_root)
        name = package.module
        if name in active:
            raise ValueError(f"Cyclic message dependency: {name}")
        if name in completed:
            if completed[name][0] != package.version:
                raise ValueError(f"Conflicting dependency versions: {name}")
            return completed[name][1]
        if manifest.get("unsupported", {}).get("cpp"):
            raise NotImplementedError(
                f"Unsupported native C++ semantics: {manifest['unsupported']['cpp']}"
            )
        active.add(name)
        dependencies = []
        for dependency, version in sorted(manifest["dependencies"].items()):
            spec = find_spec(dependency + "_schemas")
            if spec is None or spec.origin is None:
                raise ValueError(f"Missing installed message dependency: {dependency}")
            dependency_root = Path(spec.origin).parent / "package"
            if Dependency.load(dependency_root).version != version:
                raise ValueError(f"Dependency version mismatch: {dependency}")
            dependencies.append(build(dependency_root))
        definitions = Definitions([package_root / "schemas"])
        messages = definitions.resolve(list(package.owned))
        for message in messages:
            if (
                package.schemas.get(message.name)
                != sha256(definitions.schema(message.name).encode()).hexdigest()
            ):
                raise ValueError(f"Dependency schema mismatch: {message.name}")
        owned = tuple(message for message in messages if message.name in package.owned)
        key = sha256(
            (json.dumps(manifest, sort_keys=True) + str(support) + repr(dependencies)).encode()
            + Path(cpp.__file__).read_bytes()
        ).hexdigest()
        directory = cache / "messages" / key
        prefix = directory / "install"
        with _locked(directory):
            if not complete(prefix, directory / "complete.json"):
                if (directory / "complete.json").exists() and (directory / "build").exists():
                    shutil.rmtree(directory / "build")
                source = directory / "source"
                if source.exists():
                    shutil.rmtree(source)
                cpp.write_project(source, owned, name, package.version)
                prefixes = list(
                    dict.fromkeys(
                        [
                            support,
                            *(
                                Path(path)
                                for dependency in dependencies
                                for path in installed_prefixes(dependency)
                            ),
                        ]
                    )
                )
                env = build_environment(prefixes)
                _run(
                    [
                        "cmake",
                        "-S",
                        str(source),
                        "-B",
                        str(directory / "build"),
                        "-DPython3_EXECUTABLE=" + sys.executable,
                        "-DCMAKE_INSTALL_PREFIX=" + str(prefix),
                        "-DCMAKE_PREFIX_PATH=" + ";".join(map(str, prefixes)),
                        "-DDIMOS_RUNTIME_PATHS=" + ";".join(str(p / "lib") for p in prefixes),
                    ],
                    env,
                )
                _run(["cmake", "--build", str(directory / "build"), "--parallel", "2"], env)
                _run(["cmake", "--install", str(directory / "build")], env)
                (prefix / "native-prefixes.json").write_text(
                    json.dumps([str(prefix), str(support), *map(str, dependencies)]) + "\n"
                )
                save_complete(prefix, directory / "complete.json")
        active.remove(name)
        completed[name] = (package.version, prefix)
        return prefix

    return build(root.resolve())


def source_digest(source: Path) -> str:
    """Hash source content, including removals/renames, not generated build directories."""
    digest = sha256()
    for path in sorted(source.rglob("*")):
        relative = path.relative_to(source)
        if any(
            part in {".git", "build", "target", "result", "__pycache__", ".venv", "dist"}
            or part.endswith(".egg-info")
            for part in relative.parts
        ):
            continue
        if path.is_file():
            digest.update(relative.as_posix().encode())
            digest.update(path.read_bytes())
    return digest.hexdigest()


def complete(prefix: Path, marker: Path) -> bool:
    if not marker.is_file():
        return False
    files = json.loads(marker.read_text())
    return bool(files) and all(
        (prefix / name).is_file() and sha256((prefix / name).read_bytes()).hexdigest() == expected
        for name, expected in files.items()
    )


def save_complete(prefix: Path, marker: Path) -> None:
    files = {
        str(p.relative_to(prefix)): sha256(p.read_bytes()).hexdigest()
        for p in sorted(prefix.rglob("*"))
        if p.is_file() and "__pycache__" not in p.parts
    }
    marker.write_text(json.dumps(files, sort_keys=True) + "\n")


def stage_module(source: Path, executable: Path, cache: Path | None = None) -> tuple[Path, Path]:
    """Stage an opted-in CMake module into writable storage before its existing build command."""
    relative_executable = executable.relative_to(source)
    cache = cache or Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "dimos/native"
    target = cache / "modules" / source_digest(source) / "source"
    with _locked(target.parent):
        if not target.exists():
            shutil.copytree(
                source,
                target,
                ignore=shutil.ignore_patterns(
                    ".git",
                    "build",
                    "target",
                    "result",
                    "__pycache__",
                    ".venv",
                    "dist",
                    "*.egg-info",
                ),
            )
    for directory in [target, *(p for p in target.rglob("*") if p.is_dir())]:
        directory.chmod(directory.stat().st_mode | 0o700)
    return target, target / relative_executable


def installed_prefixes(prefix: Path) -> list[str]:
    result = [str(prefix)]
    for value in json.loads((prefix / "native-prefixes.json").read_text()):
        if value not in result:
            result.append(value)
        nested = Path(value) / "native-prefixes.json"
        if value != str(prefix) and nested.is_file():
            result.extend(path for path in installed_prefixes(Path(value)) if path not in result)
    return result


def write_cmake_toolchain(prefix: Path, destination: Path) -> Path:
    """Export an ordinary CMake toolchain file for this local prepared source build."""
    prefixes = [Path(value) for value in installed_prefixes(prefix)]
    spec = find_spec("dimos")
    if spec is not None and spec.submodule_search_locations:
        package = Path(next(iter(spec.submodule_search_locations)))
        for sdk in [package / "_native/cpp", package.parent / "native/cpp"]:
            if (sdk / "dimos_nativeConfig.cmake").is_file():
                prefixes.append(sdk)
                break
    environment = build_environment(prefixes)
    destination.write_text(
        "# Local build paths; regenerate with dimos build after moving the environment.\n"
        + "list(PREPEND CMAKE_PREFIX_PATH "
        + " ".join(json.dumps(str(path)) for path in prefixes)
        + ")\n"
        + "set(Python3_EXECUTABLE "
        + json.dumps(sys.executable)
        + ' CACHE FILEPATH "Python used for native support" FORCE)\n'
        + "set(ENV{PYTHONPATH} "
        + json.dumps(environment["PYTHONPATH"])
        + ")\n"
    )
    return destination
