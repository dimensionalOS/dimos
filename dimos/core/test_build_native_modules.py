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

"""Guard rails for bin/build-native-modules.

That script's AST discovery and flake-reference parsing feed the CI inputs
hash that gates the Cachix publish job: a class or path it misses would let a
module change skip publishing, and the substitute-only test runners would then
hard-fail fetching binaries that were never pushed. These tests cross-check
the script against the live tree with independent, deliberately wider sweeps,
so a refactor that breaks one of its assumptions fails here — in the cheap
no-nix test job — before it can mis-gate a publish.
"""

import ast
import importlib
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
import inspect
import json
import os
from pathlib import Path
import re
from types import ModuleType
from typing import NamedTuple

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT

_SCRIPT_PATH = DIMOS_PROJECT_ROOT / "bin" / "build-native-modules"
if not _SCRIPT_PATH.is_file():
    pytest.skip("dimos is not running from a source checkout", allow_module_level=True)


def _load_script() -> ModuleType:
    loader = SourceFileLoader("build_native_modules", str(_SCRIPT_PATH))
    spec = spec_from_loader(loader.name, loader)
    assert spec is not None
    module = module_from_spec(spec)
    loader.exec_module(module)
    return module


_SCRIPT = _load_script()
_IN_GIT_CHECKOUT = (DIMOS_PROJECT_ROOT / ".git").exists()
# (file, class) form of the script's externally-provisioned exclusions; their
# machine-dependent build commands are exempt from the literal rule, and
# discover() itself fails loudly if an entry goes stale.
_PROVISIONED = {
    (f"{qualname.rsplit('.', 1)[0].replace('.', '/')}.py", qualname.rsplit(".", 1)[-1])
    for qualname in _SCRIPT.EXTERNALLY_PROVISIONED
}


class _ClassDef(NamedTuple):
    file: str
    name: str
    bases: tuple[str, ...]
    command: str | None  # build_command literal defined in this class body
    command_kind: str  # "absent" | "literal" | "opaque"


def _base_names(node: ast.ClassDef) -> tuple[str, ...]:
    names = []
    for base in node.bases:
        target = base.value if isinstance(base, ast.Subscript) else base
        if isinstance(target, ast.Attribute):
            names.append(target.attr)
        elif isinstance(target, ast.Name):
            names.append(target.id)
    return tuple(names)


def _own_build_command(node: ast.ClassDef) -> tuple[str, str | None]:
    for stmt in node.body:
        if isinstance(stmt, ast.AnnAssign) and isinstance(stmt.target, ast.Name):
            target, value = stmt.target.id, stmt.value
        elif (
            isinstance(stmt, ast.Assign)
            and len(stmt.targets) == 1
            and isinstance(stmt.targets[0], ast.Name)
        ):
            target, value = stmt.targets[0].id, stmt.value
        else:
            continue
        if target != "build_command" or value is None:
            continue
        if isinstance(value, ast.Constant) and (
            value.value is None or isinstance(value.value, str)
        ):
            return "literal", value.value
        return "opaque", None
    return "absent", None


def _scan_all_config_classes() -> list[_ClassDef]:
    """Every class in production dimos code, nested ones included."""
    classes = []
    for dirpath, dirnames, filenames in os.walk(DIMOS_PROJECT_ROOT / "dimos"):
        dirnames[:] = sorted(d for d in dirnames if d not in _SCRIPT.IGNORED_DIRS)
        for filename in sorted(filenames):
            if not filename.endswith(".py") or filename.startswith(_SCRIPT._SKIP_PREFIXES):
                continue
            path = Path(dirpath) / filename
            rel = path.relative_to(DIMOS_PROJECT_ROOT).as_posix()
            for node in ast.walk(ast.parse(path.read_text(), filename=rel)):
                if isinstance(node, ast.ClassDef):
                    kind, command = _own_build_command(node)
                    classes.append(_ClassDef(rel, node.name, _base_names(node), command, kind))
    return classes


def _closure_nix_configs(classes: list[_ClassDef]) -> set[tuple[str, str]]:
    """(file, class) for every transitive NativeModuleConfig subclass whose
    effective build_command default (own, or inherited from another config in
    the closure) mentions nix."""
    by_name: dict[str, list[_ClassDef]] = {}
    for cls in classes:
        by_name.setdefault(cls.name, []).append(cls)
    closure = {"NativeModuleConfig"}
    while True:
        added = {
            cls.name
            for cls in classes
            if cls.name not in closure and any(base in closure for base in cls.bases)
        }
        if not added:
            break
        closure |= added

    def effective_command(cls: _ClassDef, seen: frozenset[str]) -> tuple[str, str | None]:
        if cls.command_kind != "absent":
            return cls.command_kind, cls.command
        for base in cls.bases:
            if base in closure and base != "NativeModuleConfig" and base not in seen:
                for parent in by_name.get(base, []):
                    kind, command = effective_command(parent, seen | {cls.name})
                    if kind != "absent":
                        return kind, command
        return "absent", None

    nix_configs = set()
    for cls in classes:
        if cls.name not in closure or cls.name == "NativeModuleConfig":
            continue
        if (cls.file, cls.name) in _PROVISIONED:
            # The exclusion exists because the command is machine-dependent and
            # unreadable. If it becomes statically readable, the publish gate
            # can (and must) track it — the entry would then hide real inputs.
            assert cls.command_kind == "opaque", (
                f"{cls.file}: {cls.name}.build_command is statically readable — remove it"
                " from EXTERNALLY_PROVISIONED in bin/build-native-modules"
            )
            continue
        kind, command = effective_command(cls, frozenset())
        assert kind != "opaque", (
            f"{cls.file}: {cls.name}.build_command must default to a plain string literal "
            "so bin/build-native-modules can read it without importing dimos"
        )
        if command is not None and "nix" in command:
            nix_configs.add((cls.file, cls.name))
    return nix_configs


def test_discovery_is_complete_and_flat() -> None:
    """The script's direct-base discovery must find every config the transitive
    closure finds. A mismatch means a module (e.g. a depth-2 subclass) would
    silently escape the publish gate: flatten the hierarchy, or extend the
    script's discovery to match."""
    expected = _closure_nix_configs(_scan_all_config_classes())
    discovered = {
        (module.source, module.qualname.rsplit(".", 1)[-1]) for module in _SCRIPT.discover()
    }
    assert discovered == expected
    assert discovered, "expected at least one nix-built native module"


def test_ast_extraction_matches_runtime() -> None:
    """The AST-read defaults must equal what pydantic resolves at runtime —
    this equivalence is what lets CI discover modules without installing dimos.
    The build dir mirrors NativeModule's cwd resolution, which anchors on the
    defining file: config classes must live beside their module class."""
    modules = _SCRIPT.discover()
    assert modules
    for module in modules:
        dotted, class_name = module.qualname.rsplit(".", 1)
        config_class = getattr(importlib.import_module(dotted), class_name)
        fields = config_class.model_fields
        assert fields["build_command"].default == module.build_command
        cwd = fields["cwd"].default
        base_dir = Path(inspect.getfile(config_class)).resolve().parent
        runtime_dir = Path(os.path.normpath(base_dir if cwd is None else base_dir / cwd))
        assert runtime_dir == (DIMOS_PROJECT_ROOT / module.build_dir).resolve()


@pytest.mark.skipif(not _IN_GIT_CHECKOUT, reason="needs git HEAD for object hashes")
def test_manifest_is_deterministic() -> None:
    modules = _SCRIPT.discover()
    manifest = _SCRIPT.build_manifest(modules)
    assert manifest == _SCRIPT.build_manifest(modules)
    lines = manifest.splitlines()
    assert lines[0] == f"salt {_SCRIPT.MARKER_SALT}"
    assert len([line for line in lines if line.startswith("module ")]) == len(modules)


def test_dynamic_path_composition_is_caught(tmp_path: Path) -> None:
    """A relative path continued with `+ var` or `${…}` builds the real
    reference at eval time; hashing just the literal prefix could cover a
    sibling of the actual tree, so the parser must refuse instead of guessing."""
    flake = tmp_path / "flake.nix"
    for snippet in (
        "livox-common = ../../common + suffix;",
        "src = ./cpp + name;",
        "src = ./modules/${variant};",
        "src = ./mod${variant};",
        "src = ../../common\n  + suffix;",
    ):
        flake.write_text(snippet + "\n")
        with pytest.raises(SystemExit, match="dynamically composed"):
            _SCRIPT._flake_refs(flake)


def test_unanalyzable_references_are_caught(tmp_path: Path) -> None:
    """`self` dereferences and builtins fetchers reach repo (or unpinned
    remote) content without any relative-path token, so no textual scan can
    map them to input trees — the parser must refuse rather than under-hash."""
    flake = tmp_path / "flake.nix"
    for snippet in (
        'cmakeFlags = [ "-DX=${self}/native/cpp" ];',
        'src = self + "/native";',
        "p = self.outPath;",
        "src = builtins.fetchGit ../../..;",
        'src = builtins.fetchTarball { url = "https://example.org/x.tar"; };',
    ):
        flake.write_text(snippet + "\n")
        with pytest.raises(SystemExit, match="cannot follow"):
            _SCRIPT._flake_refs(flake)


# Anything relative-path shaped, matched with no context: deliberately wider
# than the script's parser so a reference form it misses still trips this.
_RAW_REF = re.compile(r"(?P<scheme>git\+file:|path:)?(?P<path>\.{1,2}/[\w.@+/-]*)")


@pytest.mark.skipif(not _IN_GIT_CHECKOUT, reason="needs git HEAD for object hashes")
def test_flake_refs_resolve_and_are_covered() -> None:
    """Crude second opinion on the flake parsing: sweep raw flake text
    (comments and strings included) and require every existing relative
    reference to be rev-pinned (git+file:) or inside the hashed input set;
    also require every hashed path to be tracked at HEAD."""
    for module in _SCRIPT.discover():
        if not _SCRIPT._is_local_flake(module):
            continue
        covered: set[str] = _SCRIPT._collect_input_paths(module)
        for rel in sorted(covered):
            assert (DIMOS_PROJECT_ROOT / rel).exists()
            _SCRIPT._git_object_hash(rel)  # SystemExit if not tracked at HEAD
            flake = DIMOS_PROJECT_ROOT / rel / "flake.nix"
            if not flake.is_file():
                continue  # plain source tree (e.g. native/cpp), nothing to sweep
            for match in _RAW_REF.finditer(flake.read_text()):
                if match.group("scheme") == "git+file:":
                    url = match.group("path").split("?", 1)[0]
                    lock = json.loads((flake.parent / "flake.lock").read_text())
                    assert any(
                        isinstance(node, dict)
                        and node.get("locked", {}).get("type") == "git"
                        and node["locked"].get("url") == f"file:{url}"
                        and "rev" in node["locked"]
                        for node in lock["nodes"].values()
                    ), (
                        f"{flake}: git+file:{url} has no rev-pinned flake.lock node — an"
                        " unlocked self-input re-locks at HEAD on every build, changing the"
                        " derivation on every commit behind the publish gate's back"
                    )
                    continue
                token = match.group("path").split("?", 1)[0]
                target = os.path.relpath(os.path.normpath(flake.parent / token), DIMOS_PROJECT_ROOT)
                if not (DIMOS_PROJECT_ROOT / target).exists():
                    continue  # comment/string noise; real broken refs fail discovery loudly
                assert any(target == c or target.startswith(c + "/") for c in covered), (
                    f"{flake}: reference {token!r} resolves to {target!r}, outside the hashed "
                    "input set — teach bin/build-native-modules._FLAKE_REF the new form"
                )
