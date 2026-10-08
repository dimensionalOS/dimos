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

"""Guard rails for bin/native-modules.

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
import json
import os
from pathlib import Path
import re
import subprocess
from types import ModuleType
from typing import NamedTuple

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT

_SCRIPT_PATH = DIMOS_PROJECT_ROOT / "bin" / "native-modules"
if not _SCRIPT_PATH.is_file():
    pytest.skip("dimos is not running from a source checkout", allow_module_level=True)


def _load_script(path: Path, name: str) -> ModuleType:
    loader = SourceFileLoader(name, str(path))
    spec = spec_from_loader(loader.name, loader)
    assert spec is not None
    module = module_from_spec(spec)
    loader.exec_module(module)
    return module


_SCRIPT = _load_script(_SCRIPT_PATH, "native_modules")
_RELOCK = _load_script(DIMOS_PROJECT_ROOT / "bin" / "relock-shared-flakes", "relock_shared_flakes")
_IN_GIT_CHECKOUT = (DIMOS_PROJECT_ROOT / ".git").exists()


class _ClassDef(NamedTuple):
    file: str
    name: str
    bases: tuple[str, ...]
    command: str | None  # build_command literal defined in this class body
    command_kind: str  # "absent" | "literal" | "opaque"
    owns_source_dir: bool = False


def _base_names(node: ast.ClassDef) -> tuple[str, ...]:
    names = []
    for base in node.bases:
        target = base.value if isinstance(base, ast.Subscript) else base
        if isinstance(target, ast.Attribute):
            names.append(target.attr)
        elif isinstance(target, ast.Name):
            names.append(target.id)
    return tuple(names)


def _own_default(node: ast.ClassDef, field: str) -> tuple[str, str | None]:
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
        if target != field or value is None:
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
                    kind, command = _own_default(node, "build_command")
                    source_kind, _ = _own_default(node, "source_dir")
                    classes.append(
                        _ClassDef(
                            rel,
                            node.name,
                            _base_names(node),
                            command,
                            kind,
                            source_kind != "absent",
                        )
                    )
    return classes


def _closure_nix_configs(classes: list[_ClassDef]) -> set[tuple[str, str]]:
    """Build owners, including indirect configs that change a command or directory.

    Unmodified inherited builds are covered by their defining ancestor.
    """
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

    def effective_command(
        cls: _ClassDef, seen: frozenset[str]
    ) -> tuple[str, str | None, _ClassDef]:
        if cls.command_kind != "absent":
            return cls.command_kind, cls.command, cls
        for base in cls.bases:
            if base in closure and base != "NativeModuleConfig" and base not in seen:
                for parent in by_name.get(base, []):
                    kind, command, owner = effective_command(parent, seen | {cls.name})
                    if kind != "absent":
                        return kind, command, cls if cls.owns_source_dir else owner
        return "absent", None, cls

    nix_configs = set()
    for cls in classes:
        if cls.name not in closure or cls.name == "NativeModuleConfig":
            continue
        kind, command, owner = effective_command(cls, frozenset())
        assert kind != "opaque", (
            f"{cls.file}: {cls.name}.build_command must default to a plain string literal "
            "so bin/native-modules can read it without importing dimos"
        )
        # Deliberately independent of the production command parser: options
        # before `build` must not silently remove a config from both scans.
        tokens = command.split() if command else []
        if "nix" in tokens and "build" in tokens and "develop" not in tokens:
            nix_configs.add((owner.file, owner.name))
    return nix_configs


def test_discovery_covers_every_build() -> None:
    """Every distinct native build must participate in the publish gate."""
    expected = _closure_nix_configs(_scan_all_config_classes())
    discovered = {
        (module.source, module.qualname.rsplit(".", 1)[-1]) for module in _SCRIPT.discover()
    }
    assert discovered == expected
    assert discovered, "expected at least one nix-built native module"


def test_recorder_is_in_the_publish_manifest() -> None:
    recorder = next(
        module
        for module in _SCRIPT.discover()
        if module.qualname == "dimos.experimental.memory.rust_recorder.RustRecorderConfig"
    )
    assert recorder.build_dir == "dimos/experimental/memory/rust"
    assert _SCRIPT._flake_ref_of(recorder) == "path:."


@pytest.mark.parametrize("override", [None, "build_command", "source_dir"])
def test_inherited_build_coverage_tracks_overrides(override: str | None) -> None:
    owner = _ClassDef(
        "owner.py", "Owner", ("NativeModuleConfig",), "nix build .#owner", "literal", True
    )
    child = _ClassDef(
        "child.py",
        "Child",
        ("Owner",),
        "nix build .#child" if override == "build_command" else None,
        "literal" if override == "build_command" else "absent",
        override == "source_dir",
    )
    grandchild = _ClassDef("grandchild.py", "Grandchild", ("Child",), None, "absent")
    expected = {("owner.py", "Owner")}
    if override is not None:
        expected.add(("child.py", "Child"))
    assert _closure_nix_configs([owner, child, grandchild]) == expected


def test_ast_extraction_matches_runtime() -> None:
    """The AST-read defaults must equal what pydantic resolves at runtime —
    this equivalence is what lets CI discover modules without installing dimos.
    Build directories are relative to the shared project root."""
    modules = _SCRIPT.discover()
    assert modules
    for module in modules:
        dotted, class_name = module.qualname.rsplit(".", 1)
        config_class = getattr(importlib.import_module(dotted), class_name)
        fields = config_class.model_fields
        assert fields["build_command"].default == module.build_command
        source_dir = fields["source_dir"].default
        assert source_dir is not None
        runtime_dir = DIMOS_PROJECT_ROOT / source_dir
        assert runtime_dir == (DIMOS_PROJECT_ROOT / module.build_dir).resolve()


def test_every_module_builds_from_its_own_directory() -> None:
    """`path:.` from the module dir, hashing nothing outside it: a root ref copies all of LFS, an outside input escapes the publish key."""
    modules = _SCRIPT.discover()
    assert modules
    for module in modules:
        assert module.build_dir != ".", f"{module.qualname}: builds from the repository root"
        ref = _SCRIPT._flake_ref_of(module)
        assert ref == "path:." or ref.startswith("path:.#"), (
            f"{module.qualname}: flake ref {ref!r} is not the `path:.#<package>` convention"
        )
        inputs = _SCRIPT._collect_input_paths(module)
        assert inputs == {module.build_dir}, (
            f"{module.qualname}: hashed inputs {sorted(inputs)} are not just "
            f"{module.build_dir!r} — the flake reaches outside its own directory"
        )


_GUARD_EXEMPT = re.compile(r"^(target|build|__pycache__|result.*|.*~|.*\.o|.*\.so|\.sw.)$")


@pytest.mark.skipif(not _IN_GIT_CHECKOUT, reason="needs a git checkout to list untracked files")
def test_module_dirs_hold_only_tracked_source() -> None:
    """`path:.` copies untracked files too, so a stray `.DS_Store` changes the hash and misses Cachix."""
    stray = []
    for flake in _SCRIPT.flake_dirs():
        # ls-files, not status: status runs the LFS clean filter, which CI disables
        listing = subprocess.run(
            ["git", "ls-files", "--others", "--directory", "-z", "--", flake],
            cwd=DIMOS_PROJECT_ROOT,
            capture_output=True,
            text=True,
            check=True,
        ).stdout
        for entry in filter(None, listing.split("\0")):
            path = Path(entry)
            if not any(_GUARD_EXEMPT.match(part) for part in path.relative_to(flake).parts):
                stray.append(path.as_posix())
    assert not stray, f"delete these, or commit them if they are source: {stray}"


def test_a_flake_that_fails_to_evaluate_fails_the_gate(monkeypatch: pytest.MonkeyPatch) -> None:
    def broken_eval(command, **kwargs):
        raise subprocess.CalledProcessError(1, command, "", "error: syntax error")

    monkeypatch.setattr(_SCRIPT.subprocess, "run", broken_eval)
    _SCRIPT.current_system.cache_clear()
    with pytest.raises(RuntimeError, match="syntax error"):
        _SCRIPT._checks_of("native/rust")
    _SCRIPT.current_system.cache_clear()


_IN_REPO_INPUT = re.compile(r'url = "github:dimensionalOS/dimos\?(?P<query>[^"]*)"')


def _module_flakes() -> list[Path]:
    return sorted(
        flake for flake in DIMOS_PROJECT_ROOT.rglob("flake.nix") if ".git" not in flake.parts
    )


def _in_repo_input_refs() -> dict[str, list[str | None]]:
    """flake path -> the `ref=` of each in-repo input it declares (None when absent)."""
    refs: dict[str, list[str | None]] = {}
    for flake in _module_flakes():
        found: list[str | None] = []
        for match in _IN_REPO_INPUT.finditer(flake.read_text()):
            ref = re.search(r"(?:^|&)ref=([^&]*)", match.group("query"))
            found.append(ref.group(1) if ref else None)
        if found:
            refs[flake.relative_to(DIMOS_PROJECT_ROOT).as_posix()] = found
    return refs


def _resolve(nodes: dict, node: str, name: str) -> str | None:
    """The node `name` refers to from `node`, resolving a `follows` path from root."""
    edge = nodes.get(node, {}).get("inputs", {}).get(name)
    if edge is None:
        return None
    if isinstance(edge, str):
        return edge
    current = "root"
    for step in edge:
        following = _resolve(nodes, current, step)
        if following is None:
            return None
        current = following
    return current


def _build_nixpkgs_revs(lock: Path) -> set[str]:
    """Every nixpkgs revision a flake actually builds against.

    Reached by walking only the edges that carry a build: a flake's own `nixpkgs`,
    and the in-repo shared flakes, whose nixpkgs is the one their `buildNativeModule`
    and C++ SDK use. Everything else in the graph is somebody's tooling -- crate2nix
    pulls in cachix, which pins a nixpkgs of its own that no derivation here is built
    from. Counting those would report a divergence that does not exist.
    """
    nodes = json.loads(lock.read_text()).get("nodes", {})
    revs: set[str] = set()
    seen: set[str] = set()
    frontier = ["root"]
    while frontier:
        node = frontier.pop()
        if node in seen:
            continue
        seen.add(node)
        locked = nodes.get(node, {}).get("locked", {})
        if locked.get("repo") == "nixpkgs" and locked.get("owner") in ("NixOS", "nixos"):
            revs.add(locked["rev"])
            continue
        for name in _BUILD_EDGES:
            if (target := _resolve(nodes, node, name)) is not None:
                frontier.append(target)
    return revs


@pytest.mark.skipif(not _IN_GIT_CHECKOUT, reason="needs a git checkout to find the locks")
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
            raw = flake.read_text()
            for match in _RAW_REF.finditer(raw):
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
                if "fileset.toSource" in raw and re.search(r"\broot\s*=\s*$", raw[: match.start()]):
                    continue  # fileset anchor, deliberately not an input (see _flake_refs)
                token = match.group("path").split("?", 1)[0]
                target = os.path.relpath(os.path.normpath(flake.parent / token), DIMOS_PROJECT_ROOT)
                if not (DIMOS_PROJECT_ROOT / target).exists():
                    continue  # comment/string noise; real broken refs fail discovery loudly
                assert any(target == c or target.startswith(c + "/") for c in covered), (
                    f"{flake}: reference {token!r} resolves to {target!r}, outside the hashed "
                    "input set — teach bin/native-modules._FLAKE_REF the new form"
                )


@pytest.mark.skipif(not _IN_GIT_CHECKOUT, reason="needs a git checkout to read refs")
def test_in_repo_inputs_follow_the_default_branch() -> None:
    """A `ref=` on an in-repo input names a branch that is gone once its PR merges.

    The revision is pinned by bin/relock-shared-flakes; the input must not name a branch.
    """
    pinned = {flake: refs for flake, refs in _in_repo_input_refs().items() if any(refs)}
    assert not pinned, f"drop the `ref=` from these in-repo inputs: {pinned}"


@pytest.mark.skipif(not _IN_GIT_CHECKOUT, reason="needs a git checkout to read refs")
def test_module_locks_pin_the_shared_code_as_it_is_now() -> None:
    """Every module's pin on native/cpp or native/rust must hold this commit's tree.

    `nix flake lock` and `cargo build --locked` do not notice that the shared code
    moved -- they only check that the lock is complete -- so a change there can land
    while every module still builds against the revision before it. The revision is
    free to be older than HEAD; what must match is the content it pins.
    """
    stale = {
        _rel(lock): names
        for lock in _RELOCK.flake_locks()
        if (names := _RELOCK.stale_flake_inputs(lock))
    }
    stale |= {
        _rel(manifest): why
        for manifest in _RELOCK.cargo_manifests()
        if (why := _RELOCK.stale_cargo_pins(manifest))
    }
    assert not stale, (
        "these modules pin shared code that is not the code in this commit, so they build "
        f"against the older version: {stale} -- push, run bin/relock-shared-flakes, commit"
    )


def _rel(path: Path) -> str:
    return path.relative_to(DIMOS_PROJECT_ROOT).as_posix()


_BUILD_EDGES = ("nixpkgs", "dimos-native-rust", "dimos-native-cpp")


def _root_nixpkgs(lock: Path) -> str | None:
    """The nixpkgs revision a lock's root actually builds against, following follows."""
    data = json.loads(lock.read_text())
    nodes, root = data["nodes"], data.get("root", "root")
    name = nodes[root].get("inputs", {}).get("nixpkgs")
    seen: set[str] = set()
    while isinstance(name, str) and name not in seen:
        seen.add(name)
        node = nodes.get(name, {})
        if "locked" in node:
            return node["locked"].get("rev")
        name = node.get("inputs", {}).get("nixpkgs")
    return None


def test_every_module_flake_builds_against_one_nixpkgs() -> None:
    """Every module flake pins the same nixpkgs.

    Each flake is standalone and names `nixos-unstable` itself, so nothing makes them
    agree -- they pin whenever they happen to be locked and drift apart silently. The
    cost is not subtle: a different nixpkgs is a different rustc, so two modules on
    two revisions share no dependency derivations at all, and CI rebuilds from scratch
    what it should have substituted.

    The repo-root flake is excluded on purpose: it builds no module, it is the
    devcontainer, and bumping it is an everyone-lives-here change.
    """
    by_rev: dict[str, list[str]] = {}
    for lock in sorted(DIMOS_PROJECT_ROOT.rglob("flake.lock")):
        if ".git" in lock.parts or lock.parent == DIMOS_PROJECT_ROOT:
            continue
        rev = _root_nixpkgs(lock)
        if rev:
            by_rev.setdefault(rev, []).append(
                lock.parent.relative_to(DIMOS_PROJECT_ROOT).as_posix()
            )
    if not by_rev:
        pytest.skip("no flake.lock pins a nixpkgs to compare")
    assert len(by_rev) == 1, (
        "module flakes disagree on nixpkgs, so they share no build: "
        + "; ".join(f"{rev[:10]} -> {mods}" for rev, mods in by_rev.items())
        + " -- run `nix flake update nixpkgs` in each and commit the locks"
    )


def _rust_flake_dirs() -> list[Path]:
    """Flake directories with a cargo manifest, plus the shared crates' own flake.

    `native/rust` has no manifest of its own -- the two crates sit one level down --
    so it is named rather than matched.
    """
    directories = {DIMOS_PROJECT_ROOT / "native" / "rust"}
    for flake in DIMOS_PROJECT_ROOT.rglob("flake.nix"):
        skipped = {".git", "target", "result", "build"}
        if not skipped.isdisjoint(flake.parts):
            continue
        if (flake.parent / "Cargo.toml").exists():
            directories.add(flake.parent)
    return sorted(directories)


def test_every_rust_flake_offers_a_tests_check() -> None:
    """Rust tests are derivations, so a flake without the check is silently untested.

    CI runs `bin/native-modules --test`, which skips a flake declaring no
    `checks.<system>.tests` rather than failing -- that is what lets the C++ flakes
    through. A rust crate that loses the attribute would be skipped just as quietly.
    """
    missing = [
        directory.relative_to(DIMOS_PROJECT_ROOT).as_posix()
        for directory in _rust_flake_dirs()
        if "checks.tests" not in re.sub(r"\s+", "", (directory / "flake.nix").read_text())
    ]
    assert not missing, f"rust flakes with no tests check, so nothing runs their tests: {missing}"


def test_ci_never_names_a_module_directory() -> None:
    """The workflow must discover modules, not list them.

    Every hardcoded module path in ci.yml has been a maintenance bug waiting to
    happen: the list goes stale when a module is added, renamed or moved, and CI
    keeps passing while silently testing less. Discovery belongs in `bin/`, where
    it can be run and tested locally.
    """
    workflow = (DIMOS_PROJECT_ROOT / ".github" / "workflows" / "ci.yml").read_text()
    module_dirs = [
        directory.relative_to(DIMOS_PROJECT_ROOT).as_posix() for directory in _rust_flake_dirs()
    ]
    named = sorted(directory for directory in module_dirs if directory in workflow)
    assert not named, (
        f"ci.yml names module directories: {named} -- discover them in a bin/ script instead"
    )
