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
import subprocess
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
        if _SCRIPT.is_nix_build(command):
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


def test_no_module_hashes_the_repo_root() -> None:
    """A collected input of "." puts the whole-repo tree SHA in the marker key,
    so it changes on every commit and the marker never matches."""
    for module in _SCRIPT.discover():
        assert "." not in _SCRIPT._collect_input_paths(module), (
            f"{module.qualname}: input set includes the repo root — a fileset root anchor "
            "is being hashed, which busts the publish marker on every commit"
        )


def test_every_module_flake_is_self_contained() -> None:
    """A module flake may reach its own directory and nothing else.

    This is what makes the publish gate's key correct: `--inputs-hash` hashes each
    module's directory tree and nothing more, so a path literal or `path:` input
    pointing outside it would let content change without changing the key, and the
    module would be served from Cachix as already published. Anything shared has to
    arrive as a remote input pinned in the module's own flake.lock, or be copied in.
    """
    modules = _SCRIPT.discover()
    assert modules
    for module in modules:
        inputs = _SCRIPT._collect_input_paths(module)
        assert inputs == {module.build_dir}, (
            f"{module.qualname}: hashed inputs {sorted(inputs)} are not just "
            f"{module.build_dir!r} — the flake reaches outside its own directory"
        )


def test_no_module_reaches_the_repository_root() -> None:
    """No `nix build` may resolve to the repo root.

    A ref that walks up to `.git` copies the entire working tree into the store,
    including every smudged git-lfs blob under `data/` — 16 GB per build, and a
    source hash that differs between machines depending on what they have pulled.
    """
    for module in _SCRIPT.discover():
        assert module.build_dir != ".", f"{module.qualname}: builds from the repository root"
        ref = _SCRIPT._flake_ref_of(module)
        assert ref == "path:." or ref.startswith(("path:.#", "github:")), (
            f"{module.qualname}: flake ref {ref!r} is not the `path:.#<package>` convention"
        )


_SHARED_FLAKES = ("native/rust/flake.nix", "native/cpp/flake.nix")
_DEFAULT_BRANCH = "main"

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


def _current_branch_names() -> set[str]:
    """Every name this checkout answers to.

    A pull request is checked out detached, where `--abbrev-ref HEAD` is the literal
    string "HEAD" and names nothing; the branch is then only in the environment the
    runner sets. Both are consulted so the same test means the same thing on a
    developer's machine and on a runner.
    """
    done = subprocess.run(
        ("git", "-C", str(DIMOS_PROJECT_ROOT), "rev-parse", "--abbrev-ref", "HEAD"),
        capture_output=True,
        text=True,
    )
    names = {done.stdout.strip()} - {"HEAD", ""}
    names |= {
        os.environ[key] for key in ("GITHUB_HEAD_REF", "GITHUB_REF_NAME") if os.environ.get(key)
    }
    return names


def _default_branch_has_shared_flakes() -> bool:
    """True once the shared flakes exist on the default branch we can see locally."""
    for ref in ("origin/HEAD", f"origin/{_DEFAULT_BRANCH}", _DEFAULT_BRANCH):
        for flake in _SHARED_FLAKES:
            done = subprocess.run(
                ("git", "-C", str(DIMOS_PROJECT_ROOT), "cat-file", "-e", f"{ref}:{flake}"),
                capture_output=True,
            )
            if done.returncode == 0:
                return True
    return False


@pytest.mark.skipif(not _IN_GIT_CHECKOUT, reason="needs a git checkout to read refs")
def _locked_in_repo_inputs() -> dict[str, list[tuple[str, str]]]:
    """flake.lock path -> the (rev, dir) of each in-repo input it pins."""
    locked: dict[str, list[tuple[str, str]]] = {}
    for lock in DIMOS_PROJECT_ROOT.rglob("flake.lock"):
        if ".git" in lock.parts:
            continue
        nodes = json.loads(lock.read_text()).get("nodes", {})
        pins = [
            (node["locked"]["rev"], node["locked"]["dir"])
            for node in nodes.values()
            if node.get("locked", {}).get("repo") == "dimos"
            and node["locked"].get("owner") == "dimensionalOS"
            and "dir" in node["locked"]
        ]
        if pins:
            locked[lock.relative_to(DIMOS_PROJECT_ROOT).as_posix()] = pins
    return locked


def _tree_at(rev: str, path: str) -> str | None:
    """The git tree hash of `path` at `rev`, or None if the revision cannot be had."""

    def read() -> str | None:
        done = subprocess.run(
            ("git", "-C", str(DIMOS_PROJECT_ROOT), "rev-parse", f"{rev}:{path}"),
            capture_output=True,
            text=True,
        )
        return done.stdout.strip() if done.returncode == 0 else None

    if (tree := read()) is not None:
        return tree
    subprocess.run(
        ("git", "-C", str(DIMOS_PROJECT_ROOT), "fetch", "--depth=1", "origin", rev),
        capture_output=True,
        timeout=120,
    )
    return read()


@pytest.mark.skipif(not _IN_GIT_CHECKOUT, reason="needs a git checkout to read refs")
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
                    "input set — teach bin/build-native-modules._FLAKE_REF the new form"
                )


def test_module_locks_pin_the_shared_flakes_as_they_are_now() -> None:
    """A module's lock must name a revision whose shared tree is the one in this commit.

    `nix flake lock` does not notice that the shared flake moved -- it only checks
    that the lock is complete -- so a change to native/rust can land while every
    module still builds against the revision before it, and nothing says so. The
    revision itself is free to be older than HEAD; what must match is the content it
    pins.

    Shallow clones cannot answer the question at all, so this skips rather than
    guesses when the pinned revision is not in the checkout.
    """
    stale: dict[str, list[str]] = {}
    checked = 0
    for lock, pins in _locked_in_repo_inputs().items():
        for rev, subdir in pins:
            pinned = _tree_at(rev, subdir)
            if pinned is None:
                continue  # shallow clone: the revision is not here to compare
            checked += 1
            here = _tree_at("HEAD", subdir)
            if pinned != here:
                stale.setdefault(lock, []).append(f"{subdir} @ {rev[:10]}")
    if not checked:
        pytest.skip("no pinned revision is present in this checkout to compare against")
    assert not stale, (
        "these locks pin a revision whose shared tree is not the one in this commit, "
        f"so the modules build against the older shared flake: {stale} -- run "
        "`nix flake update <input>` in each and commit the lock"
    )


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

    CI runs `bin/build-native-modules --tests`, which skips a flake declaring no
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
