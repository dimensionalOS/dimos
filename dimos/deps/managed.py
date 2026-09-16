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

"""Managed runtime environments: prepared once, immutable, reused by identity.

An environment lives under ``DATA_DIR/envs/<profile>-py<X.Y>-<digest>`` where the
digest covers the profile, the launcher's Python, the extras, the source
(checkout path or release version) and the lock: the checkout's ``uv.lock``
and ``pyproject.toml``, or the tested lock a release ships. Anything that
would change the installed packages therefore yields a new directory;
nothing is ever synchronized into an environment a run may be using.

A checkout environment is ``uv sync --frozen`` of the repository lock for the
selected extras; an installation environment is a generated uv project that
depends on ``dimos[extras]==<version>`` under the repository's resolver
policy, constrained to the release's tested versions. Both end with a probe,
and the stamp file is written only after it passes, so a directory without a
stamp is an unfinished attempt.

The stamp says which packages are installed; it does not say which runs the
environment was validated for. ``ensure_environment`` records one validation
per run shape (blueprints, dependency-affecting configuration, prerequisites,
profile and host) under ``<env>/validations/`` and probes again whenever a
request has a shape it has not seen.

Every environment has two leases in ``DATA_DIR/envs/.locks`` (see
:mod:`dimos.deps.lease`). ``ensure_environment`` holds the prepare mutex while
it reads the stamp, installs and validates, then hands the caller a shared use
lease that lasts for the run. Removal claims both exclusively without waiting,
so it never touches an environment that is being prepared or used.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator, Mapping, Sequence
import contextlib
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from typing import Any, Literal

from dimos.constants import DATA_DIR, DIMOS_PROJECT_ROOT
from dimos.deps.catalog import Plan
from dimos.deps.environment import ALL_CHECKS, EnvironmentReport, format_report, is_checkout
from dimos.deps.lease import EnvironmentBusyError, EnvironmentLease, is_held, prepare_lock, use_lock
from dimos.deps.policy import (
    SHIPPED_CONSTRAINTS_FILE,
    find_project_file,
    load_constraints,
    load_uv_policy,
    packages_outside,
    render_wheel_project,
)
from dimos.deps.probe import REQUEST_SCHEMA, ProbeRequest, run_probe
from dimos.deps.profiles import HostFacts, Profile
from dimos.deps.rules import DEFAULTS
from dimos.deps.uv import UvRunner, uv_environment, uv_version

ENVS_DIR = DATA_DIR / "envs"
VENV_DIR = ".venv"
PROJECT_DIR = "project"
STAMP_NAME = "dimos-env.json"
STAMP_SCHEMA = 2
VALIDATIONS_DIR = "validations"
VALIDATION_SCHEMA = 1
PACKAGE_CHECKS = ("packages", "providers")
LOCK_FILES = ("uv.lock", "pyproject.toml")
ENVIRONMENT_NAME = re.compile(r"[a-z0-9_-]+-py\d+\.\d+-[0-9a-f]{8}")
"""What ``EnvironmentKey.name`` produces; nothing else in the store is an environment."""
IN_USE = "in use by another dimos process"
PREPARING = "being prepared or removed by another dimos process"
Echo = Callable[[str], None]


class PreparationError(RuntimeError):
    """Preparation failed; the message is ready for the user."""


@dataclass(frozen=True)
class Source:
    mode: Literal["checkout", "wheel"]
    root: Path | None
    version: str
    find_links: str | None = None

    @property
    def identity(self) -> str:
        if self.mode == "checkout":
            return f"checkout:{self.root}"
        return f"wheel:{self.version}"

    def to_json(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "root": str(self.root) if self.root else None,
            "version": self.version,
            "find_links": self.find_links,
        }


def detect_source(find_links: str | None = None, project_root: Path = DIMOS_PROJECT_ROOT) -> Source:
    try:
        version = importlib.metadata.version("dimos")
    except importlib.metadata.PackageNotFoundError:
        version = "unknown"
    if is_checkout(project_root):
        return Source("checkout", project_root.resolve(), version, find_links)
    return Source("wheel", None, version, find_links)


def checkout_lock_hash(root: Path) -> str:
    digest = hashlib.sha256()
    for name in LOCK_FILES:
        digest.update((root / name).read_bytes())
        digest.update(b"\0")
    return f"sha256:{digest.hexdigest()}"


def envs_dir() -> Path:
    return ENVS_DIR


@dataclass(frozen=True)
class EnvironmentKey:
    profile: str
    python: str
    extras: tuple[str, ...]
    source: Source
    lock_hash: str | None

    @property
    def digest(self) -> str:
        material = "\n".join(
            [
                self.profile,
                self.python,
                ",".join(sorted(self.extras)),
                self.source.identity,
                self.lock_hash or "",
            ]
        )
        return hashlib.sha256(material.encode()).hexdigest()[:8]

    @property
    def name(self) -> str:
        return f"{self.profile}-py{self.python}-{self.digest}"

    @property
    def path(self) -> Path:
        return envs_dir() / self.name

    @property
    def venv(self) -> Path:
        return self.path / VENV_DIR

    @property
    def python_executable(self) -> Path:
        return self.venv / "bin" / "python"

    @property
    def dimos_executable(self) -> Path:
        return self.venv / "bin" / "dimos"

    @property
    def project(self) -> Path:
        return self.path / PROJECT_DIR

    @property
    def stamp_path(self) -> Path:
        return self.path / STAMP_NAME


def environment_key(plan: Plan, profile: Profile, source: Source) -> EnvironmentKey:
    python = f"{sys.version_info.major}.{sys.version_info.minor}"
    if source.mode == "checkout" and source.root:
        lock_hash: str | None = checkout_lock_hash(source.root)
    else:
        lock_hash = _file_hash(SHIPPED_CONSTRAINTS_FILE)
    return EnvironmentKey(profile.name, python, tuple(sorted(plan.extras)), source, lock_hash)


def _save_json(path: Path, data: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    staging = path.with_suffix(".tmp")
    staging.write_text(json.dumps(data, indent=2) + "\n")
    os.replace(staging, path)


def _load_json(path: Path, schema: int) -> dict[str, Any] | None:
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    if not isinstance(data, dict) or data.get("schema") != schema:
        return None
    return data


@dataclass
class Stamp:
    """What is installed: written once the packages are in place."""

    name: str
    profile: str
    python: str
    python_executable: str
    extras: list[str]
    backends: dict[str, str]
    source: dict[str, Any]
    lock_hash: str | None
    uv_version: str
    created_at: str
    fixups: list[str] = field(default_factory=list)
    schema: int = STAMP_SCHEMA

    def save(self, path: Path) -> None:
        _save_json(path, asdict(self))

    @classmethod
    def load(cls, path: Path) -> Stamp | None:
        data = _load_json(path, STAMP_SCHEMA)
        if data is None:
            return None
        try:
            return cls(**data)
        except TypeError:
            return None


@dataclass
class Validation:
    """One run shape the environment was probed for, and what the probe found."""

    digest: str
    material: dict[str, Any]
    created_at: str
    report: dict[str, Any]
    schema: int = VALIDATION_SCHEMA

    def save(self, path: Path) -> None:
        _save_json(path, asdict(self))

    @classmethod
    def load(cls, path: Path) -> Validation | None:
        data = _load_json(path, VALIDATION_SCHEMA)
        if data is None:
            return None
        try:
            return cls(**data)
        except TypeError:
            return None


def validation_material(
    plan: Plan,
    profile: Profile,
    *,
    overridden: bool,
    blueprints: Sequence[str],
    global_config: Mapping[str, Any],
    facts: HostFacts,
) -> dict[str, Any]:
    """Everything a validation's result depends on, besides the installed packages."""
    return {
        "blueprints": sorted(blueprints),
        "config": {key: global_config.get(key, default) for key, default in DEFAULTS.items()},
        "native": sorted(plan.native),
        "system": sorted(plan.system),
        "tools": sorted(plan.tools),
        "backends": dict(sorted(plan.backends.items())),
        "profile": profile.name,
        "overridden": overridden,
        "probe_schema": REQUEST_SCHEMA,
        "host": asdict(facts),
    }


def is_running_inside(key: EnvironmentKey) -> bool:
    return Path(sys.prefix).resolve() == key.venv.resolve()


def managed_environment_of(prefix: Path) -> Path | None:
    """The environment whose virtualenv is ``prefix``, or ``None`` outside the store."""
    venv = prefix.resolve()
    if venv.name != VENV_DIR or venv.parent.parent != envs_dir().resolve():
        return None
    return venv.parent


def _echo(message: str) -> None:
    print(message, file=sys.stderr)


def use_lease(env_path: Path, *, echo: Echo = _echo) -> EnvironmentLease:
    """The shared lease every process running from ``env_path`` holds for its lifetime."""
    return EnvironmentLease.acquire(
        use_lock(env_path),
        shared=True,
        blocking=True,
        busy=f"{env_path.name} is {PREPARING}",
        echo=echo,
    )


def ensure_environment(
    key: EnvironmentKey,
    plan: Plan,
    profile: Profile,
    *,
    overridden_profile: bool,
    blueprints: Sequence[str],
    global_config: Mapping[str, Any],
    offline: bool,
    prepare_missing: bool = True,
    uv: UvRunner | None = None,
    probe: Callable[..., EnvironmentReport] = run_probe,
    echo: Echo = _echo,
    facts: HostFacts | None = None,
) -> tuple[Stamp, EnvironmentLease]:
    """The environment for ``key``, validated for this run, and a shared lease on it.

    Prepares the environment when it is absent. ``offline`` makes uv install
    from its cache only; ``prepare_missing=False`` refuses to prepare at all,
    which is what an offline run wants.
    """
    mutex = EnvironmentLease.acquire(
        prepare_lock(key.path),
        shared=False,
        blocking=True,
        busy=f"{key.name} is {PREPARING}",
        echo=echo,
    )
    with mutex:
        stamp = Stamp.load(key.stamp_path)
        if stamp is not None:
            echo(f"Reusing prepared environment {key.name} at {key.path}")
        elif not prepare_missing:
            raise PreparationError(
                f"environment {key.name} is not prepared and --offline forbids preparing it; "
                f"run `dimos prepare {' '.join(blueprints)}` while online"
            )
        else:
            stamp = prepare_environment(
                key, plan, profile, offline=offline, uv=uv, probe=probe, echo=echo
            )
        if "deno" in plan.tools and not offline:
            from dimos.utils.deno import ensure_deno  # downloads on demand; not an import-time need

            ensure_deno()
        validate_run(
            key,
            plan,
            profile,
            overridden_profile=overridden_profile,
            blueprints=blueprints,
            global_config=global_config,
            probe=probe,
            echo=echo,
            facts=facts,
        )
        lease = use_lease(key.path, echo=echo)
    return stamp, lease


def validate_run(
    key: EnvironmentKey,
    plan: Plan,
    profile: Profile,
    *,
    overridden_profile: bool,
    blueprints: Sequence[str],
    global_config: Mapping[str, Any],
    probe: Callable[..., EnvironmentReport] = run_probe,
    echo: Echo = _echo,
    facts: HostFacts | None = None,
) -> Validation:
    """The validation record for this run shape, probing when there is none yet.

    Runs under the prepare mutex. A failed probe leaves the stamp and writes
    no record, so the next request for the same shape probes again.
    """
    material = validation_material(
        plan,
        profile,
        overridden=overridden_profile,
        blueprints=blueprints,
        global_config=global_config,
        facts=facts or HostFacts.detect(),
    )
    digest = hashlib.sha256(json.dumps(material, sort_keys=True, default=str).encode()).hexdigest()
    path = key.path / VALIDATIONS_DIR / f"{digest[:12]}.json"
    record = Validation.load(path)
    if record is not None:
        return record
    echo(f"Validating {key.name} for {', '.join(blueprints) or 'core'}")
    report = probe(
        key.python_executable,
        ProbeRequest.from_plan(
            plan, ALL_CHECKS, blueprints=tuple(blueprints), global_config=global_config
        ),
    )
    _require_ok(report, plan, key, overridden_profile=overridden_profile, echo=echo)
    record = Validation(
        digest[:12], material, datetime.now(timezone.utc).isoformat(), report.to_json()
    )
    record.save(path)
    return record


def prepare_environment(
    key: EnvironmentKey,
    plan: Plan,
    profile: Profile,
    *,
    offline: bool,
    uv: UvRunner | None = None,
    probe: Callable[..., EnvironmentReport] = run_probe,
    echo: Echo = _echo,
) -> Stamp:
    """Install the packages, confirm them with the probe and stamp; the caller holds the mutex."""
    uv = uv or UvRunner()
    try:
        exclusive = EnvironmentLease.acquire(
            use_lock(key.path),
            shared=False,
            blocking=False,
            busy=f"{key.name} is incomplete but {IN_USE}",
        )
    except EnvironmentBusyError as error:
        raise PreparationError(str(error)) from error
    with exclusive:
        if key.path.exists():
            shutil.rmtree(key.path)
        key.path.mkdir(parents=True)
        echo(f"Preparing environment {key.name} at {key.path}")
        _sync(key, uv, offline=offline)
        fixups = _post_sync_fixups(key, profile, uv, probe, offline=offline)
        report = probe(key.python_executable, ProbeRequest.from_plan(plan, PACKAGE_CHECKS))
        if not report.satisfied_for_launch:
            raise PreparationError(
                f"{key.name} lacks packages after installation:\n"
                f"{format_report(report, plan, checkout=False)}"
            )
        stamp = Stamp(
            name=key.name,
            profile=profile.name,
            python=key.python,
            python_executable=str(key.python_executable),
            extras=list(key.extras),
            backends=dict(plan.backends),
            source=key.source.to_json(),
            lock_hash=key.lock_hash,
            uv_version=str(uv_version(uv.command)),
            created_at=datetime.now(timezone.utc).isoformat(),
            fixups=fixups,
        )
        stamp.save(key.stamp_path)
        echo(f"Prepared {key.name} at {key.path}")
        return stamp


def _sync(key: EnvironmentKey, uv: UvRunner, *, offline: bool) -> None:
    source = key.source
    env = uv_environment(key.venv, offline=offline, find_links=source.find_links)
    common = ["--python", sys.executable] + (["--offline"] if offline else [])
    if source.mode == "checkout" and source.root is not None:
        args = ["sync", "--frozen", "--no-default-groups", "--project", str(source.root), *common]
        for extra in key.extras:
            args += ["--extra", extra]
        if uv.run(args, env=env, cwd=source.root) != 0:
            raise PreparationError(
                f"uv sync failed for {key.name}. If the error is a compiler error: building "
                "dimos' native extension needs a C++ toolchain (apt install build-essential "
                "or xcode-select --install)."
            )
        return
    project_file = find_project_file(DIMOS_PROJECT_ROOT)
    if project_file is None:
        raise PreparationError(
            "the installed dimos ships no resolver policy (pyproject.toml copy); "
            "managed environments need a wheel built with it"
        )
    if not SHIPPED_CONSTRAINTS_FILE.is_file():
        raise PreparationError(
            "the installed dimos ships no tested lock (constraints.txt); "
            "managed environments need a wheel built with it"
        )
    constraints = load_constraints(SHIPPED_CONSTRAINTS_FILE)
    key.project.mkdir(parents=True, exist_ok=True)
    (key.project / "pyproject.toml").write_text(
        render_wheel_project(
            source.version,
            key.extras,
            key.python,
            _profile_for(key),
            load_uv_policy(project_file),
            constraints,
        )
    )
    if uv.run(["lock", "--project", str(key.project), *common], env=env, cwd=key.project) != 0:
        raise PreparationError(
            f"uv lock failed for {key.name}: the versions dimos {source.version} was tested "
            f"with could not be resolved here. If dimos {source.version} is not published, pass "
            "`dimos prepare --find-links <wheelhouse>` with a locally built wheel; otherwise "
            "upgrade dimos or run with --environment current."
        )
    outside = packages_outside(key.project / "uv.lock", constraints)
    if outside:
        raise PreparationError(
            f"{key.name} resolved packages outside the tested set: {', '.join(outside)}"
        )
    if (
        uv.run(
            ["sync", "--frozen", "--project", str(key.project), *common], env=env, cwd=key.project
        )
        != 0
    ):
        raise PreparationError(f"uv sync failed for {key.name}")


def _profile_for(key: EnvironmentKey) -> Profile:
    from dimos.deps.profiles import PROFILES

    return PROFILES[key.profile]


def _post_sync_fixups(
    key: EnvironmentKey,
    profile: Profile,
    uv: UvRunner,
    probe: Callable[..., EnvironmentReport],
    *,
    offline: bool,
) -> list[str]:
    """Make the GPU onnxruntime win when chromadb dragged the CPU one in beside it."""
    if profile.accelerator != "cuda":
        return []
    report = probe(key.python_executable, ProbeRequest(extras=key.extras, checks=("providers",)))
    if ("onnxruntime", "onnxruntime-gpu") not in report.conflicts:
        return []
    version = _installed_version(key.python_executable, "onnxruntime-gpu")
    args = [
        "pip",
        "install",
        "--python",
        str(key.python_executable),
        "--no-deps",
        "--reinstall",
        f"onnxruntime-gpu=={version}" if version else "onnxruntime-gpu",
    ]
    if offline:
        args.append("--offline")
    if uv.run(args, env=uv_environment(key.venv, offline=offline)) != 0:
        raise PreparationError(f"could not re-layer onnxruntime-gpu in {key.name}")
    return ["onnxruntime-gpu-relayer"]


def _installed_version(python: Path, distribution: str) -> str | None:
    script = f"import importlib.metadata as m; print(m.version({distribution!r}))"
    completed = subprocess.run(
        [str(python), "-c", script], capture_output=True, text=True, check=False
    )
    return completed.stdout.strip() or None


def _require_ok(
    report: EnvironmentReport,
    plan: Plan,
    key: EnvironmentKey,
    *,
    overridden_profile: bool,
    echo: Echo,
) -> None:
    if report.ok:
        return
    text = format_report(report, plan, checkout=False)
    backends_only = report.satisfied_for_launch and all(
        kind == "backends" for kind, _name, _outcome in report.missing_prerequisites
    )
    if backends_only and overridden_profile:
        echo(f"Warning: backend probes failed for the explicitly requested profile:\n{text}")
        return
    raise PreparationError(f"{key.name} does not satisfy this run:\n{text}")


def _file_hash(path: Path) -> str | None:
    if not path.is_file():
        return None
    return f"sha256:{hashlib.sha256(path.read_bytes()).hexdigest()}"


def list_environments() -> list[tuple[Path, Stamp | None]]:
    if not envs_dir().is_dir():
        return []
    found = []
    for path in sorted(envs_dir().iterdir()):
        if path.is_dir() and ENVIRONMENT_NAME.fullmatch(path.name):
            found.append((path, Stamp.load(path / STAMP_NAME)))
    return found


def environment_state(path: Path) -> str:
    """``preparing``, ``in use`` or ``idle``, as of this instant."""
    if is_held(prepare_lock(path)):
        return "preparing"
    if is_held(use_lock(path)):
        return "in use"
    return "idle"


def managed_path(name: str) -> Path:
    """The directory of the environment called ``name``; only store children qualify."""
    if not ENVIRONMENT_NAME.fullmatch(name):
        raise ValueError(f"{name!r} is not a managed environment name")
    path = envs_dir() / name
    if not path.is_dir():
        raise FileNotFoundError(f"no managed environment named {name!r} in {envs_dir()}")
    return path


@contextlib.contextmanager
def claim(path: Path) -> Iterator[None]:
    """Hold both leases of ``path`` exclusively, failing at once when either is taken."""
    with EnvironmentLease.acquire(
        prepare_lock(path), shared=False, blocking=False, busy=f"{path.name} is {PREPARING}"
    ):
        with EnvironmentLease.acquire(
            use_lock(path), shared=False, blocking=False, busy=f"{path.name} is {IN_USE}"
        ):
            yield


def remove_environment(name: str) -> Path:
    """Delete one environment; refused while it is being prepared or used."""
    path = managed_path(name)
    with claim(path):
        shutil.rmtree(path)
    return path


def prune_environments(*, all_unused: bool = False) -> tuple[list[Path], list[str]]:
    """Remove unfinished directories and environments that no longer apply.

    Without ``all_unused`` an environment is kept while its checkout and lock
    still match the current sources; with it, every idle environment goes.
    Returns the removed paths and why each busy candidate was skipped.
    """
    removed: list[Path] = []
    skipped: list[str] = []
    current_hash = (
        checkout_lock_hash(DIMOS_PROJECT_ROOT) if is_checkout(DIMOS_PROJECT_ROOT) else None
    )
    for path, stamp in list_environments():
        if not _is_stale(stamp, all_unused, current_hash):
            continue
        try:
            with claim(path):
                # A preparation may have completed between the listing and the claim.
                if _is_stale(Stamp.load(path / STAMP_NAME), all_unused, current_hash):
                    shutil.rmtree(path)
                    removed.append(path)
        except EnvironmentBusyError as error:
            skipped.append(str(error))
    return removed, skipped


def _is_stale(stamp: Stamp | None, all_unused: bool, current_hash: str | None) -> bool:
    if stamp is None or all_unused:
        return True
    if stamp.source.get("mode") != "checkout":
        return False
    root = Path(str(stamp.source.get("root")))
    return not root.is_dir() or (
        root.resolve() == DIMOS_PROJECT_ROOT.resolve() and stamp.lock_hash != current_hash
    )
