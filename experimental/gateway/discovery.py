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

from __future__ import annotations

import asyncio
import calendar
from collections import deque
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
import hashlib
import itertools
import json
import os
from pathlib import Path
import re
import shutil
import sys
import time
from typing import Any

from packaging.requirements import InvalidRequirement, Requirement
from packaging.utils import canonicalize_name
import yaml

from dimos.utils.logging_config import setup_logger
from experimental.gateway import robots, store
from experimental.gateway.introspect import MARKER, child_command, kill_group

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

logger = setup_logger()

MAX_ERRORS = 50
ITEM_TIMEOUT_S = 120.0
JOB_KEEP_S = 30 * 60
ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")
IMPORT_ALIASES: dict[str, tuple[str, ...]] = {
    "cv2": ("opencv-python", "opencv-python-headless", "opencv-contrib-python"),
    "PIL": ("pillow",),
    "yaml": ("pyyaml",),
    "sklearn": ("scikit-learn",),
    "skimage": ("scikit-image",),
    "pydrake": ("drake",),
    "pxr": ("usd-core",),
    "can": ("python-can",),
    "open_clip": ("open-clip-torch",),
    "serial": ("pyserial",),
    "usb": ("pyusb",),
    "OpenGL": ("pyopengl",),
}


SCANNED = (
    "ref",
    "robot",
    "importable",
    "import_error",
    "missing_module",
    "optional_dependency",
    "suggested_extras",
    "modules",
)


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def fingerprint(dimos_dir: Path) -> str:
    """What a scan's results depend on: the checkout (commit, and each changed file's size and time) and the venv."""
    parts = [str(dimos_dir.resolve()), str(store.checkout_version(dimos_dir))]
    parts.append(store.git(dimos_dir, "rev-parse", "HEAD") or "")
    for line in (
        store.git(dimos_dir, "status", "--porcelain", "--untracked-files=all") or ""
    ).splitlines():
        path = dimos_dir / line[3:].split(" -> ")[-1].strip('"')
        try:
            found = path.stat()
            parts.append(f"{line} {found.st_size} {found.st_mtime_ns}")
        except OSError:
            parts.append(line)
    for packages in sorted((store.venv_dir(dimos_dir) / "lib").glob("python*/site-packages")):
        parts.append(f"{packages} {packages.stat().st_mtime_ns}")
    return hashlib.sha256("\n".join(parts).encode()).hexdigest()


def saved_scan_file(dimos_dir: Path) -> Path:
    return (
        store.gateway_dir()
        / "scans"
        / f"{hashlib.sha256(str(dimos_dir.resolve()).encode()).hexdigest()[:16]}.json"
    )


FINGERPRINT_TTL_S = 2.0


class Scanner:
    """The blueprint scan's cache: it changes only when a scan (POST /catalog/scan) finishes, and survives restarts."""

    def __init__(self, dimos_dir: Path, jobs: Jobs, command: list[str] | None = None) -> None:
        self.dimos_dir = dimos_dir
        self.jobs = jobs
        self.command = command or child_command(dimos_dir)
        # what the running scan found so far; the cache is replaced with it when the scan finishes
        self.blueprints: dict[str, dict[str, Any]] = {}
        self.modules: dict[str, dict[str, Any]] = {}
        self.errors: list[str] = []
        self.names: dict[str, list[str]] = {"blueprints": [], "modules": []}
        self.finished_modules: set[str] = set()
        self.task: asyncio.Task[None] | None = None
        self.job: Job | None = None
        self.state = "idle"
        self.current: str | None = None
        self.saved: dict[str, Any] | None = None
        self.looked = False
        self.now_print: tuple[float, str] | None = None

    def cache(self) -> dict[str, Any] | None:
        """The last finished scan (from disk after a restart), or None if this checkout was never scanned."""
        if not self.looked:
            self.looked = True
            saved = store.read_json(saved_scan_file(self.dimos_dir))
            if isinstance(saved, dict) and {"results", "modules", "finished_at"} <= saved.keys():
                self.saved = saved
        return self.saved

    def current_fingerprint(self) -> str:
        if self.now_print is None or time.monotonic() - self.now_print[0] > FINGERPRINT_TTL_S:
            self.now_print = (time.monotonic(), fingerprint(self.dimos_dir))
        return self.now_print[1]

    def snapshot(self) -> dict[str, Any]:
        """How stale the cache is: when its scan finished, how long ago, and whether the checkout or venv changed since."""
        cache = self.cache()
        finished = cache["finished_at"] if cache else None
        age = None
        if finished:
            age = max(
                0, int(time.time() - calendar.timegm(time.strptime(finished, "%Y-%m-%dT%H:%M:%SZ")))
            )
        return {
            "job": self.job and self.job.id,
            "state": self.state,
            "scanned": cache is not None,
            "finished_at": finished,
            "age_seconds": age,
            "stale": cache is None or cache.get("fingerprint") != self.current_fingerprint(),
        }

    def start(self, reason: str = "requested") -> Job:
        """The running scan's job, or a new scan's."""
        if self.task is None or self.task.done():
            self.state = "scanning"
            self.job = self.jobs.add(f"Scan blueprints ({reason})", "scan", [*self.command, "scan"])
            self.task = asyncio.get_running_loop().create_task(self.scan(self.job))
        assert self.job is not None
        return self.job

    def pending(self) -> tuple[list[str], list[str]]:
        return (
            [n for n in self.names["blueprints"] if n not in self.blueprints],
            [n for n in self.names["modules"] if n not in self.finished_modules],
        )

    def error(self, text: str) -> None:
        self.errors.append(text)
        if self.job is not None:
            self.job.lines.append(text)

    def tick(self) -> None:
        if self.job is not None:
            total = len(self.names["blueprints"]) or len(self.blueprints)
            self.job.progress = {
                "done": len(self.blueprints),
                "total": total,
                "current": self.current,
            }

    async def scan(self, job: Job) -> None:
        self.blueprints, self.modules, self.errors = {}, {}, []
        self.names, self.finished_modules = {"blueprints": [], "modules": []}, set()
        self.current = None
        # taken before the imports, so a change made while scanning still reads as stale
        before = await asyncio.to_thread(fingerprint, self.dimos_dir)
        ok = False
        try:
            problem = await self.child(None)
            while problem is not None and any(self.pending()):
                current = self.current or ""
                self.error(f"{current}: {problem}")
                if current in self.pending()[0]:
                    self.blueprints[current] = {
                        "name": current,
                        "ref": None,
                        "builtin": None,
                        "importable": False,
                        "optional_dependency": False,
                        "import_error": problem,
                        "missing_module": None,
                        "modules": [],
                    }
                elif current and current.startswith("module:"):
                    self.finished_modules.add(current[7:])
                else:
                    break
                blueprints, modules = self.pending()
                request = {
                    "blueprints": blueprints,
                    "modules": modules,
                    "known": list(self.modules),
                }
                problem = await self.child(request)
            self.current = None
            self.tick()
            results = await asyncio.to_thread(self.finish)
            self.saved = {
                "fingerprint": before,
                "finished_at": now_iso(),
                "results": results,
                "modules": self.modules,
                "errors": self.errors,
                "names": self.names,
            }
            self.looked = True
            # saved before the job says done, so whoever waited for it finds the scan on disk
            await asyncio.to_thread(self.save)
            ok = True
        except Exception as error:
            logger.exception("the blueprint scan failed")
            self.error(f"scan: {type(error).__name__}: {error}")
        finally:
            self.state = "idle" if ok else "failed"
            job.finish(ok, None if ok else (self.errors or ["the scan was cancelled"])[-1])
            if ok and self.saved:
                job.finished_at = self.saved["finished_at"]

    def save(self) -> None:
        try:
            store.write_atomic(saved_scan_file(self.dimos_dir), json.dumps(self.saved))
        except OSError as error:
            logger.warning(f"couldn't save the blueprint scan: {error}")

    async def child(self, request: dict[str, Any] | None) -> str | None:
        child = await asyncio.create_subprocess_exec(
            *self.command,
            "scan",
            cwd=self.dimos_dir,
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            start_new_session=True,
            limit=64 * 1024 * 1024,
        )
        assert child.stdin and child.stdout and child.stderr
        child.stdin.write(json.dumps(request).encode())
        child.stdin.close()
        tail: deque[str] = deque(maxlen=20)

        async def drain() -> None:
            assert child.stderr
            async for line in child.stderr:
                tail.append(line.decode("utf-8", "replace").rstrip())

        draining = asyncio.create_task(drain())
        try:
            while True:
                try:
                    line = await asyncio.wait_for(child.stdout.readline(), ITEM_TIMEOUT_S)
                except asyncio.TimeoutError:
                    return f"no answer in {ITEM_TIMEOUT_S:.0f} s (hung while importing)"
                if not line:
                    await child.wait()
                    last = next((t for t in reversed(tail) if t.strip()), "no output")
                    return f"the import crashed its process (exit {child.returncode}): {last}"
                text = line.decode("utf-8", "replace")
                if MARKER in text and self.take(json.loads(text.rpartition(MARKER)[2])):
                    return None
        finally:
            if child.returncode is None:
                kill_group(child.pid)
                await child.wait()
            draining.cancel()

    def take(self, message: dict[str, Any]) -> bool:
        kind = message.pop("kind", None)
        if kind == "names":
            self.names = {"blueprints": message["blueprints"], "modules": message["modules"]}
        elif kind == "start":
            self.current = message["name"]
        elif kind == "blueprint":
            self.blueprints[message["name"]] = message
        elif kind == "module":
            self.modules[message["class"]] = message
            self.finished_modules.add(message["name"])
        elif kind == "error" or "error" in message:
            self.error(str(message.get("error")))
            current = str(self.current or "")
            if current.startswith("module:"):
                self.finished_modules.add(current[7:])
        self.tick()
        return kind == "end" or (kind is None and "error" in message)

    def finish(self) -> dict[str, dict[str, Any]]:
        """The scan's records by name, in its order, with each one's robot and the extras that would fix it."""
        doc, providers = robots.load(), providing_extras(self.dimos_dir)
        order = {name: index for index, name in enumerate(self.names["blueprints"])}
        found = sorted(self.blueprints.values(), key=lambda r: order.get(r["name"], len(order)))
        return {
            r["name"]: {
                **r,
                "robot": robots.robot_of(doc, r["name"], r["ref"]),
                "suggested_extras": providers(r.get("missing_module")),
            }
            for r in found
        }

    def scanned(self, entry: dict[str, Any]) -> dict[str, Any]:
        """A /blueprints entry with what the last finished scan found for it."""
        cache = self.cache()
        record = cache["results"].get(entry["name"]) if cache else None
        return {**entry, **{k: record.get(k) for k in SCANNED}} if record else entry

    def catalog(self) -> dict[str, Any]:
        cache = self.cache() or {"results": {}, "modules": {}, "errors": []}
        records = list(cache["results"].values())
        used_by: dict[str, set[str]] = {}
        for record in records:
            for module in record["modules"]:
                if record["robot"]:
                    used_by.setdefault(module["class"], set()).add(record["robot"])
        blueprints = [
            {
                "name": r["name"],
                "ref": r["ref"],
                "robot": r["robot"],
                "modules": [m["module"] for m in r["modules"]],
                "doc": r.get("doc", ""),
            }
            for r in records
            if r["importable"] and r["builtin"]
        ]
        modules, skills = [], []
        for record in sorted(cache["modules"].values(), key=lambda m: m["name"]):
            owners = sorted(used_by.get(record["class"], ()))
            modules.append(
                {
                    **{k: v for k, v in record.items() if k != "skills"},
                    "robots": owners,
                    "skills": [s["name"] for s in record["skills"]],
                }
            )
            skills += [{**s, "module": record["name"], "robots": owners} for s in record["skills"]]
        errors = [
            f"blueprint {r['name']}: {r['import_error']}" for r in records if not r["importable"]
        ]
        errors += [e for e in cache.get("errors", []) if e.startswith("module ")]
        if len(errors) > MAX_ERRORS:
            errors = [*errors[:MAX_ERRORS], f"... and {len(errors) - MAX_ERRORS} more"]
        return {"blueprints": blueprints, "modules": modules, "skills": skills, "errors": errors}


def is_checkout(dimos_dir: Path) -> bool:
    return store.checkout_version(dimos_dir)[0] and (dimos_dir / "uv.lock").is_file()


def applies(requirement: Requirement, environment: dict[str, str], extra: str = "") -> bool:
    if requirement.marker is None:
        return True
    try:
        return bool(requirement.marker.evaluate({**environment, "extra": extra}))
    except Exception:
        return True


def declared(dimos_dir: Path, probe: dict[str, Any]) -> dict[str, list[str]]:
    try:
        project = tomllib.loads((dimos_dir / "pyproject.toml").read_text())["project"]
        if project.get("name") == "dimos":
            return {k: list(v) for k, v in project.get("optional-dependencies", {}).items()}
    except (OSError, KeyError, tomllib.TOMLDecodeError):
        pass
    environment = probe.get("environment", {})
    parsed = []
    for text in probe.get("dimos_requires", []):
        try:
            parsed.append((text, Requirement(text)))
        except InvalidRequirement:
            continue
    return {
        extra: [
            text
            for text, r in parsed
            if r.marker is not None
            and applies(r, environment, extra)
            and not applies(r, environment)
        ]
        for extra in probe.get("dimos_extras", [])
    }


def parse_extras(extras: dict[str, list[str]]) -> dict[str, tuple[list[Requirement], list[str]]]:
    """Each extra's requirements, and the other extras it includes through `dimos[...]`."""
    parsed: dict[str, tuple[list[Requirement], list[str]]] = {}
    for name, texts in extras.items():
        requirements, includes = [], []
        for text in texts:
            try:
                requirement = Requirement(text)
            except InvalidRequirement:
                continue
            if canonicalize_name(requirement.name) == "dimos":
                includes += sorted(requirement.extras)
            else:
                requirements.append(requirement)
        parsed[name] = (requirements, includes)
    return parsed


def extras_status(dimos_dir: Path, probe: dict[str, Any]) -> list[dict[str, Any]]:
    environment, installed = probe.get("environment", {}), probe.get("packages", {})
    extras = declared(dimos_dir, probe)
    parsed = {
        name: ([r for r in requirements if applies(r, environment, name)], includes)
        for name, (requirements, includes) in parse_extras(extras).items()
    }

    def missing(name: str, seen: frozenset[str] = frozenset()) -> list[str]:
        requirements, includes = parsed.get(name, ([], []))
        found: list[str] = []
        for requirement in requirements:
            version = installed.get(str(canonicalize_name(requirement.name)))
            if version is None or not requirement.specifier.contains(version, prereleases=True):
                found.append(canonicalize_name(requirement.name))
        for include in includes:
            if include not in seen:
                found += [m for m in missing(include, seen | {name}) if m not in found]
        return found

    lacking = {name: missing(name) for name in extras}
    return [
        {
            "name": name,
            "installed": not lacking[name],
            "applicable": bool(parsed[name][0] or parsed[name][1]),
            "requires": extras[name],
            "includes": parsed[name][1],
            "missing": lacking[name],
            "download_bytes": None,
        }
        for name in extras
    ]


def read_lock(dimos_dir: Path) -> dict[str, Any] | None:
    try:
        return tomllib.loads((dimos_dir / "uv.lock").read_text())
    except (OSError, tomllib.TOMLDecodeError):
        return None


def lock_closures(dimos_dir: Path) -> dict[str, set[str]]:
    lock = read_lock(dimos_dir) or {}
    by_name: dict[str, Any] = {canonicalize_name(e["name"]): e for e in lock.get("package", [])}
    dimos = by_name.get("dimos")
    if dimos is None:
        return {}
    closures = {}
    every = {"": dimos.get("dependencies", []), **dimos.get("optional-dependencies", {})}
    for extra, dependencies in every.items():
        seen: set[str] = set()
        stack = list(dependencies)
        while stack:
            name = canonicalize_name(stack.pop()["name"])
            if name in seen or name == "dimos" or name not in by_name:
                continue
            seen.add(name)
            stack += by_name[name].get("dependencies", [])
        closures[extra] = seen
    return closures


def providing_extras(dimos_dir: Path) -> Callable[[str | None], list[str]]:
    extras = parse_extras(declared(dimos_dir, {}))
    closures = lock_closures(dimos_dir)
    direct = {e: {canonicalize_name(r.name) for r in reqs} for e, (reqs, _) in extras.items()}

    def contains(extra: str, other: str, seen: frozenset[str] = frozenset()) -> bool:
        inner = set(extras.get(extra, ([], []))[1]) - seen
        return other in inner or any(contains(i, other, seen | {extra}) for i in inner)

    def providers(module: str | None) -> list[str]:
        if not module:
            return []
        names = {canonicalize_name(module), *IMPORT_ALIASES.get(module, ())}

        def provides(package: str) -> bool:
            return any(package == n or package.startswith(n + "-") for n in names)

        if any(provides(p) for p in closures.get("", ())):
            return []
        found = [e for e in extras if any(provides(p) for p in direct[e])]
        found = found or [e for e in extras if any(provides(p) for p in closures.get(e, ()))]
        minimal = [e for e in found if not any(o != e and contains(e, o) for o in found)]
        return sorted(minimal, key=lambda e: len(closures.get(e, ())))

    return providers


def find_uv() -> str | None:
    candidates = [Path.home() / ".local/bin/uv", Path.home() / ".cargo/bin/uv"]
    return shutil.which("uv") or next((str(c) for c in candidates if c.is_file()), None)


def install_command(dimos_dir: Path, extras: list[str], uv: str) -> list[str]:
    if is_checkout(dimos_dir):
        flags = [arg for extra in extras for arg in ("--extra", extra)]
        return [uv, "sync", "--locked", "--inexact", "--no-progress", *flags]
    version = store.checkout_version(dimos_dir)[1]
    spec = f"dimos[{','.join(extras)}]" + (f"=={version}" if version else "")
    python = ["--python", store.python_for(dimos_dir)]
    torch = ["--torch-backend", "cu128" if "cuda" in extras else "cpu"]
    return [uv, "pip", "install", "--no-progress", *python, *torch, spec]


@dataclass
class Job:
    id: str
    title: str
    kind: str
    command: list[str]
    lines: list[str] = field(default_factory=list)
    done: bool = False
    ok: bool | None = None
    error: str | None = None
    failure: list[str] = field(default_factory=list)
    started_at: str = field(default_factory=now_iso)
    finished_at: str | None = None
    ended: float | None = None
    code: str | None = None
    progress: dict[str, Any] | None = None

    def finish(self, ok: bool, error: str | None = None) -> None:
        self.ok, self.error = ok, error
        self.done, self.finished_at, self.ended = True, now_iso(), time.monotonic()

    def status(self) -> dict[str, Any]:
        return {
            "job": self.id,
            "title": self.title,
            "kind": self.kind,
            "done": self.done,
            "ok": self.ok,
            "progress": self.progress,
            "error": self.error,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
        }

    def log(self, after: int = 0) -> dict[str, Any]:
        return {
            **self.status(),
            "command": self.command,
            "lines": self.lines[max(after, 0) :],
            "next": len(self.lines),
            "failure": self.failure,
            "code": self.code,
        }


Prepare = Callable[[Callable[[list[str]], Awaitable[int]]], Awaitable[dict[str, str]]]


class MissingForJobError(Exception):
    pass


class Jobs:
    def __init__(self) -> None:
        self.jobs: dict[str, Job] = {}
        self.ids = itertools.count(1)

    def prune(self) -> None:
        cutoff = time.monotonic() - JOB_KEEP_S
        self.jobs = {k: j for k, j in self.jobs.items() if j.ended is None or j.ended >= cutoff}

    def get(self, job_id: str) -> Job | None:
        self.prune()
        return self.jobs.get(job_id)

    def running(self, kind: str) -> Job | None:
        return next((j for j in self.jobs.values() if j.kind == kind and not j.done), None)

    def add(self, title: str, kind: str, command: list[str]) -> Job:
        self.prune()
        job = Job(f"{kind}-{next(self.ids)}-{int(time.time())}", title, kind, command)
        self.jobs[job.id] = job
        return job

    def start(
        self,
        title: str,
        kind: str,
        command: list[str],
        cwd: Path,
        env: dict[str, str],
        then: Callable[[], None],
        prepare: Prepare | None = None,
    ) -> Job:
        job = self.add(title, kind, command)

        async def run() -> None:
            try:
                ready = env
                if prepare is not None:
                    ready = {**env, **await prepare(lambda line: self.step(job, line, cwd, env))}
                code = await self.step(job, command, cwd, ready)
                job.ok = code == 0
                if code != 0:
                    job.error = f"{title} failed (exit {code})"
                    job.failure = [line for line in job.lines if line.strip()][-15:]
            except MissingForJobError as error:
                job.ok, job.code, job.error = False, "cyclonedds_missing", str(error)
                job.failure = [job.error]
                job.lines.append(job.error)
            except Exception as error:
                job.ok = False
                job.error = f"{title} couldn't start: {type(error).__name__}: {error}"
                job.failure = [job.error]
            job.finish(bool(job.ok), job.error)
            then()

        asyncio.get_running_loop().create_task(run())
        return job

    async def step(self, job: Job, command: list[str], cwd: Path, env: dict[str, str]) -> int:
        job.lines.append("$ " + " ".join(command))
        process = await asyncio.create_subprocess_exec(
            *command,
            cwd=cwd,
            env={**os.environ, "NO_COLOR": "1", **env},
            stdin=asyncio.subprocess.DEVNULL,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
            start_new_session=True,
        )
        assert process.stdout
        async for raw in process.stdout:
            job.lines += ANSI.sub("", raw.decode("utf-8", "replace")).rstrip("\r\n").split("\r")
        return await process.wait()


BREWED_CYCLONEDDS = (Path("/opt/homebrew/opt/cyclonedds"), Path("/usr/local/opt/cyclonedds"))


def has_wheel_here(entry: dict[str, Any], environment: dict[str, str]) -> bool:
    system = {"Darwin": "macosx", "Linux": "linux"}.get(
        environment.get("platform_system", ""), "win"
    )
    machine = environment.get("platform_machine", "").lower()
    cpus = {"arm64": ("arm64", "universal2"), "aarch64": ("aarch64",)}.get(machine, (machine,))
    python = "cp" + "".join(environment.get("python_version", "3.12").split(".")[:2])
    for wheel in entry.get("wheels", []):
        file = str(wheel.get("url", "")).rsplit("/", 1)[-1]
        tags = file.removesuffix(".whl").split("-")[-3:] if file.endswith(".whl") else []
        if len(tags) != 3:
            continue
        py_ok = tags[0].startswith(("py3", "py2.py3")) or python in tags[0] or tags[1] == "abi3"
        plat_ok = tags[2] == "any" or (system in tags[2] and any(c in tags[2] for c in cpus))
        if py_ok and plat_ok:
            return True
    return False


def builds_cyclonedds(dimos_dir: Path, probe: dict[str, Any], wanted: list[str]) -> bool:
    statuses = extras_status(dimos_dir, probe)
    if not any("cyclonedds" in e["missing"] for e in statuses if e["name"] in wanted):
        return False
    lock = read_lock(dimos_dir) or {}
    entry = next((e for e in lock.get("package", []) if e["name"] == "cyclonedds"), None)
    return entry is None or not has_wheel_here(entry, probe.get("environment", {}))


def find_nix() -> str | None:
    fixed = [Path("/nix/var/nix/profiles/default/bin/nix"), Path("/run/current-system/sw/bin/nix")]
    return shutil.which("nix") or next((str(p) for p in fixed if p.is_file()), None)


def nixpkgs_ref(dimos_dir: Path) -> str:
    try:
        lock = json.loads((dimos_dir / "flake.lock").read_text())
        nodes = lock["nodes"]
        locked = nodes[nodes[lock["root"]]["inputs"]["nixpkgs"]]["locked"]
        if locked.get("type") == "github":
            return f"github:{locked['owner']}/{locked['repo']}/{locked['rev']}"
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return "nixpkgs"


def cyclonedds_env(home: Path) -> dict[str, str]:
    prefix = os.pathsep.join(p for p in (str(home), os.environ.get("CMAKE_PREFIX_PATH", "")) if p)
    return {"CYCLONEDDS_HOME": str(home), "CMAKE_PREFIX_PATH": prefix}


def prepare_cyclonedds(dimos_dir: Path) -> Prepare:
    async def prepare(step: Callable[[list[str]], Awaitable[int]]) -> dict[str, str]:
        given = os.environ.get("CYCLONEDDS_HOME")
        if given and (Path(given) / "lib").is_dir():
            return cyclonedds_env(Path(given))
        link = store.venv_dir(dimos_dir) / "cyclonedds"
        nix = find_nix()
        if nix is not None:
            features = ["--extra-experimental-features", "nix-command flakes"]
            flake = f"{nixpkgs_ref(dimos_dir)}#cyclonedds"
            built = await step([nix, *features, "build", "--out-link", str(link), flake]) == 0
            if built and (link / "lib").is_dir():
                return cyclonedds_env(link.resolve())
        for brewed in BREWED_CYCLONEDDS:
            if (brewed / "lib").is_dir():
                return cyclonedds_env(brewed.resolve())
        raise MissingForJobError(
            "The cyclonedds package is built here against the CycloneDDS C library, and none was found"
            + (" (nix couldn't build it, see above)" if nix else "")
            + ": install nix, or `brew install cyclonedds`, or set CYCLONEDDS_HOME to an install of"
            " CycloneDDS 0.10, then install again."
        )

    return prepare


def docs_site(dimos_dir: Path) -> tuple[str | None, str | None]:
    try:
        settings = yaml.load((dimos_dir / "mkdocs.yml").read_text(), Loader=yaml.BaseLoader)
    except (OSError, yaml.YAMLError):
        return None, None
    settings = settings if isinstance(settings, dict) else {}
    url, repo = settings.get("site_url"), settings.get("repo_url")
    return (str(url).rstrip("/") + "/" if url else None, str(repo) if repo else None)


def docs_search_url(dimos_dir: Path, query: str) -> str | None:
    """The docs site with its search open on `query` (mkdocs-material reads ?q=); None when the checkout names no site."""
    from urllib.parse import quote

    base, _ = docs_site(dimos_dir)
    return f"{base}?q={quote(query)}" if base else None
