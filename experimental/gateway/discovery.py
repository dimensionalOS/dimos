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
from collections.abc import Callable
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
from experimental.gateway import annotations as annotation_file, constants, store
from experimental.gateway.introspect import MARKER, child_command, kill_group

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

logger = setup_logger()

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


def failed_record(name: str, problem: str) -> dict[str, Any]:
    """A blueprint whose import crashed or hung the scan's child process."""
    return {
        "name": name,
        "ref": None,
        "builtin": None,
        "importable": False,
        "optional_dependency": False,
        "import_error": problem,
        "missing_module": None,
        "modules": [],
    }


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def fingerprint(dimos_dir: Path) -> str:
    """What a scan's results depend on: the checkout (commit, and each changed file's contents) and the venv."""
    parts = [str(dimos_dir.resolve()), str(store.checkout_version(dimos_dir))]
    parts.append(store.git(dimos_dir, "rev-parse", "HEAD") or "")
    for line in (
        store.git(dimos_dir, "status", "--porcelain", "--untracked-files=all") or ""
    ).splitlines():
        path = dimos_dir / line[3:].split(" -> ")[-1].strip('"')
        # contents, not mtime: Linux mtimes are coarse, so a same-size edit within ~4ms looks unchanged
        try:
            parts.append(f"{line} {hashlib.sha256(path.read_bytes()).hexdigest()}")
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


# a saved scan in an older format reads as never scanned (2: modules carry rpcs, not skills)
SCAN_FORMAT = 2


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
        self.item_timeout_s = constants.SCAN_ITEM_TIMEOUT_S
        self.looked = False
        self.now_print: tuple[float, str] | None = None
        # one child at a time: a full scan and a single blueprint's share the fields above
        self.lock = asyncio.Lock()
        self.single: set[asyncio.Task[None]] = set()

    def cache(self) -> dict[str, Any] | None:
        """The last finished scan (from disk after a restart), or None if this checkout was never scanned."""
        if not self.looked:
            self.looked = True
            saved = store.read_json(saved_scan_file(self.dimos_dir))
            if (
                isinstance(saved, dict)
                and saved.get("format") == SCAN_FORMAT
                and {"results", "modules", "finished_at"} <= saved.keys()
            ):
                self.saved = saved
        return self.saved

    def current_fingerprint(self) -> str:
        if (
            self.now_print is None
            or time.monotonic() - self.now_print[0] > constants.FINGERPRINT_CACHE_S
        ):
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
            "scanned": finished is not None,
            "finished_at": finished,
            "age_seconds": age,
            "stale": cache is None or cache.get("fingerprint") != self.current_fingerprint(),
        }

    def start(
        self, reason: str = "requested", item_timeout_s: float = constants.SCAN_ITEM_TIMEOUT_S
    ) -> Job:
        """The running scan's job, or a new scan's."""
        if self.task is None or self.task.done():
            self.item_timeout_s = item_timeout_s
            self.state = "scanning"
            self.job = self.jobs.add(f"Scan blueprints ({reason})", "scan", [*self.command, "scan"])
            self.task = asyncio.get_running_loop().create_task(self.scan(self.job))
        assert self.job is not None
        return self.job

    def pending(self) -> tuple[list[str], list[str]]:
        return (
            [name for name in self.names["blueprints"] if name not in self.blueprints],
            [name for name in self.names["modules"] if name not in self.finished_modules],
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

    def start_one(self, name: str, item_timeout_s: float = constants.SCAN_ITEM_TIMEOUT_S) -> Job:
        """A scan of just `name` and its modules (the full scan's job if one is running)."""
        if self.task is not None and not self.task.done():
            assert self.job is not None
            return self.job
        self.item_timeout_s = item_timeout_s
        job = self.jobs.add(f"Scan {name}", "scan", [*self.command, "scan"])
        task = asyncio.get_running_loop().create_task(self.scan_one(job, name))
        self.single.add(task)
        task.add_done_callback(self.single.discard)
        return job

    def running(self) -> list[asyncio.Task[None]]:
        """The scans in flight, the full one and single blueprints'."""
        return [task for task in (self.task, *self.single) if task is not None and not task.done()]

    async def scan_one(self, job: Job, name: str) -> None:
        ok = False
        async with self.lock:
            self.blueprints, self.modules, self.errors = {}, {}, []
            self.names, self.finished_modules = {"blueprints": [name], "modules": []}, set()
            self.job = job
            try:
                problem = await self.child({"blueprints": [name], "modules": [], "known": []})
                if name not in self.blueprints:
                    self.blueprints[name] = failed_record(name, problem or "no answer")
                found = await asyncio.to_thread(self.finish)
                cache = self.cache() or {
                    "format": SCAN_FORMAT,
                    "fingerprint": None,
                    "finished_at": None,
                    "results": {},
                    "modules": {},
                    "errors": [],
                    "names": {"blueprints": [], "modules": []},
                }
                cache["results"][name] = {**found[name], "scanned_at": now_iso()}
                cache["modules"].update(self.modules)
                self.saved, self.looked = cache, True
                await asyncio.to_thread(self.save)
                ok = True
            except Exception as error:
                logger.exception("scanning a blueprint failed", blueprint=name)
                self.error(f"scan {name}: {type(error).__name__}: {error}")
            finally:
                job.finish(ok, None if ok else (self.errors or ["the scan was cancelled"])[-1])

    async def scan(self, job: Job) -> None:
        async with self.lock:
            await self.scan_all(job)

    async def scan_all(self, job: Job) -> None:
        self.job = job
        self.blueprints, self.modules, self.errors = {}, {}, []
        self.names, self.finished_modules = {"blueprints": [], "modules": []}, set()
        self.current = None
        # taken before the imports, so a change made while scanning still reads as stale
        before = await asyncio.to_thread(fingerprint, self.dimos_dir)
        ok = False
        try:
            problem = await self.child(None)
            tried: Any = None
            while problem is not None and any(self.pending()):
                current = self.current or ""
                self.error(f"{current}: {problem}")
                if current in self.pending()[0]:
                    self.blueprints[current] = failed_record(current, problem)
                elif current.startswith("module:"):
                    self.finished_modules.add(current[7:])
                # else the child died between items (after a blueprint's record): go on with what's left
                if self.pending() == tried:
                    left = sum(map(len, tried))
                    raise RuntimeError(f"the scan child keeps exiting ({problem}) with {left} left")
                tried = self.pending()
                blueprints, modules = tried
                request = {
                    "blueprints": blueprints,
                    "modules": modules,
                    "known": list(self.modules),
                }
                problem = await self.child(request)
            self.current = None
            self.tick()
            results = await asyncio.to_thread(self.finish)
            finished = now_iso()
            self.saved = {
                "format": SCAN_FORMAT,
                "fingerprint": before,
                "finished_at": finished,
                "results": {
                    name: {**result, "scanned_at": finished} for name, result in results.items()
                },
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
            limit=constants.SCAN_STDOUT_LIMIT_BYTES,
        )
        assert child.stdin and child.stdout and child.stderr
        child.stdin.write(json.dumps(request).encode())
        child.stdin.close()
        tail: deque[str] = deque(maxlen=constants.SCAN_STDERR_TAIL_LINES)

        async def drain() -> None:
            assert child.stderr
            async for line in child.stderr:
                tail.append(line.decode("utf-8", "replace").rstrip())

        draining = asyncio.create_task(drain())
        try:
            while True:
                try:
                    line = await asyncio.wait_for(child.stdout.readline(), self.item_timeout_s)
                except asyncio.TimeoutError:
                    return f"no answer in {self.item_timeout_s:g} s (hung while importing)"
                if not line:
                    await child.wait()
                    last = next(
                        (tail_line for tail_line in reversed(tail) if tail_line.strip()),
                        "no output",
                    )
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
        notes, providers = annotation_file.load(), providing_extras(self.dimos_dir)
        order = {name: index for index, name in enumerate(self.names["blueprints"])}
        found = sorted(
            self.blueprints.values(), key=lambda record: order.get(record["name"], len(order))
        )
        return {
            record["name"]: {
                **record,
                "robot": annotation_file.robot_of(notes, record["name"], record["ref"]),
                "suggested_extras": providers(record.get("missing_module")),
            }
            for record in found
        }

    def scanned(self, entry: dict[str, Any]) -> dict[str, Any]:
        """A /blueprints entry with what the last finished scan found for it."""
        cache = self.cache()
        record = cache["results"].get(entry["name"]) if cache else None
        return {**entry, **{key: record.get(key) for key in SCANNED}} if record else entry


def is_checkout(dimos_dir: Path) -> bool:
    return store.checkout_version(dimos_dir)[0] and (dimos_dir / "uv.lock").is_file()


def applies(requirement: Requirement, environment: dict[str, str], extra: str = "") -> bool:
    if requirement.marker is None:
        return True
    try:
        return bool(requirement.marker.evaluate({**environment, "extra": extra}))
    except ValueError:
        return True


def declared(dimos_dir: Path, probe: dict[str, Any]) -> dict[str, list[str]]:
    try:
        project = tomllib.loads((dimos_dir / "pyproject.toml").read_text())["project"]
        if project.get("name") == "dimos":
            return {
                extra_name: list(requirements)
                for extra_name, requirements in project.get("optional-dependencies", {}).items()
            }
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
            for text, requirement in parsed
            if requirement.marker is not None
            and applies(requirement, environment, extra)
            and not applies(requirement, environment)
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
        name: (
            [required for required in requirements if applies(required, environment, name)],
            includes,
        )
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
                found += [
                    package for package in missing(include, seen | {name}) if package not in found
                ]
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
    by_name: dict[str, Any] = {
        canonicalize_name(package["name"]): package for package in lock.get("package", [])
    }
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
    direct = {
        extra_name: {canonicalize_name(requirement.name) for requirement in requirements}
        for extra_name, (requirements, _) in extras.items()
    }

    def contains(extra: str, other: str, seen: frozenset[str] = frozenset()) -> bool:
        inner = set(extras.get(extra, ([], []))[1]) - seen
        return other in inner or any(
            contains(included, other, seen | {extra}) for included in inner
        )

    def providers(module: str | None) -> list[str]:
        if not module:
            return []
        names = {canonicalize_name(module), *IMPORT_ALIASES.get(module, ())}

        def provides(package: str) -> bool:
            return any(package == name or package.startswith(name + "-") for name in names)

        if any(provides(dependency) for dependency in closures.get("", ())):
            return []
        found = [
            extra_name
            for extra_name in extras
            if any(provides(dependency) for dependency in direct[extra_name])
        ]
        found = found or [
            extra_name
            for extra_name in extras
            if any(provides(dependency) for dependency in closures.get(extra_name, ()))
        ]
        minimal = [
            extra_name
            for extra_name in found
            if not any(
                other_extra != extra_name and contains(extra_name, other_extra)
                for other_extra in found
            )
        ]
        return sorted(minimal, key=lambda extra_name: len(closures.get(extra_name, ())))

    return providers


def find_uv() -> str | None:
    candidates = [Path.home() / ".local/bin/uv", Path.home() / ".cargo/bin/uv"]
    return shutil.which("uv") or next(
        (str(candidate) for candidate in candidates if candidate.is_file()), None
    )


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
        }


class Jobs:
    def __init__(self) -> None:
        self.jobs: dict[str, Job] = {}
        self.ids = itertools.count(1)
        self.tasks: set[asyncio.Task[None]] = set()

    def prune(self) -> None:
        """Keeps every running job and the latest finished job of each kind."""
        latest: dict[str, Job] = {}
        for job in self.jobs.values():
            kept = latest.get(job.kind)
            if job.ended is not None and (kept is None or job.ended >= (kept.ended or 0.0)):
                latest[job.kind] = job
        self.jobs = {
            job_id: job
            for job_id, job in self.jobs.items()
            if job.ended is None or latest.get(job.kind) is job
        }

    def get(self, job_id: str) -> Job | None:
        self.prune()
        return self.jobs.get(job_id)

    def running(self, kind: str) -> Job | None:
        return next((job for job in self.jobs.values() if job.kind == kind and not job.done), None)

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
    ) -> Job:
        job = self.add(title, kind, command)

        async def run() -> None:
            try:
                code = await self.step(job, command, cwd, env)
                job.ok = code == 0
                if code != 0:
                    job.error = f"{title} failed (exit {code})"
                    printed = [line for line in job.lines if line.strip()]
                    job.failure = printed[-constants.JOB_FAILURE_TAIL_LINES :]
            except OSError as error:
                job.ok = False
                job.error = f"{title} couldn't start: {type(error).__name__}: {error}"
                job.failure = [job.error]
            finally:
                # a cancel or a bug still ends the job, and the bug's traceback reaches the log
                if job.ok is None:
                    job.error = f"{title} stopped before it finished (see the gateway log)"
                job.finish(bool(job.ok), job.error)
                then()

        task = asyncio.get_running_loop().create_task(run())
        self.tasks.add(task)
        task.add_done_callback(self.tasks.discard)
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
            # without the ANSI colour codes
            text = re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", raw.decode("utf-8", "replace"))
            job.lines += text.rstrip("\r\n").split("\r")
        return await process.wait()


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
