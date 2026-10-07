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

"""The discovery cache: every blueprint in dimos's registry, whether it imports, its modules and their streams, every
module's config, and the message types between them.

The server starts a scan as soon as it starts. The scan runs `python -m dimos.server.discover` children (the checkout's
own python) that import one blueprint after another and stream an answer per blueprint; a blueprint that hangs (no
answer for `item_timeout` s) or crashes the child is marked not importable and a new child carries on with the rest.

The answer is saved to `<server state>/discovery/<key>.json`. The key is a hash of the checkout's commit, its dirty
files (path, size, mtime) and the installed packages (the venv's *.dist-info names), so a restart with nothing changed
answers at once with no scan. The key is checked again every `check_interval` s and after an extras install. When it
changes, the previous answer keeps being served (`stale: true`) while a new scan runs, except when nothing that can
change an import changed (can_change_imports: no file inside an importable package, no Python file, no packaging
metadata; e.g. only docs or a README), when the old answer is re-keyed without a scan. An interrupted scan resumes
where it stopped (its partial file has the same key).
"""

from __future__ import annotations

import asyncio
import collections
from collections.abc import Callable
import fnmatch
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import sysconfig
import time
from typing import Any

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

from dimos.server import config
from dimos.server.introspect import MARKER
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

CACHE_VERSION = 1
KEEP_CACHE_FILES = 5
EVENT_INTERVAL_S = 0.5
SAVE_INTERVAL_S = 3.0
MAX_ERRORS = 50
# a Python file anywhere can be imported (`python -m` puts the checkout on sys.path), and these say what's installed
PYTHON_SUFFIXES = (".py", ".pyi", ".so", ".pth")
PACKAGING_FILES = ("pyproject.toml", "uv.lock", "setup.py", "setup.cfg")

FORMULA = (
    "score = (blueprints of this robot that use the module / blueprints of this robot) "
    "* ln(robots / robots whose blueprints use the module); a module every robot uses scores 0"
)


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def python_for(dimos_dir: Path) -> str:
    """The checkout's own python (what its blueprints run with), else this server's."""
    venv_python = config.venv_dir(dimos_dir) / "bin" / "python"
    return str(venv_python) if venv_python.exists() else sys.executable


def site_dirs(dimos_dir: Path) -> list[Path]:
    found = sorted((config.venv_dir(dimos_dir) / "lib").glob("python*/site-packages"))
    return found or [Path(sysconfig.get_paths()["purelib"])]


def git(dimos_dir: Path, *args: str) -> str | None:
    try:
        done = subprocess.run(
            ["git", "-C", str(dimos_dir), *args],
            capture_output=True,
            text=True,
            timeout=15,
            stdin=subprocess.DEVNULL,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    return done.stdout if done.returncode == 0 else None


def checkout_key(dimos_dir: Path) -> dict[str, Any]:
    """What the cache is good for: {digest, commit, dirty, packages}. Cheap (two git calls and a directory listing)."""
    commit = (git(dimos_dir, "rev-parse", "HEAD") or "").strip() or None
    status = git(dimos_dir, "status", "--porcelain=v1", "--untracked-files=normal") or ""
    dirty: dict[str, str] = {}
    for line in status.splitlines():
        path = line[3:].split(" -> ")[-1].strip('"')
        try:
            stat = (dimos_dir / path).stat()
            dirty[path] = f"{stat.st_size}:{stat.st_mtime_ns}"
        except OSError:
            dirty[path] = "gone"
    installed = sorted(
        entry.name
        for site in site_dirs(dimos_dir)
        if site.is_dir()
        for entry in site.iterdir()
        if entry.name.endswith((".dist-info", ".egg-info", ".pth"))
    )
    packages = hashlib.sha256("\n".join(installed).encode()).hexdigest()[:16]
    parts = {
        "commit": commit,
        "dirty": dirty,
        "packages": packages,
        "python": python_for(dimos_dir),
    }
    digest = hashlib.sha256(json.dumps(parts, sort_keys=True).encode()).hexdigest()[:16]
    return {"digest": digest, **parts}


def code_changed(dimos_dir: Path, old: dict[str, Any], new: dict[str, Any]) -> bool:
    """Whether anything that can change an import differs between two keys (True when unsure)."""
    if old.get("packages") != new.get("packages") or old.get("python") != new.get("python"):
        return True
    changed = set(old.get("dirty", {})) | set(new.get("dirty", {}))
    if old.get("commit") != new.get("commit"):
        if not old.get("commit") or not new.get("commit"):
            return True
        listed = git(dimos_dir, "diff", "--name-only", old["commit"], new["commit"])
        if listed is None:
            return True
        changed |= set(listed.splitlines())
    else:
        # same commit: only the dirty files that differ between the two keys matter
        changed = {
            path
            for path in changed
            if old.get("dirty", {}).get(path) != new.get("dirty", {}).get(path)
        }
    packages = package_patterns(dimos_dir)
    return any(can_change_imports(path, packages) for path in changed)


def package_patterns(dimos_dir: Path) -> list[str] | None:
    """pyproject.toml's `[tool.setuptools.packages.find] include` (`dimos*`): the folders whose files are importable,
    or None when it can't be read."""
    try:
        project = tomllib.loads((dimos_dir / "pyproject.toml").read_text())
        found = project["tool"]["setuptools"]["packages"]["find"]["include"]
    except (OSError, KeyError, TypeError, tomllib.TOMLDecodeError):
        return None
    return [str(pattern) for pattern in found]


def can_change_imports(path: str, packages: list[str] | None) -> bool:
    """A change to `path` can change what imports: any file inside an importable package (its code, or data it reads
    while importing: yaml, json, URDF), a Python file anywhere, or the packaging metadata. Docs, READMEs, web
    frontends, CI files can't. Without the package list, everything counts."""
    if packages is None:
        return True
    top = path.split("/", 1)[0]
    return (
        any(fnmatch.fnmatch(top, pattern) for pattern in packages)
        or path.endswith(PYTHON_SUFFIXES)
        or path.rsplit("/", 1)[-1] in PACKAGING_FILES
    )


class Discovery:
    def __init__(
        self,
        dimos_dir: Path,
        send: Callable[[dict[str, Any]], None],
        cache_dir: Path | None = None,
        command: list[str] | None = None,
        item_timeout: float = 120.0,
        check_interval: float = 30.0,
        key: Callable[[Path], dict[str, Any]] = checkout_key,
    ) -> None:
        self.dimos_dir = dimos_dir
        self.send = send
        self.cache_dir = cache_dir or config.server_dir() / "discovery"
        # the child's command, without its sub-command (tests swap in a fake)
        self.command = command or [python_for(dimos_dir), "-m", "dimos.server.discover"]
        self.item_timeout = item_timeout
        self.check_interval = check_interval
        self.key = key
        self.data: dict[str, Any] = self.empty({})
        self.current_key: dict[str, Any] = {}
        # the previous answer, served while a changed checkout is scanned
        self.stale_data: dict[str, Any] | None = None
        self.lock = asyncio.Lock()
        self.wake = asyncio.Event()
        self.requested: tuple[str, bool] | None = None
        self.last_event = 0.0
        self.status: dict[str, Any] = {
            "state": "idle",
            "reason": None,
            "key": None,
            "stale": False,
            "started_at": None,
            "finished_at": None,
            "blueprints_total": 0,
            "blueprints_done": 0,
            "importable": 0,
            "not_importable": 0,
            "modules_total": 0,
            "modules_done": 0,
            "current": None,
            "cached_from": None,
            "errors": [],
        }

    @staticmethod
    def empty(key: dict[str, Any]) -> dict[str, Any]:
        return {
            "version": CACHE_VERSION,
            "key": key,
            "complete": False,
            "names": {"blueprints": [], "modules": []},
            "blueprints": {},
            "modules": {},
            "module_errors": {},
            "errors": [],
            "scanned_at": None,
        }

    # persistence

    def path_for(self, digest: str) -> Path:
        return self.cache_dir / f"{digest}.json"

    def read(self, path: Path) -> dict[str, Any] | None:
        try:
            value = json.loads(path.read_text())
        except (OSError, ValueError):
            return None
        return value if isinstance(value, dict) and value.get("version") == CACHE_VERSION else None

    def latest(self) -> dict[str, Any] | None:
        files = sorted(self.cache_dir.glob("*.json"), key=lambda f: f.stat().st_mtime, reverse=True)
        for file in files:
            value = self.read(file)
            if value is not None:
                return value
        return None

    async def save(self) -> None:
        """The data to disk: serialized here (the loop is the only writer of it), written in a thread."""
        digest = self.data["key"].get("digest")
        if digest:
            await asyncio.to_thread(self.write, self.path_for(digest), json.dumps(self.data))

    def write(self, path: Path, text: str) -> None:
        try:
            config.write_atomic(path, text)
            files = sorted(
                self.cache_dir.glob("*.json"), key=lambda f: f.stat().st_mtime, reverse=True
            )
            for old in files[KEEP_CACHE_FILES:]:
                old.unlink(missing_ok=True)
        except OSError as error:
            logger.warning("couldn't save the discovery cache", error=str(error))

    def load_cached(self) -> None:
        """At startup: the newest saved answer, to serve while the key is checked."""
        value = self.latest()
        if value is not None:
            self.data = value
            self.status["cached_from"] = value.get("scanned_at")
            self.update_counts()

    # status and events

    def update_counts(self) -> None:
        records = self.data["blueprints"].values()
        importable = sum(1 for record in records if record.get("importable"))
        served = self.answer()["key"].get("digest")
        self.status.update(
            key=served,
            stale=bool(self.current_key) and served != self.current_key.get("digest"),
            blueprints_total=len(self.data["names"]["blueprints"]) or len(self.data["blueprints"]),
            blueprints_done=len(self.data["blueprints"]),
            importable=importable,
            not_importable=len(self.data["blueprints"]) - importable,
            modules_total=len(self.data["names"]["modules"]),
            modules_done=sum(
                1
                for name in self.data["names"]["modules"]
                if name in self.data["module_errors"] or lookup(self.data["modules"], name)
            ),
            errors=self.data["errors"][-MAX_ERRORS:],
        )

    def notify(self, force: bool = False) -> None:
        if not force and time.monotonic() - self.last_event < EVENT_INTERVAL_S:
            return
        self.last_event = time.monotonic()
        self.update_counts()
        self.send({"type": "discovery", "status": dict(self.status)})

    # the loop

    async def run(self) -> None:
        """Forever: check the key now, then every check_interval s or when refresh() asks."""
        await asyncio.to_thread(self.load_cached)
        while True:
            requested, self.requested = self.requested, None
            try:
                await self.check(
                    *(requested or ("startup" if not self.current_key else "changed", False))
                )
            except asyncio.CancelledError:
                raise
            except Exception as error:
                logger.exception("discovery failed")
                self.status.update(state="failed", finished_at=now_iso())
                self.data["errors"].append(f"scan: {type(error).__name__}: {error}")
                self.notify(force=True)
            self.wake.clear()
            try:
                await asyncio.wait_for(self.wake.wait(), self.check_interval)
            except asyncio.TimeoutError:
                pass

    def refresh(self, reason: str = "requested", full: bool = False) -> None:
        self.requested = (reason, full)
        self.wake.set()

    async def check(self, reason: str, full: bool = False) -> None:
        async with self.lock:
            key = await asyncio.to_thread(self.key, self.dimos_dir)
            self.current_key = key
            data = self.data
            if not full and data["key"].get("digest") == key["digest"] and data["complete"]:
                self.finished()
                return
            if full:
                self.data = self.empty(key)
            elif data["key"].get("digest") != key["digest"]:
                saved = await asyncio.to_thread(self.read, self.path_for(key["digest"]))
                if saved is not None:
                    self.data = saved  # this key was scanned before (or partly)
                elif data["complete"] and not await asyncio.to_thread(
                    code_changed, self.dimos_dir, data["key"], key
                ):
                    self.data = {**data, "key": key}
                    await self.save()
                    self.finished()
                    return
                else:
                    # serve the old answer (stale) until the new one is in
                    self.stale_data = data
                    self.data = self.empty(key)
            if self.data["complete"]:
                self.stale_data = None
                self.finished()
                return
            await self.scan(reason)

    def finished(self) -> None:
        """The data is complete for the current key (from the cache or a re-key): say so once."""
        changed = self.status["state"] != "done" or self.status["key"] != self.data["key"].get(
            "digest"
        )
        if self.status["state"] != "done":
            self.status.update(state="done", finished_at=self.data.get("scanned_at"), current=None)
        if changed:
            self.notify(force=True)
        else:
            self.update_counts()

    async def scan(self, reason: str) -> None:
        self.status.update(
            state="scanning", reason=reason, started_at=now_iso(), finished_at=None, current=None
        )
        self.notify(force=True)
        names = await self.child_answer("names")
        if "error" in names:
            raise RuntimeError(names["error"])
        self.data["names"] = {"blueprints": names["blueprints"], "modules": names["modules"]}
        self.data["errors"] += names.get("errors", [])
        todo_blueprints = [n for n in names["blueprints"] if n not in self.data["blueprints"]]
        todo_modules = [
            n
            for n in names["modules"]
            if n not in self.data["module_errors"] and not lookup(self.data["modules"], n)
        ]
        await self.scan_items(todo_blueprints, todo_modules)
        self.data.update(complete=True, scanned_at=now_iso())
        self.stale_data = None
        await self.save()
        self.status.update(state="done", finished_at=now_iso(), current=None, cached_from=None)
        self.notify(force=True)

    async def child_messages(
        self, command: str, request: dict[str, Any] | None = None
    ) -> list[dict[str, Any]]:
        """Every MARKER line of one child run to its end, or [{"error"}] when it fails or takes too long."""
        child = await asyncio.create_subprocess_exec(
            *self.command,
            command,
            cwd=self.dimos_dir,
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            start_new_session=True,
            limit=64 * 1024 * 1024,
        )
        try:
            stdout, stderr = await asyncio.wait_for(
                child.communicate(json.dumps(request or {}).encode()), self.item_timeout
            )
        except asyncio.TimeoutError:
            kill(child)
            await child.wait()
            return [{"error": f"`discover {command}` took over {self.item_timeout:.0f} s"}]
        messages = [
            json.loads(line.rpartition(MARKER)[2])
            for line in stdout.decode("utf-8", "replace").splitlines()
            if MARKER in line
        ]
        if not messages:
            tail = (stderr.decode("utf-8", "replace").strip().splitlines() or ["no output"])[-1]
            return [{"error": f"`discover {command}` failed (exit {child.returncode}): {tail}"}]
        return messages

    async def child_answer(self, command: str) -> dict[str, Any]:
        return (await self.child_messages(command))[-1]

    async def module_now(self, name: str) -> dict[str, Any] | None:
        """A registry module the scan hasn't reached: imported now, in its own child. None when it isn't in the
        registry; {"error"} when it doesn't import."""
        for message in await self.child_messages("scan", {"modules": [name]}):
            kind = message.pop("kind", None)
            if kind == "module":
                self.data["modules"].setdefault(message["class"], message)
                return message
            if kind == "module-error":
                if message.get("unknown"):
                    return None
                return {"error": message.get("import_error")}
            if "error" in message:
                return {"error": message["error"]}
        return None

    async def scan_items(self, blueprints: list[str], modules: list[str]) -> None:
        """Children until every name has an answer: a crash or hang costs only the name in flight."""
        todo = [*blueprints, *(f"module:{name}" for name in modules)]
        stalled = 0
        while todo:
            finished, problem = await self.one_child(todo)
            todo = [name for name in todo if name not in finished]
            if problem is None:
                break
            current = self.status["current"]
            if current in todo:
                self.record_failure(current, problem)
                todo.remove(current)
                stalled = 0
            else:
                stalled += 1
                self.data["errors"].append(f"scan: {problem}")
                if stalled >= 2:
                    for name in todo:
                        self.record_failure(name, problem)
                    break
            self.notify()

    def record_failure(self, name: str, problem: str) -> None:
        self.data["errors"].append(f"{name}: {problem}")
        if name.startswith("module:"):
            self.data["module_errors"][name[7:]] = {"import_error": problem}
            return
        self.data["blueprints"][name] = {
            "name": name,
            "ref": None,
            "builtin": None,
            "robot": None,
            "importable": False,
            "modules": [],
            "optional_dependency": False,
            "import_error": problem,
            "import_traceback": None,
            "missing_module": None,
        }

    async def one_child(self, todo: list[str]) -> tuple[set[str], str | None]:
        """One child over `todo`: (names it answered, why it stopped early or None)."""
        request = {
            "blueprints": [n for n in todo if not n.startswith("module:")],
            "modules": [n[7:] for n in todo if n.startswith("module:")],
            "known": list(self.data["modules"]),
        }
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
        await child.stdin.drain()
        child.stdin.close()
        stderr_tail: collections.deque[str] = collections.deque(maxlen=20)

        async def drain() -> None:
            assert child.stderr
            async for line in child.stderr:
                stderr_tail.append(line.decode("utf-8", "replace").rstrip())

        draining = asyncio.create_task(drain())
        finished: set[str] = set()
        last_save = time.monotonic()
        problem: str | None = None
        try:
            while True:
                try:
                    line = await asyncio.wait_for(child.stdout.readline(), self.item_timeout)
                except asyncio.TimeoutError:
                    problem = f"no answer in {self.item_timeout:.0f} s (hung while importing)"
                    break
                if not line:
                    await child.wait()
                    tail = next((t for t in reversed(stderr_tail) if t.strip()), "no output")
                    problem = f"the import crashed its process (exit {child.returncode}): {tail}"
                    break
                text = line.decode("utf-8", "replace")
                if MARKER not in text:
                    continue
                message = json.loads(text.rpartition(MARKER)[2])
                kind = message.pop("kind", None)
                if kind == "start":
                    self.status["current"] = message["name"]
                elif kind == "blueprint":
                    self.data["blueprints"][message["name"]] = message
                    finished.add(message["name"])
                elif kind == "module":
                    self.data["modules"][message["class"]] = message
                    finished.add(f"module:{message['name']}")
                elif kind == "module-error":
                    self.data["module_errors"][message["name"]] = message
                    finished.add(f"module:{message['name']}")
                elif kind == "end":
                    break
                elif kind == "error":
                    problem = message.get("error", "the scan failed")
                    break
                self.notify()
                if time.monotonic() - last_save > SAVE_INTERVAL_S:
                    last_save = time.monotonic()
                    await self.save()
        finally:
            if child.returncode is None:
                kill(child)
                await child.wait()
            draining.cancel()
        return finished, problem

    # answers

    def answer(self) -> dict[str, Any]:
        """The data to answer from: the previous answer while a changed checkout is being scanned."""
        if self.stale_data is not None and not self.data["complete"]:
            return self.stale_data
        return self.data

    def find_module(self, name: str) -> dict[str, Any] | None:
        return lookup(self.answer()["modules"], name) or lookup(self.data["modules"], name)

    def blueprint(self, name: str) -> dict[str, Any] | None:
        record = self.answer()["blueprints"].get(name)
        return record if isinstance(record, dict) else None

    def import_status(self, entry: dict[str, Any]) -> dict[str, Any]:
        """A /dimos/blueprints entry with what the scan found: importable, import_error, missing_module (null until
        scanned)."""
        record = self.blueprint(entry["name"]) or {}
        return {
            **entry,
            "importable": record.get("importable"),
            "import_error": record.get("import_error"),
            "missing_module": record.get("missing_module"),
        }

    def records(self) -> list[dict[str, Any]]:
        """The answer's blueprints, in registry order, each with its robot from robots.json."""
        data = self.answer()
        order = data["names"]["blueprints"]
        found = [data["blueprints"][n] for n in order if n in data["blueprints"]]
        found += [r for n, r in data["blueprints"].items() if n not in order]
        return with_robots(self.dimos_dir, found)

    def blueprint_list(self) -> list[dict[str, Any]]:
        return self.records()

    def module_list(self) -> list[dict[str, Any]]:
        data = self.answer()
        stats = module_stats(self.records())
        return [
            {
                **{k: v for k, v in record.items() if k != "config"},
                "blueprint_count": stats["count"].get(record["class"], 0),
                "robots": sorted(stats["robots"].get(record["class"], ())),
            }
            for record in sorted(data["modules"].values(), key=lambda r: r["name"])
        ]

    def message_types(self) -> list[dict[str, Any]]:
        types: dict[str, dict[str, set[str]]] = {}
        for record in self.answer()["modules"].values():
            for port in record.get("outputs", []):
                types.setdefault(port["type"], {"publishers": set(), "subscribers": set()})[
                    "publishers"
                ].add(record["name"])
            for port in record.get("inputs", []):
                types.setdefault(port["type"], {"publishers": set(), "subscribers": set()})[
                    "subscribers"
                ].add(record["name"])
        return [
            {
                "type": name,
                "publishers": sorted(sides["publishers"]),
                "subscribers": sorted(sides["subscribers"]),
            }
            for name, sides in sorted(types.items())
        ]

    def robots(self) -> dict[str, list[str]]:
        """Robot -> its blueprints (importable or not), from the folder each blueprint lives in."""
        robots: dict[str, list[str]] = {}
        for record in self.records():
            if record.get("robot"):
                robots.setdefault(record["robot"], []).append(record["name"])
        return robots

    def robot_modules(self, robot: str) -> dict[str, Any] | None:
        return rank_modules(self.records(), robot)


def robots_doc(dimos_dir: Path) -> dict[str, Any] | None:
    """The checkout's dimos/server/robots.json (None when it has none or it doesn't parse)."""
    try:
        from dimos.server import robots

        return robots.load(dimos_dir / "dimos" / "server" / "robots.json")
    except Exception:
        return None


def with_robots(dimos_dir: Path, records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Each blueprint's robot from robots.json: the robot that lists it, else the one whose dirs hold its file; without
    a robots.json, the robot folder the scan found."""
    doc = robots_doc(dimos_dir)
    if doc is None:
        return records
    from dimos.server.robots import blueprint_file, owner_of

    listed = {
        name: robot_id
        for robot_id, robot in doc.get("robots", {}).items()
        for name in robot.get("blueprints", {})
    }
    return [
        {
            **record,
            "robot": listed.get(record["name"])
            or (owner_of(doc, blueprint_file(record["ref"])) if record.get("ref") else None),
        }
        for record in records
    ]


def kill(child: asyncio.subprocess.Process) -> None:
    import os
    import signal

    try:
        os.killpg(child.pid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        pass


def lookup(modules: dict[str, dict[str, Any]], name: str) -> dict[str, Any] | None:
    """A module record by registry name (`camera-module`), class name (`CameraModule`) or `pkg.mod.Class`."""
    if name in modules:
        return modules[name]
    for record in modules.values():
        if name in (record["name"], record["class"].rsplit(".", 1)[-1]):
            return record
    return None


def module_stats(blueprints: Any) -> dict[str, Any]:
    """Per module class: how many importable blueprints use it, and the robots those blueprints belong to."""
    count: dict[str, int] = {}
    robots: dict[str, set[str]] = {}
    for record in blueprints:
        if not record.get("importable"):
            continue
        for cls in {module["class"] for module in record.get("modules", [])}:
            count[cls] = count.get(cls, 0) + 1
            if record.get("robot"):
                robots.setdefault(cls, set()).add(record["robot"])
    return {"count": count, "robots": robots}


def rank_modules(blueprints: Any, robot: str) -> dict[str, Any] | None:
    """The modules of `robot`'s blueprints ranked by how specific they are to it (see FORMULA); None for no such
    robot. Only importable blueprints count (a blueprint that doesn't import has no known modules)."""
    records = list(blueprints)
    if not any(record.get("robot") == robot for record in records):
        return None
    usable = [record for record in records if record.get("importable")]
    stats = module_stats(usable)
    robots_total = len({record["robot"] for record in usable if record.get("robot")})
    mine = [record for record in usable if record.get("robot") == robot]
    names: dict[str, str] = {}
    in_robot: dict[str, int] = {}
    for record in mine:
        for module in record["modules"]:
            names.setdefault(module["class"], module.get("module") or module["name"])
        for cls in {module["class"] for module in record["modules"]}:
            in_robot[cls] = in_robot.get(cls, 0) + 1
    ranked = []
    for cls, uses in in_robot.items():
        robots_using = len(stats["robots"].get(cls, ()))
        tf = uses / len(mine)
        idf = math.log(robots_total / robots_using) if robots_using else 0.0
        ranked.append(
            {
                "name": names[cls],
                "class": cls,
                "score": round(tf * idf, 4),
                "in_robot_blueprints": uses,
                "robot_blueprints": len(mine),
                "robots_using": robots_using,
                "blueprint_count": stats["count"].get(cls, 0),
            }
        )
    ranked.sort(
        key=lambda m: (-m["score"], -m["in_robot_blueprints"], m["blueprint_count"], m["name"])
    )
    return {
        "robot": robot,
        "blueprints": sum(1 for record in records if record.get("robot") == robot),
        "blueprints_importable": len(mine),
        "robots_total": robots_total,
        "formula": FORMULA,
        "modules": ranked,
    }
