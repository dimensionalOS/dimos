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


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


class Scanner:
    def __init__(self, dimos_dir: Path, command: list[str] | None = None) -> None:
        self.dimos_dir = dimos_dir
        self.command = command or child_command(dimos_dir)
        self.blueprints: dict[str, dict[str, Any]] = {}
        self.modules: dict[str, dict[str, Any]] = {}
        self.errors: list[str] = []
        self.names: dict[str, list[str]] = {"blueprints": [], "modules": []}
        self.finished_modules: set[str] = set()
        self.task: asyncio.Task[None] | None = None
        self.status: dict[str, Any] = {
            "state": "idle",
            "reason": None,
            "key": None,
            "stale": False,
            "started_at": None,
            "finished_at": None,
            "current": None,
            "cached_from": None,
        }

    def snapshot(self) -> dict[str, Any]:
        importable = sum(1 for r in self.blueprints.values() if r["importable"])
        return {
            **self.status,
            "blueprints_total": len(self.names["blueprints"]) or len(self.blueprints),
            "blueprints_done": len(self.blueprints),
            "importable": importable,
            "not_importable": len(self.blueprints) - importable,
            "modules_total": len(self.names["modules"]),
            "modules_done": len(self.finished_modules & set(self.names["modules"])),
            "errors": self.errors[-MAX_ERRORS:],
        }

    def ensure(self, reason: str = "requested") -> None:
        if self.task is None:
            self.refresh(reason)

    def refresh(self, reason: str = "requested") -> None:
        self.invalidate()
        self.status.update(state="scanning", reason=reason)
        self.task = asyncio.get_running_loop().create_task(self.scan(reason))

    def invalidate(self) -> None:
        if self.task is not None:
            self.task.cancel()
            self.status["state"] = "idle"
        self.task = None

    async def wait(self) -> None:
        while True:
            self.ensure()
            task = self.task
            assert task is not None
            await asyncio.wait({task})
            if task is self.task:
                return

    async def scan(self, reason: str) -> None:
        self.blueprints, self.modules, self.errors = {}, {}, []
        self.names, self.finished_modules = {"blueprints": [], "modules": []}, set()
        self.status.update(
            state="scanning",
            reason=reason,
            key=await asyncio.to_thread(packages_key),
            started_at=now_iso(),
            finished_at=None,
            current=None,
        )
        try:
            request: dict[str, Any] = {}
            while True:
                problem = await self.child(request)
                remaining = [n for n in self.names["blueprints"] if n not in self.blueprints]
                modules = [n for n in self.names["modules"] if n not in self.finished_modules]
                if problem is None or not (remaining or modules):
                    break
                current = self.status["current"]
                self.errors.append(f"{current}: {problem}")
                if current in remaining:
                    self.blueprints[current] = failed_record(current, problem)
                    remaining.remove(current)
                elif current and current.startswith("module:"):
                    self.finished_modules.add(current[7:])
                    modules = [m for m in modules if m != current[7:]]
                else:
                    break
                request = {"blueprints": remaining, "modules": modules, "known": list(self.modules)}
            self.status.update(state="done")
        except Exception as error:
            logger.exception("the discovery scan failed")
            self.errors.append(f"scan: {type(error).__name__}: {error}")
            self.status.update(state="failed")
        finally:
            self.status.update(finished_at=now_iso(), current=None)

    async def child(self, request: dict[str, Any]) -> str | None:
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
        child.stdin.write(json.dumps(request or None).encode())
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
            self.status["current"] = message["name"]
        elif kind == "blueprint":
            self.blueprints[message["name"]] = message
        elif kind == "module":
            self.modules[message["class"]] = message
            self.finished_modules.add(message["name"])
        elif kind == "error" or "error" in message:
            self.errors.append(str(message.get("error")))
            current = str(self.status["current"] or "")
            if current.startswith("module:"):
                self.finished_modules.add(current[7:])
        return kind == "end" or (kind is None and "error" in message)

    def records(self) -> list[dict[str, Any]]:
        doc = robots.load()
        order = {name: index for index, name in enumerate(self.names["blueprints"])}
        found = sorted(self.blueprints.values(), key=lambda r: order.get(r["name"], len(order)))
        return [{**r, "robot": robots.robot_of(doc, r["name"], r["ref"])} for r in found]

    def import_status(self, entry: dict[str, Any]) -> dict[str, Any]:
        record = self.blueprints.get(entry["name"]) or {}
        return {
            **entry,
            "importable": record.get("importable"),
            "import_error": record.get("import_error"),
            "missing_module": record.get("missing_module"),
        }

    def discovered(self) -> dict[str, Any]:
        providers = providing_extras(self.dimos_dir)
        blueprints = [
            {
                **{k: v for k, v in record.items() if k != "doc"},
                "suggested_extras": providers(record.get("missing_module")),
            }
            for record in self.records()
        ]
        return {"stale": False, "blueprints": blueprints}

    def catalog(self) -> dict[str, Any]:
        records = self.records()
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
        for record in sorted(self.modules.values(), key=lambda m: m["name"]):
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
        errors += [e for e in self.errors if e.startswith("module ")]
        if len(errors) > MAX_ERRORS:
            errors = [*errors[:MAX_ERRORS], f"... and {len(errors) - MAX_ERRORS} more"]
        return {"blueprints": blueprints, "modules": modules, "skills": skills, "errors": errors}


def failed_record(name: str, problem: str) -> dict[str, Any]:
    return {
        "name": name,
        "ref": None,
        "builtin": None,
        "importable": False,
        "optional_dependency": False,
        "import_error": problem,
        "import_traceback": None,
        "missing_module": None,
        "modules": [],
    }


def packages_key() -> str:
    from importlib.metadata import distributions

    names = sorted(f"{d.metadata['Name']}=={d.version}" for d in distributions())
    return hashlib.sha256("\n".join(names).encode()).hexdigest()[:16]


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


def extras_status(dimos_dir: Path, probe: dict[str, Any]) -> list[dict[str, Any]]:
    environment, installed = probe.get("environment", {}), probe.get("packages", {})
    extras = declared(dimos_dir, probe)
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
            elif applies(requirement, environment, name):
                requirements.append(requirement)
        parsed[name] = (requirements, includes)

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

    return [
        {
            "name": name,
            "installed": not missing(name),
            "applicable": bool(parsed[name][0] or parsed[name][1]),
            "requires": extras[name],
            "includes": parsed[name][1],
            "missing": missing(name),
            "download_bytes": None,
        }
        for name in extras
    ]


def lock_closures(dimos_dir: Path) -> dict[str, set[str]]:
    try:
        lock = tomllib.loads((dimos_dir / "uv.lock").read_text())
    except (OSError, tomllib.TOMLDecodeError):
        return {}
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
    extras = declared(dimos_dir, {})
    closures = lock_closures(dimos_dir)
    direct: dict[str, set[str]] = {}
    includes: dict[str, set[str]] = {}
    for extra, texts in extras.items():
        direct[extra], includes[extra] = set(), set()
        for text in texts:
            try:
                requirement = Requirement(text)
            except InvalidRequirement:
                continue
            if canonicalize_name(requirement.name) == "dimos":
                includes[extra] |= set(requirement.extras)
            else:
                direct[extra].add(canonicalize_name(requirement.name))

    def contains(extra: str, other: str, seen: frozenset[str] = frozenset()) -> bool:
        inner = includes.get(extra, set()) - seen
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
        return [
            uv,
            "sync",
            "--locked",
            "--inexact",
            "--no-progress",
            *(f"--extra={e}" for e in extras),
        ]
    version = store.checkout_version(dimos_dir)[1]
    return [
        uv,
        "pip",
        "install",
        "--no-progress",
        "--python",
        store.python_for(dimos_dir),
        "--torch-backend",
        "cu128" if "cuda" in extras else "cpu",
        f"dimos[{','.join(extras)}]" + (f"=={version}" if version else ""),
    ]


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

    def log(self, after: int = 0) -> dict[str, Any]:
        return {
            "job": self.id,
            "title": self.title,
            "kind": self.kind,
            "done": self.done,
            "ok": self.ok,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "command": self.command,
            "lines": self.lines[max(after, 0) :],
            "next": len(self.lines),
            "error": self.error,
            "failure": self.failure,
            "code": None,
        }


class Jobs:
    def __init__(self) -> None:
        self.jobs: dict[str, Job] = {}
        self.ids = itertools.count(1)

    def get(self, job_id: str) -> Job:
        cutoff = time.monotonic() - JOB_KEEP_S
        self.jobs = {k: j for k, j in self.jobs.items() if j.ended is None or j.ended >= cutoff}
        if job_id not in self.jobs:
            raise KeyError(f"no job {job_id}")
        return self.jobs[job_id]

    def running(self, kind: str) -> Job | None:
        return next((j for j in self.jobs.values() if j.kind == kind and not j.done), None)

    def start(
        self,
        title: str,
        kind: str,
        command: list[str],
        cwd: Path,
        env: dict[str, str],
        then: Callable[[], None],
    ) -> Job:
        job = Job(f"{kind}-{next(self.ids)}-{int(time.time())}", title, kind, command)
        self.jobs[job.id] = job
        asyncio.get_running_loop().create_task(self.run(job, cwd, env, then))
        return job

    async def run(self, job: Job, cwd: Path, env: dict[str, str], then: Callable[[], None]) -> None:
        job.lines.append("$ " + " ".join(job.command))
        try:
            process = await asyncio.create_subprocess_exec(
                *job.command,
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
            code = await process.wait()
            job.ok = code == 0
            if code != 0:
                job.error = f"{job.title} failed (exit {code})"
                job.failure = [line for line in job.lines if line.strip()][-15:]
        except Exception as error:
            job.ok = False
            job.error = f"{job.title} couldn't start: {type(error).__name__}: {error}"
            job.failure = [job.error]
        job.done, job.finished_at, job.ended = True, now_iso(), time.monotonic()
        then()


LINK = re.compile(r"(!?\[[^\]]*\]\()([^)\s]+)(\s+\"[^\"]*\")?\)")
CUSTOM_ROBOT = (
    r"(custom|new|own|your)[ _-]+(robot|platform|hardware)",
    r"(adding|add|integrat\w*)[ _-]+(a[ _-]+)?(custom|new)",
    r"(custom|new)[ _-]+\w+[ _-]+(robot|arm|platform)",
)


class _AnyTagLoader(yaml.SafeLoader):
    pass


_AnyTagLoader.add_multi_constructor(  # type: ignore[no-untyped-call]
    "", lambda loader, suffix, node: None
)


def docs_site(dimos_dir: Path) -> tuple[str | None, str | None]:
    try:
        settings = yaml.load((dimos_dir / "mkdocs.yml").read_text(), Loader=_AnyTagLoader)
    except (OSError, yaml.YAMLError):
        return None, None
    settings = settings if isinstance(settings, dict) else {}
    url, repo = settings.get("site_url"), settings.get("repo_url")
    return (str(url).rstrip("/") + "/" if url else None, str(repo) if repo else None)


def doc_title(path: Path) -> str:
    try:
        heading = next((l[2:] for l in path.read_text().splitlines() if l.startswith("# ")), None)
    except OSError:
        heading = None
    return heading.strip() if heading else path.stem.replace("_", " ")


def custom_robot(dimos_dir: Path) -> dict[str, Any] | None:
    root = dimos_dir / "docs"
    base, repo = docs_site(dimos_dir)
    found: list[tuple[int, int, Path]] = []
    for path in sorted(root.rglob("*.md")):
        if any(part.startswith(".") for part in path.relative_to(root).parts):
            continue
        names = f"{path.stem} {doc_title(path)}".lower()
        rank = next((i for i, p in enumerate(CUSTOM_ROBOT) if re.search(p, names)), None)
        if rank is not None:
            found.append((rank, len(path.parts), path))
    pages = [path for _, _, path in sorted(found)]
    if not pages:
        return None

    def page_url(path: Path) -> str | None:
        parts = list(path.relative_to(root).with_suffix("").parts)
        if parts and parts[-1] in ("index", "README"):
            parts = parts[:-1]
        return base + "".join(f"{part}/" for part in parts) if base else None

    def fix(match: re.Match[str], page: Path) -> str:
        opener, target, label = match.group(1), match.group(2), match.group(3) or ""
        if re.match(r"^[a-z][a-z0-9+.-]*:|^#", target):
            return match.group(0)
        file, _, anchor = target.partition("#")
        resolved = (
            dimos_dir / file.lstrip("/") if file.startswith("/") else page.parent / file
        ).resolve()
        url = None
        if resolved.is_relative_to(root.resolve()) and base:
            inside = root / resolved.relative_to(root.resolve())
            url = (
                page_url(inside)
                if resolved.suffix == ".md"
                else base + inside.relative_to(root).as_posix()
            )
        elif repo and resolved.is_relative_to(dimos_dir.resolve()):
            url = f"{repo.rstrip('/')}/blob/main/{resolved.relative_to(dimos_dir.resolve()).as_posix()}"
        return f"{opener}{url}{'#' + anchor if anchor else ''}{label})" if url else match.group(0)

    page = pages[0]
    markdown = LINK.sub(lambda m: fix(m, page), page.read_text())
    try:
        from markdown_it import MarkdownIt

        html: str | None = str(
            MarkdownIt("commonmark", {"html": False}).enable("table").render(markdown)
        )
    except ImportError:
        html = None
    return {
        "title": doc_title(page),
        "markdown": markdown,
        "html": html,
        "source_path": page.relative_to(dimos_dir).as_posix(),
        "url": page_url(page),
        "others": [
            {
                "title": doc_title(p),
                "source_path": p.relative_to(dimos_dir).as_posix(),
                "url": page_url(p),
            }
            for p in pages[1:]
        ],
    }
