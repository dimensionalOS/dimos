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

"""Agent modules and their deployment into the running system.

Each module has one source file, ``NAME.py`` in the ``dimos_agent`` package of
``RUNTIME_MODULES_DIR``, which the agent edits. A restart snapshots it into a new revision,
``NAME_rN.pyc``, so no process runs a stale copy: the deployed revision is unloaded and
the new one runs. A module runs under its class name, and the harness finds it in the
running system by its import path.
"""

from __future__ import annotations

import builtins
from collections.abc import Callable, Iterator
from dataclasses import asdict, dataclass, replace
import importlib
import importlib.util
import json
import marshal
from pathlib import Path
import shutil
import sys
import threading
import traceback
from typing import TYPE_CHECKING, Literal

from dimos.agents.harness.runtime.agent_module import AgentModule
from dimos.agents.harness.runtime.constants import (
    AGENT_MODULE_NAME,
    AGENT_MODULE_PACKAGE,
    AGENT_MODULE_PATH,
    EXCEPTION_GROUP_MARGIN,
    JSON,
    TRACEBACK_HEADER,
)
from dimos.protocol.rpc.rpc_utils import RemoteError
from dimos.utils.safe_thread_map import ExceptionGroup

if TYPE_CHECKING:
    from dimos.porcelain.dimos import Dimos

Op = Literal["create", "restart", "unload", "delete"]
State = Literal["running", "stopped", "absent"]


@dataclass
class _Record:
    owner: int | None
    class_name: str
    last_error: str | None


@dataclass(frozen=True)
class AgentModuleStatus:
    name: str
    class_name: str
    state: Literal["running", "stopped"]
    revision: int | None
    """The revision the running system holds; None when it does not run the module."""
    owner: int | None
    """The conversation that created the module."""
    last_error: str | None
    """Why the last create or restart failed, until one succeeds."""
    path: str
    """The module's source file, which the agent edits."""

    def to_json(self) -> dict[str, JSON]:
        return {
            "name": self.name,
            "class": self.class_name,
            "state": self.state,
            "revision": self.revision,
            "owner": self.owner,
            "last_error": self.last_error,
            "path": self.path,
        }


@dataclass(frozen=True)
class AgentModuleOutcome:
    """What an op did to one module, as the agent reads it. Never the source."""

    name: str
    op: Op
    ok: bool
    state: State
    """The module's state after the op; absent after a delete or a create that failed
    before the source imported."""
    path: str
    """The module's source file."""
    revision: int | None
    """The revision running after the op."""
    error: str | None
    """Why the op failed, in one line."""
    traceback: str | None
    """The failure's traceback, from the module's worker when it failed there."""

    def text(self) -> str:
        if self.ok and self.op == "delete":
            return f"{self.name} deleted, with its source {self.path}"
        match self.state:
            case "running":
                now = f"running r{self.revision} from {self.path}"
            case "stopped":
                now = f"not running; its source is {self.path}"
            case "absent":
                now = f"not created; its source is {self.path}"
        if self.ok:
            done = {"create": "created", "restart": "restarted", "unload": "unloaded"}[self.op]
            return f"{self.name} {done}: {now}"
        return f"{self.name} {self.op} failed: {self.error}\nNow {now}."

    def to_json(self) -> dict[str, JSON]:
        return {
            "name": self.name,
            "op": self.op,
            "ok": self.ok,
            "state": self.state,
            "path": self.path,
            "revision": self.revision,
            "error": self.error,
            "traceback": self.traceback,
        }


def _revision(name: str, module: str) -> int | None:
    match = AGENT_MODULE_PATH.match(module)
    return int(match["revision"]) if match and match["name"] == name else None


def client_gone(error: BaseException) -> bool:
    """Whether error is the Dimos client's own: stopped, or no coordinator to connect to.
    An error the coordinator sent back has a RemoteError as its cause."""
    return isinstance(error, RuntimeError) and not isinstance(error.__cause__, RemoteError)


def _describe(error: BaseException) -> str:
    if isinstance(error, ExceptionGroup):
        return "; ".join(_describe(inner) for inner in error.exceptions)
    return f"{type(error).__name__}: {error}"


def _line(error: BaseException) -> str:
    return _describe(error).splitlines()[0]


def _traceback(error: BaseException, source: Path) -> str | None:
    frame = error.__traceback__
    while frame is not None and frame.tb_frame.f_code.co_filename != str(source):
        frame = frame.tb_next
    if frame is None and not isinstance(error, SyntaxError):
        return None
    return "".join(traceback.format_exception(type(error), error, frame)).rstrip()


def _texts(error: BaseException) -> Iterator[str]:
    yield str(error)
    if isinstance(error, RemoteError):
        yield error.remote_traceback
    if isinstance(error, ExceptionGroup):
        for inner in error.exceptions:
            yield from _texts(inner)
    if error.__cause__ is not None:
        yield from _texts(error.__cause__)


def _traceback_in(text: str, source: Path) -> str | None:
    lines = [EXCEPTION_GROUP_MARGIN.sub("", line) for line in text.splitlines()]
    frame = f'  File "{source}", '
    for start, line in enumerate(lines):
        if line.startswith(frame):
            end = next(
                (index for index in range(start, len(lines)) if not lines[index].startswith(" ")),
                len(lines) - 1,
            )
            return "\n".join([TRACEBACK_HEADER, *lines[start : end + 1]])
    return None


def _remote_failure(error: BaseException, source: Path) -> tuple[str, str | None]:
    for text in _texts(error):
        trace = _traceback_in(text, source)
        if trace is not None:
            return trace.splitlines()[-1], trace
    return _line(error), None


def deployed_revisions(modules: list[tuple[str, str | None]]) -> dict[str, int]:
    """Agent module name -> deployed revision, from the (instance name, qualified class
    path) of each running module."""
    deployed = {}
    for _, path in modules:
        match = AGENT_MODULE_PATH.match((path or "").rpartition(".")[0])
        if match:
            deployed[match["name"]] = int(match["revision"])
    return deployed


class AgentModuleManager:
    def __init__(
        self,
        root: Path,
        package_dir: Path,
        *,
        connect: Callable[[], Dimos],
        reconnect: Callable[[], None],
    ) -> None:
        """root keeps metadata; package_dir is RUNTIME_MODULES_DIR/dimos_agent."""
        self._root = root
        self._package_dir = package_dir
        self._connect = connect
        self._reconnect = reconnect
        self._lock = threading.RLock()
        root.mkdir(parents=True, exist_ok=True)
        package_dir.mkdir(parents=True, exist_ok=True)
        (package_dir / "__init__.py").touch()
        if str(package_dir.parent) not in sys.path:
            sys.path.append(str(package_dir.parent))

    def _dir(self, name: str) -> Path:
        if not AGENT_MODULE_NAME.match(name):
            raise ValueError(
                "Agent module names start with a lowercase letter, use lowercase letters, "
                "digits and '_' (max 64 characters), and do not end in _r and digits"
            )
        return self._root / name

    def source(self, name: str) -> Path:
        """The module's source file, which the agent edits."""
        self._dir(name)
        return self._package_dir / f"{name}.py"

    def _record(self, name: str) -> _Record:
        path = self._dir(name) / "module.json"
        if not path.exists():
            raise KeyError(f"No agent module named {name!r}")
        saved = json.loads(path.read_text())
        return _Record(
            owner=saved["owner"],
            class_name=saved["class_name"],
            last_error=saved["last_error"],
        )

    def _save(self, name: str, record: _Record) -> None:
        directory = self._dir(name)
        directory.mkdir(parents=True, exist_ok=True)
        pending = directory / "module.json.tmp"
        pending.write_text(json.dumps(asdict(record), indent=2))
        pending.replace(directory / "module.json")

    def _revision_files(self, name: str) -> dict[Path, int]:
        return {
            path: revision
            for path in self._package_dir.glob(f"{name}_r*.pyc")
            if (revision := _revision(name, f"{AGENT_MODULE_PACKAGE}.{path.stem}")) is not None
        }

    def _remove_revision(self, path: Path) -> None:
        path.unlink(missing_ok=True)
        sys.modules.pop(f"{AGENT_MODULE_PACKAGE}.{path.stem}", None)

    def _next_revision(self, name: str) -> int:
        """Revisions are never reused, even after delete: a running system may have
        imported an earlier file with the same name."""
        path = self._root / "revisions.json"
        revisions: dict[str, int] = json.loads(path.read_text()) if path.exists() else {}
        revisions[name] = revisions.get(name, 0) + 1
        path.write_text(json.dumps(revisions, indent=2))
        return revisions[name]

    def _snapshot(self, name: str) -> type[AgentModule]:
        source = self.source(name)
        if not source.exists():
            raise FileNotFoundError(f"{source} does not exist; write the module's source there")
        code = compile(source.read_bytes(), str(source), "exec", dont_inherit=True)
        revision = self._next_revision(name)
        # Sourceless, and compiled under the source's name: tracebacks name the file the
        # agent edits, and there is no revision source to edit by mistake.
        path = self._package_dir / f"{name}_r{revision}.pyc"
        path.write_bytes(importlib.util.MAGIC_NUMBER + bytes(12) + marshal.dumps(code))
        try:
            return self._load(name, revision)
        except BaseException:
            self._remove_revision(path)
            raise

    def _load(self, name: str, revision: int) -> type[AgentModule]:
        importlib.invalidate_caches()
        module = importlib.import_module(f"{AGENT_MODULE_PACKAGE}.{name}_r{revision}")
        classes = [
            value
            for value in vars(module).values()
            if isinstance(value, type)
            and issubclass(value, AgentModule)
            and value.__module__ == module.__name__
        ]
        if len(classes) != 1:
            raise ValueError(
                "An agent module source defines exactly one subclass of "
                f"dimos.agents.harness.runtime.agent_module.AgentModule; found {len(classes)}"
            )
        return classes[0]

    def _deployed_in(self, app: Dimos) -> dict[str, int]:
        return deployed_revisions([(info.name, info.qualified_path) for info in app.list_modules()])

    def _unload_in(self, app: Dimos, name: str) -> None:
        revision = self._deployed_in(app).get(name)
        if revision is None:
            return
        cls = self._load(name, revision)
        app.unload(cls)

    def deployed(self) -> dict[str, int]:
        """From the running system: reconnect once if the client stopped or found no
        coordinator. Raises if DimOS does not answer."""
        try:
            return self._deployed_in(self._connect())
        except Exception as error:
            if not client_gone(error):
                raise
            self._reconnect()
            return self._deployed_in(self._connect())

    def _deployed_or_none(self) -> dict[str, int]:
        try:
            return self.deployed()
        except Exception:
            return {}

    def _outcome(
        self,
        name: str,
        op: Op,
        error: str | None = None,
        trace: str | None = None,
        *,
        absent: bool = False,
    ) -> AgentModuleOutcome:
        revision = None if absent else self._deployed_or_none().get(name)
        return AgentModuleOutcome(
            name=name,
            op=op,
            ok=error is None,
            state="absent" if absent else "stopped" if revision is None else "running",
            path=str(self.source(name)),
            revision=revision,
            error=error,
            traceback=trace,
        )

    def status(self, name: str, deployed: dict[str, int] | None = None) -> AgentModuleStatus:
        """deployed comes from the running system; None asks it."""
        record = self._record(name)
        revision = (self._deployed_or_none() if deployed is None else deployed).get(name)
        return AgentModuleStatus(
            name=name,
            class_name=record.class_name,
            state="stopped" if revision is None else "running",
            revision=revision,
            owner=record.owner,
            last_error=record.last_error,
            path=str(self.source(name)),
        )

    def names(self) -> list[str]:
        return [
            path.name for path in sorted(self._root.iterdir()) if (path / "module.json").exists()
        ]

    def list(self, deployed: dict[str, int] | None = None) -> builtins.list[AgentModuleStatus]:
        deployed = self._deployed_or_none() if deployed is None else deployed
        return [self.status(name, deployed) for name in self.names()]

    def create(self, name: str, path: Path | None, *, owner: int | None) -> AgentModuleOutcome:
        """Copy path to the module's source file, unless path is that file, and start it.
        Without path, the source file must exist."""
        with self._lock:
            source = self.source(name)
            if (self._dir(name) / "module.json").exists():
                raise ValueError(f"Agent module {name!r} exists; edit {source} and restart it")
            if path is not None and not path.is_absolute():
                raise ValueError(f"path must be absolute: {path}")
            if path is not None and path.resolve() != source.resolve():
                shutil.copyfile(path, source)
            if not source.exists():
                raise ValueError(f"create needs the module's source: write {source}, or pass path")
            try:
                cls = self._snapshot(name)
            except Exception as error:
                trace = _traceback(error, source)
                return self._outcome(name, "create", _line(error), trace, absent=True)
            record = _Record(
                owner=owner,
                class_name=cls.__name__,
                last_error=None,
            )
            return self._deploy(name, record, cls, "create")

    def restart(self, name: str) -> AgentModuleOutcome:
        """Run the source file's current code. A source that does not import leaves the
        running revision running."""
        with self._lock:
            record = self._record(name)
            try:
                cls = self._snapshot(name)
            except Exception as error:
                record.last_error = _line(error)
                self._save(name, record)
                trace = _traceback(error, self.source(name))
                return self._outcome(name, "restart", record.last_error, trace)
            return self._deploy(name, record, cls, "restart")

    def _deploy(
        self, name: str, record: _Record, cls: type[AgentModule], op: Op
    ) -> AgentModuleOutcome:
        record.class_name = cls.__name__
        self._save(name, record)
        try:
            app = self._connect()
            self._unload_in(app, name)
            # Unload goes by class name, so two running modules cannot share one.
            taken = [info.name for info in app.list_modules() if info.class_name == cls.__name__]
            if taken:
                raise ValueError(
                    f"{', '.join(taken)} already runs with the class name {cls.__name__}; "
                    "rename the class"
                )
            try:
                # A blueprint, not the class: run() would load a registered module with
                # the same class name instead.
                app.run(cls.blueprint())
            except BaseException:
                # A module whose start() failed is still deployed.
                self._unload_in(app, name)
                raise
        except Exception as error:
            record.last_error, trace = _remote_failure(error, self.source(name))
            self._save(name, record)
            self._prune(name)
            return self._outcome(name, op, record.last_error, trace)
        record.last_error = None
        self._save(name, record)
        self._prune(name)
        return self._outcome(name, op)

    def unload(self, name: str) -> AgentModuleOutcome:
        """Stop the module."""
        with self._lock:
            self._record(name)
            try:
                self._unload_in(self._connect(), name)
            except Exception as error:
                line, trace = _remote_failure(error, self.source(name))
                return self._outcome(name, "unload", line, trace)
            self._prune(name)
        return self._outcome(name, "unload")

    def delete(self, name: str) -> AgentModuleOutcome:
        """Unload the module and remove it with its source file."""
        with self._lock:
            self._record(name)
            if name in self.deployed():
                unloaded = self.unload(name)
                if not unloaded.ok:
                    return replace(unloaded, op="delete")
            for path in self._revision_files(name):
                self._remove_revision(path)
            self.source(name).unlink(missing_ok=True)
            shutil.rmtree(self._dir(name))
        return self._outcome(name, "delete", absent=True)

    def _prune(self, name: str) -> None:
        try:
            deployed = self.deployed().get(name)
        except Exception:
            return
        for path, revision in self._revision_files(name).items():
            if revision != deployed:
                self._remove_revision(path)

    def owner(self, name: str) -> int | None:
        """The conversation that created the module."""
        return self._record(name).owner
