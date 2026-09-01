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

"""Run a concrete Module subclass in an isolated sibling Python project."""

from __future__ import annotations

import inspect
import os
from pathlib import Path
import pickle
import select
import subprocess
import threading
import time
from typing import Any, ClassVar

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.utils.generic import short_id
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def isolated_python_run_command(project: Path, *command: str) -> list[str]:
    """Run a command with the host DimOS available in an isolated project."""
    args = ["uv", "run"]
    if (project / "uv.lock").is_file():
        args.append("--frozen")
    if (DIMOS_PROJECT_ROOT / "pyproject.toml").is_file():
        args.extend(("--with-editable", str(DIMOS_PROJECT_ROOT)))
    else:
        # Installed hosts intentionally accept the newest compatible DimOS.
        args.extend(("--with", "dimos"))
    args.extend(command)
    if (project / "pixi.toml").is_file():
        return ["pixi", "run", "--executable", *args]
    return args


class IsolatedPythonModuleConfig(NativeModuleConfig):
    """Process settings for an isolated Python module."""

    # Isolated Python modules resolve their real command from the sibling project.
    executable: str = "uv"
    startup_timeout: float = 30.0
    output_limit: int = 64 * 1024


def contract_rpc_names(module_class: type[IsolatedPythonModule]) -> frozenset[str]:
    """Return RPC names introduced below IsolatedPythonModule in the MRO."""

    framework = set(IsolatedPythonModule.rpcs)
    return frozenset(set(module_class.rpcs) - framework)


class IsolatedPythonModule(NativeModule):
    """A host RPC contract implemented by an isolated Python subclass.

    Contract classes set :attr:`implementation` to an import reference in a
    sibling ``python/`` project. Calls to RPCs introduced by the contract are
    forwarded to the concrete runtime subclass. Framework and lifecycle RPCs
    remain on the host facade.
    """

    config: IsolatedPythonModuleConfig
    implementation: ClassVar[str]

    _isolated_python_runtime: bool
    _runtime_client: RPCClient | None
    _runtime_name: str | None
    _module_refs: dict[str, RPCClient]

    def __init__(self, _isolated_python_runtime: bool = False, **kwargs: Any) -> None:
        self._isolated_python_runtime = _isolated_python_runtime
        self._runtime_client = None
        self._runtime_name = None
        self._module_refs = {}
        super().__init__(**kwargs)

    def __getattribute__(self, name: str) -> Any:
        if not name.startswith("_"):
            runtime_side = object.__getattribute__(self, "__dict__").get(
                "_isolated_python_runtime", False
            )
            if not runtime_side and name in contract_rpc_names(type(self)):
                client = object.__getattribute__(self, "__dict__").get("_runtime_client")
                if client is None:
                    raise RuntimeError(
                        f"{type(self).__name__} runtime is not ready; call build() first"
                    )
                return getattr(client, name)
        return super().__getattribute__(name)

    @property
    def runtime_project(self) -> Path:
        source = Path(inspect.getfile(type(self))).resolve()
        project = source.parent / "python"
        if not project.is_dir():
            raise FileNotFoundError(
                f"Isolated Python runtime project is missing: {project}; "
                "create a sibling 'python/' directory"
            )
        if not (project / "pyproject.toml").is_file():
            raise FileNotFoundError(
                f"Isolated Python runtime manifest is missing: {project / 'pyproject.toml'}"
            )
        return project

    def _uv_command(self, *args: str) -> list[str]:
        command = ["uv", *args]
        if (self.runtime_project / "pixi.toml").is_file():
            return ["pixi", "run", "--executable", *command]
        return command

    def _prepare_command(self) -> list[str]:
        args = ["sync"]
        if (self.runtime_project / "uv.lock").is_file():
            args.append("--frozen")
        return self._uv_command(*args)

    def _launch_command(self, handshake_fd: int) -> list[str]:
        return isolated_python_run_command(
            self.runtime_project,
            "python",
            "-m",
            "dimos.core.isolated_python_bootstrap",
            "--declaration",
            f"{type(self).__module__}:{type(self).__name__}",
            "--implementation",
            self.implementation,
            "--instance-name",
            self._new_runtime_name(),
            "--handshake-fd",
            str(handshake_fd),
        )

    def _new_runtime_name(self) -> str:
        public_name = self.config.instance_name or type(self).__name__
        self._runtime_name = f"__isolated_python__/{public_name}/{short_id()}"
        return self._runtime_name

    def _runtime_env(self) -> dict[str, str]:
        env = dict(os.environ)
        env.pop("VIRTUAL_ENV", None)
        env.update(self.config.extra_env)
        return env

    def _run_prepare(self) -> None:
        command = self._prepare_command()
        result = subprocess.run(
            command,
            cwd=self.runtime_project,
            env=self._runtime_env(),
            capture_output=True,
            text=True,
        )
        if result.returncode:
            output = (result.stdout + "\n" + result.stderr).strip()
            raise RuntimeError(
                f"Isolated Python environment preparation failed (exit {result.returncode}): "
                f"{output[-self.config.output_limit :]}"
            )

    def _spawn_runtime(self) -> None:
        parent_read, child_write = os.pipe()
        os.set_inheritable(child_write, True)
        try:
            command = self._launch_command(child_write)
            logger.info(
                "Starting isolated Python runtime",
                module=type(self).__name__,
                command=" ".join(command),
                cwd=str(self.runtime_project),
            )
            self._process = subprocess.Popen(
                command,
                cwd=self.runtime_project,
                env=self._runtime_env(),
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                pass_fds=(child_write,),
                start_new_session=True,
            )
            os.close(child_write)
            child_write = -1

            assert self._process.stdin is not None
            kwargs = self.config.model_dump()
            kwargs["instance_name"] = self._runtime_name
            pickle.dump(kwargs, self._process.stdin)
            self._process.stdin.close()

            watchdog = threading.Thread(
                target=self._watch_process,
                daemon=True,
                name=f"isolated-python-watchdog-{type(self).__name__}",
            )
            with self._stop_lock:
                self._stopping = False
                self._watchdog = watchdog
            watchdog.start()

            deadline = time.monotonic() + self.config.startup_timeout
            with os.fdopen(parent_read, "rb") as ready:
                parent_read = -1
                while time.monotonic() < deadline:
                    if select.select([ready], [], [], 0.1)[0]:
                        message = ready.readline().decode(errors="replace").strip()
                        if message == "READY":
                            return
                        raise RuntimeError(f"Isolated Python runtime failed to start: {message}")
                    if self._process.poll() is not None:
                        break
            raise RuntimeError("Isolated Python runtime exited before becoming ready")
        except BaseException:
            if self._process is not None:
                self.stop()
            raise
        finally:
            if parent_read >= 0:
                os.close(parent_read)
            if child_write >= 0:
                os.close(child_write)

    def _connect_runtime(self) -> None:
        if self._runtime_name is None:
            raise RuntimeError("Isolated Python runtime did not choose an RPC name")
        self._runtime_client = RPCClient.remote(type(self), remote_name=self._runtime_name)
        for name, stream in {**self.inputs, **self.outputs, **self.ios}.items():
            transport = getattr(stream, "_transport", None)
            if transport is not None:
                self._runtime_client.set_transport(name, transport)
        for name, module_ref in self._module_refs.items():
            self._runtime_client.set_module_ref(name, module_ref)

    def _maybe_build(self) -> None:
        if not self._isolated_python_runtime:
            self._run_prepare()

    @rpc
    def build(self) -> None:
        super().build()
        if self._isolated_python_runtime:
            return
        self._spawn_runtime()
        self._connect_runtime()
        assert self._runtime_client is not None
        self._runtime_client.build()

    @rpc
    def start(self) -> None:
        # NativeModule.start() would launch a second process through its generic
        # executable path. This runtime is already running after build().
        if self._isolated_python_runtime:
            Module.start(self)
            return
        Module.start(self)
        if self._runtime_client is None:
            raise RuntimeError(f"{type(self).__name__} was not built")
        self._runtime_client.start()

    @rpc
    def set_transport(self, stream_name: str, transport: Any) -> bool:
        result = Module.set_transport(self, stream_name, transport)
        if not self._isolated_python_runtime and self._runtime_client is not None:
            self._runtime_client.set_transport(stream_name, transport)
        return result

    @rpc
    def set_module_ref(self, name: str, module_ref: RPCClient) -> None:
        Module.set_module_ref(self, name, module_ref)
        if self._isolated_python_runtime:
            return
        self._module_refs[name] = module_ref
        if self._runtime_client is not None:
            self._runtime_client.set_module_ref(name, module_ref)

    @rpc
    def stop(self) -> None:
        if self._isolated_python_runtime:
            Module.stop(self)
            return
        client = self._runtime_client
        self._runtime_client = None
        if client is not None:
            try:
                client.stop()
            except Exception:
                logger.exception("Failed to stop isolated Python runtime module")
            finally:
                client.stop_rpc_client()
        NativeModule.stop(self)
