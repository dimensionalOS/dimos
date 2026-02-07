# Copyright 2025-2026 Dimensional Inc.
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

import argparse
from contextlib import suppress
from dataclasses import dataclass, field
import importlib
import json
import os
import signal
import subprocess
import threading
import time
from typing import TYPE_CHECKING, Any

from dimos.core.docker_build import build_image, image_exists
from dimos.core.module import Module, ModuleConfig
from dimos.core.rpc_client import RpcCall
from dimos.dashboard.rerun_init import RERUN_GRPC_PORT, RERUN_WEB_PORT
from dimos.protocol.rpc import LCMRPC
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable
    from pathlib import Path

logger = setup_logger()

DOCKER_RUN_TIMEOUT = 120  #     Timeout for `docker run` command execution
DOCKER_CMD_TIMEOUT = 20  #       Timeout for quick Docker commands (inspect, rm, logs)
DOCKER_STATUS_TIMEOUT = 10  #    Timeout for container status checks
DOCKER_STOP_TIMEOUT = 30  #      Timeout for `docker stop` command (graceful shutdown)
RPC_READY_TIMEOUT = 3.0  #       Timeout for RPC readiness probe during container startup
LOG_TAIL_LINES = 200  #          Number of log lines to include in error messages


@dataclass(kw_only=True)
class DockerModuleConfig(ModuleConfig):
    """
    Configuration for running a DimOS module inside Docker.

    For advanced Docker options not listed here, use docker_extra_args.
    Example: docker_extra_args=["--cap-add=SYS_ADMIN", "--read-only"]
    """

    # Build / image
    docker_image: str
    docker_file: Path | None = None  # Required on host for building, not needed in container
    docker_build_context: Path | None = None
    docker_build_args: dict[str, str] = field(default_factory=dict)

    # Identity
    docker_container_name: str | None = None
    docker_labels: dict[str, str] = field(default_factory=dict)

    # Networking (host mode recommended for LCM multicast)
    docker_network_mode: str = "host"
    docker_network: str | None = None
    docker_ports: list[tuple[int, int, str]] = field(
        default_factory=list
    )  # (host, container, proto)

    # Runtime resources
    docker_gpus: str | None = "all"
    docker_shm_size: str = "2g"
    docker_restart_policy: str = "on-failure:3"

    # Env + volumes + devices
    docker_env_files: list[str] = field(default_factory=list)
    docker_env: dict[str, str] = field(default_factory=dict)
    docker_volumes: list[tuple[str, str, str]] = field(
        default_factory=list
    )  # (host, container, mode)
    docker_devices: list[str] = field(default_factory=list)  # --device args as strings

    # Security
    docker_privileged: bool = False

    # Lifecycle / overrides
    docker_rm: bool = False
    docker_entrypoint: str | None = None
    docker_command: list[str] | None = None
    docker_extra_args: list[str] = field(default_factory=list)

    # Startup readiness
    docker_startup_timeout: float = 120.0
    docker_poll_interval: float = 1.0

    # Advanced
    docker_bin: str = "docker"


def is_docker_module(module_class: type) -> bool:
    """Check if a module class should run in Docker based on its default_config."""
    default_config = getattr(module_class, "default_config", None)
    return default_config is not None and issubclass(default_config, DockerModuleConfig)


# Docker helpers


def _run(cmd: list[str], *, timeout: float | None = None) -> subprocess.CompletedProcess[str]:
    logger.debug(f"exec: {' '.join(cmd)}")
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=False)


def _docker_bin(cfg: DockerModuleConfig) -> str:
    """Get docker binary path, defaulting to 'docker' if empty/None."""
    return cfg.docker_bin or "docker"


def _remove_container(cfg: DockerModuleConfig, name: str) -> None:
    _run([_docker_bin(cfg), "rm", "-f", name], timeout=DOCKER_CMD_TIMEOUT)


def _is_container_running(cfg: DockerModuleConfig, name: str) -> bool:
    r = _run(
        [_docker_bin(cfg), "inspect", "-f", "{{.State.Running}}", name],
        timeout=DOCKER_STATUS_TIMEOUT,
    )
    return r.returncode == 0 and r.stdout.strip() == "true"


def _tail_logs(cfg: DockerModuleConfig, name: str, n: int = LOG_TAIL_LINES) -> str:
    r = _run([_docker_bin(cfg), "logs", "--tail", str(n), name], timeout=DOCKER_CMD_TIMEOUT)
    out = (r.stdout or "").rstrip()
    err = (r.stderr or "").rstrip()
    return out + ("\n" + err if err else "")


def _extract_module_config(cfg: DockerModuleConfig) -> dict[str, Any]:
    """Extract JSON-serializable config fields for the container (excludes docker_* fields)."""
    out: dict[str, Any] = {}
    for k, v in cfg.__dict__.items():
        if k.startswith("docker_") or isinstance(v, type) or callable(v):
            continue
        try:
            json.dumps(v)
            out[k] = v
        except (TypeError, ValueError):
            logger.debug(f"Config field '{k}' not JSON-serializable, skipping")
    return out


# Host-side Docker-backed Module handle


class DockerModule:
    """
    Host-side handle for a module running inside Docker.

    Lifecycle:
    - start(): launches container, waits for module ready via RPC
    - stop(): stops container
    - __getattr__: exposes RpcCall for @rpc methods on remote module

    Communication: All RPC happens via LCM multicast (requires --network=host).
    """

    def __init__(self, module_class: type[Module], *args: Any, **kwargs: Any) -> None:
        # Config
        config_class = getattr(module_class, "default_config", DockerModuleConfig)
        config = config_class(**kwargs)

        # Module info
        self._module_class = module_class
        self._config = config
        self._args = args
        self._kwargs = kwargs
        self._running = False
        self.remote_name = module_class.__name__
        self._container_name = (
            config.docker_container_name
            or f"dimos_{module_class.__name__.lower()}_{os.getpid()}_{int(time.time())}"
        )

        # RPC setup
        self.rpc = LCMRPC()
        self.rpcs = set(module_class.rpcs.keys())  # type: ignore[attr-defined]
        self.rpc_calls: list[str] = getattr(module_class, "rpc_calls", [])
        self._unsub_fns: list[Callable[[], None]] = []
        self._bound_rpc_calls: dict[str, RpcCall] = {}

        # Build image if needed (but don't start - caller must call start() explicitly)
        if not image_exists(config):
            logger.info(f"Building {config.docker_image}")
            build_image(config)

    def set_rpc_method(self, method: str, callable: RpcCall) -> None:
        callable.set_rpc(self.rpc)
        self._bound_rpc_calls[method] = callable

    def get_rpc_calls(self, *methods: str) -> RpcCall | tuple[RpcCall, ...]:
        # Check all requested methods exist
        missing = set(methods) - self._bound_rpc_calls.keys()
        if missing:
            raise ValueError(f"RPC methods not found: {missing}")
        # Return single RpcCall or tuple
        calls = tuple(self._bound_rpc_calls[m] for m in methods)
        return calls[0] if len(calls) == 1 else calls

    def start(self) -> None:
        if self._running:
            return

        cfg = self._config

        # Prevent accidental kill of running container with same name
        if _is_container_running(cfg, self._container_name):
            raise RuntimeError(
                f"Container '{self._container_name}' already running. "
                "Choose a different container_name or stop the existing container."
            )
        _remove_container(cfg, self._container_name)

        cmd = self._build_docker_run_command()
        logger.info(f"Starting docker container: {self._container_name}")
        r = _run(cmd, timeout=DOCKER_RUN_TIMEOUT)
        if r.returncode != 0:
            raise RuntimeError(
                f"Failed to start container.\nSTDOUT:\n{r.stdout}\nSTDERR:\n{r.stderr}"
            )

        self.rpc.start()
        self._running = True
        self._wait_for_ready()

    def stop(self) -> None:
        """Gracefully stop the Docker container and clean up resources."""
        # Signal remote module, stop RPC, unsubscribe handlers (ignore failures)
        with suppress(Exception):
            if self._running:
                self.rpc.call_nowait(f"{self.remote_name}/stop", ([], {}))
        with suppress(Exception):
            self.rpc.stop()
        for unsub in self._unsub_fns:
            with suppress(Exception):
                unsub()
        self._unsub_fns.clear()

        # Stop and remove container
        _run([_docker_bin(self._config), "stop", self._container_name], timeout=DOCKER_STOP_TIMEOUT)
        _remove_container(self._config, self._container_name)
        self._running = False
        logger.info(f"Stopped container: {self._container_name}")

    def status(self) -> dict[str, Any]:
        cfg = self._config
        return {
            "module": self.remote_name,
            "container_name": self._container_name,
            "image": cfg.docker_image,
            "running": bool(self._running and _is_container_running(cfg, self._container_name)),
        }

    def tail_logs(self, n: int = 200) -> str:
        return _tail_logs(self._config, self._container_name, n=n)

    def set_transport(self, stream_name: str, transport: Any) -> bool:
        """Configure stream transport in container. Mirrors DaskModule.set_transport() for autoconnect()."""
        topic = getattr(transport, "topic", None)
        if topic is None:
            return False
        if hasattr(topic, "topic"):
            topic = topic.topic
        result, _ = self.rpc.call_sync(
            f"{self.remote_name}/configure_stream", ([stream_name, str(topic)], {})
        )
        return bool(result)

    def __getattr__(self, name: str) -> Any:
        if name in self.rpcs:
            original_method = getattr(self._module_class, name, None)
            return RpcCall(original_method, self.rpc, name, self.remote_name, self._unsub_fns, None)
        raise AttributeError(f"{name} not found on {self._module_class.__name__}")

    # Docker command building (split into focused helpers for readability)

    def _build_docker_run_command(self) -> list[str]:
        """Build the complete `docker run` command."""
        cfg = self._config
        self._validate_config(cfg)

        cmd = [_docker_bin(cfg), "run", "-d"]
        self._add_lifecycle_args(cmd, cfg)
        self._add_network_args(cmd, cfg)
        self._add_port_args(cmd, cfg)
        self._add_resource_args(cmd, cfg)
        self._add_security_args(cmd, cfg)
        self._add_device_args(cmd, cfg)
        self._add_label_args(cmd, cfg)
        self._add_env_args(cmd, cfg)
        self._add_volume_args(cmd, cfg)
        self._add_entrypoint_args(cmd, cfg)
        cmd.extend(cfg.docker_extra_args)

        cmd.append(cfg.docker_image)
        cmd.extend(self._build_container_command(cfg))
        return cmd

    def _validate_config(self, cfg: DockerModuleConfig) -> None:
        """Validate config before building command."""
        # Warn about network mode - LCM multicast requires host network
        using_host_network = cfg.docker_network is None and cfg.docker_network_mode == "host"
        if not using_host_network:
            logger.warning(
                "DockerModule not using host network. LCM multicast requires --network=host. "
                "RPC communication may not work with bridge/custom networks."
            )

    def _add_lifecycle_args(self, cmd: list[str], cfg: DockerModuleConfig) -> None:
        """Add --rm and --name args."""
        if cfg.docker_rm:
            cmd.append("--rm")
            if cfg.docker_restart_policy and cfg.docker_restart_policy != "no":
                logger.warning(
                    "--rm with docker_restart_policy is unusual; consider docker_restart_policy='no'."
                )
        cmd.extend(["--name", self._container_name])

    def _add_network_args(self, cmd: list[str], cfg: DockerModuleConfig) -> None:
        """Add --network args."""
        if cfg.docker_network and cfg.docker_network_mode != "host":
            logger.warning(
                "Both 'docker_network' and 'docker_network_mode' set; using 'docker_network' and ignoring 'docker_network_mode'."
            )
        if cfg.docker_network:
            cmd.extend(["--network", cfg.docker_network])
        else:
            cmd.append(f"--network={cfg.docker_network_mode}")

    def _add_port_args(self, cmd: list[str], cfg: DockerModuleConfig) -> None:
        """Add -p port args. No-op for host network (ports auto-exposed)."""
        if cfg.docker_network is None and cfg.docker_network_mode == "host":
            return
        # Non-host network: map Rerun ports + any custom ports
        for port in (RERUN_GRPC_PORT, RERUN_WEB_PORT):
            cmd.extend(["-p", f"{port}:{port}/tcp"])
        for host_port, container_port, proto in cfg.docker_ports:
            cmd.extend(["-p", f"{host_port}:{container_port}/{proto or 'tcp'}"])

    def _add_resource_args(self, cmd: list[str], cfg: DockerModuleConfig) -> None:
        """Add --shm-size, --restart, --gpus args."""
        cmd.append(f"--shm-size={cfg.docker_shm_size}")
        if cfg.docker_restart_policy:
            cmd.append(f"--restart={cfg.docker_restart_policy}")
        if cfg.docker_gpus:
            cmd.extend(["--gpus", cfg.docker_gpus])

    def _add_security_args(self, cmd: list[str], cfg: DockerModuleConfig) -> None:
        """Add --privileged if enabled."""
        if cfg.docker_privileged:
            cmd.append("--privileged")

    def _add_device_args(self, cmd: list[str], cfg: DockerModuleConfig) -> None:
        """Add --device args."""
        for dev in cfg.docker_devices:
            cmd.extend(["--device", dev])

    def _add_label_args(self, cmd: list[str], cfg: DockerModuleConfig) -> None:
        """Add --label args with DimOS defaults."""
        labels = dict(cfg.docker_labels)
        labels.setdefault("dimos.kind", "module")
        labels.setdefault("dimos.module", self._module_class.__name__)
        for k, v in labels.items():
            cmd.extend(["--label", f"{k}={v}"])

    def _add_env_args(self, cmd: list[str], cfg: DockerModuleConfig) -> None:
        """Add -e and --env-file args."""
        cmd.extend(["-e", "PYTHONUNBUFFERED=1"])
        for env_file in cfg.docker_env_files:
            cmd.extend(["--env-file", env_file])
        for k, v in cfg.docker_env.items():
            cmd.extend(["-e", f"{k}={v}"])

    def _add_volume_args(self, cmd: list[str], cfg: DockerModuleConfig) -> None:
        """Add -v volume args."""
        for host_path, container_path, mode in cfg.docker_volumes:
            cmd.extend(["-v", f"{host_path}:{container_path}:{mode}"])

    def _add_entrypoint_args(self, cmd: list[str], cfg: DockerModuleConfig) -> None:
        """Add --entrypoint override."""
        if cfg.docker_entrypoint:
            cmd.extend(["--entrypoint", cfg.docker_entrypoint])

    def _build_container_command(self, cfg: DockerModuleConfig) -> list[str]:
        """Build the container command (module runner or custom)."""
        if cfg.docker_command:
            return list(cfg.docker_command)

        module_path = f"{self._module_class.__module__}.{self._module_class.__name__}"
        # Filter out docker-specific kwargs (paths, etc.) - only pass module config
        kwargs = {"config": _extract_module_config(cfg)}
        payload = {"module_path": module_path, "args": list(self._args), "kwargs": kwargs}
        # DimOS base image entrypoint already runs "dimos.core.docker_runner run"
        return ["--payload", json.dumps(payload, separators=(",", ":"))]

    def _wait_for_ready(self) -> None:
        """Poll the module's RPC endpoint until ready, crashed, or timeout."""
        cfg = self._config
        start_time = time.time()

        logger.info(f"Waiting for {self.remote_name} to be ready...")

        while (time.time() - start_time) < cfg.docker_startup_timeout:
            if not _is_container_running(cfg, self._container_name):
                logs = _tail_logs(cfg, self._container_name)
                raise RuntimeError(f"Container died during startup:\n{logs}")

            try:
                self.rpc.call_sync(
                    f"{self.remote_name}/start", ([], {}), rpc_timeout=RPC_READY_TIMEOUT
                )
                elapsed = time.time() - start_time
                logger.info(f"{self.remote_name} ready ({elapsed:.1f}s)")
                return
            except (TimeoutError, ConnectionError, OSError):
                # Module not ready yet - retry after poll interval
                time.sleep(cfg.docker_poll_interval)

        logs = _tail_logs(cfg, self._container_name)
        raise RuntimeError(
            f"Timeout waiting for {self.remote_name} after {cfg.docker_startup_timeout:.1f}s:\n{logs}"
        )


# Container-side runner


class StandaloneModuleRunner:
    """Runs a module inside Docker container. Blocks until SIGTERM/SIGINT."""

    def __init__(self, module_path: str, args: list[Any], kwargs: dict[str, Any]) -> None:
        self._module_path = module_path
        self._args = args
        self._module: Module | None = None
        self._shutdown = threading.Event()

        # Merge config fields into kwargs (Configurable creates config from these)
        if "config" in kwargs:
            config_dict = kwargs.pop("config")
            kwargs = {**config_dict, **kwargs}
        self._kwargs = kwargs

    def start(self) -> None:
        mod_path, class_name = self._module_path.rsplit(".", 1)
        mod = importlib.import_module(mod_path)
        module_class = getattr(mod, class_name)

        self._module = module_class(*self._args, **self._kwargs)
        logger.info(f"[docker runner] module constructed: {class_name}")

    def stop(self) -> None:
        self._shutdown.set()
        if self._module is not None:
            try:
                self._module.stop()
            except Exception as e:
                logger.error(f"[docker runner] error stopping module: {e}")

    def wait(self) -> None:
        self._shutdown.wait()


def _install_signal_handlers(runner: StandaloneModuleRunner) -> None:
    def shutdown(_sig: int, _frame: Any) -> None:
        runner.stop()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)


def _cli_run(payload_json: str) -> None:
    payload = json.loads(payload_json)
    runner = StandaloneModuleRunner(
        payload["module_path"],
        payload.get("args", []),
        payload.get("kwargs", {}),
    )
    _install_signal_handlers(runner)
    runner.start()
    runner.wait()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="dimos.core.docker_runner")
    sub = parser.add_subparsers(dest="cmd", required=True)

    runp = sub.add_parser("run", help="Run a module inside a container")
    runp.add_argument("--payload", required=True, help="JSON payload with module_path and config")

    args = parser.parse_args(argv)

    if args.cmd == "run":
        _cli_run(args.payload)
        return

    raise ValueError(f"Unknown cmd: {args.cmd}")


if __name__ == "__main__":
    main()


__all__ = [
    "DockerModule",
    "DockerModuleConfig",
    "is_docker_module",
]
