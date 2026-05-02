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
from dataclasses import field
import hashlib
import importlib
import json
import signal
import subprocess
import threading
import time
from typing import TYPE_CHECKING, Any, get_type_hints

from dimos.core.module import ModuleBase, ModuleConfig
from dimos.core.rpc_client import ModuleProxyProtocol, RpcCall
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.bridge import RERUN_GRPC_PORT, RERUN_WEB_PORT

if TYPE_CHECKING:
    from collections.abc import Callable
    from pathlib import Path

    from dimos.core.module import Module

logger = setup_logger()

DOCKER_RUN_TIMEOUT = 120  # Timeout for `docker run` command execution
DOCKER_PULL_TIMEOUT_DEFAULT = None  # No timeout for `docker pull` (images can be large)
DOCKER_CMD_TIMEOUT = 20  # Timeout for quick Docker commands (inspect, rm, logs)
DOCKER_STATUS_TIMEOUT = 10  # Timeout for container status checks
DOCKER_STOP_TIMEOUT = 30  # Timeout for `docker stop` command (graceful shutdown)
LOG_TAIL_LINES = 200  # Number of log lines to include in error messages


class DockerModuleConfig(ModuleConfig):
    """
    Configuration for running a DimOS module inside Docker.

    For advanced Docker options not listed here, use docker_extra_args.
    Example: docker_extra_args=["--cap-add=SYS_ADMIN", "--read-only"]

    NOTE: a DockerModuleProxy will rebuild automatically if the Dockerfile or build args change
    """

    # Build / image
    docker_image: str
    docker_file: Path | None = None  # Required on host for building, not needed in container
    docker_build_context: Path | None = None
    docker_build_args: dict[str, str] = field(default_factory=dict)
    docker_build_extra_args: list[str] = field(default_factory=list)  # Extra args for docker build

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
    docker_gpus: str | None = None
    docker_shm_size: str = "4g"
    docker_restart_policy: str = "no"

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

    # Timeouts
    docker_pull_timeout: float | None = DOCKER_PULL_TIMEOUT_DEFAULT
    docker_startup_timeout: float = 120.0
    docker_poll_interval: float = 1.0

    # Reconnect to a running container instead of restarting it
    docker_reconnect_container: bool = False

    # Advanced
    docker_bin: str = "docker"


class DockerModuleProxy(ModuleProxyProtocol):
    """
    Host-side handle for a module running inside Docker.

    Lifecycle:
    - __init__(): lightweight setup — config, names, RPC client, no side-effects
    - build(): heavy work — docker build/pull image, launch container, wait for RPC readiness
    - start(): invoke remote module's start() RPC (after streams are wired)
    - stop(): stops the container and cleans up

    Communication: All RPC happens via LCM multicast (requires --network=host).
    """

    config: DockerModuleConfig

    def __init__(self, module_class: type[ModuleBase], *args: Any, **kwargs: Any) -> None:
        config_class = get_type_hints(module_class).get("config", DockerModuleConfig)
        if not issubclass(config_class, DockerModuleConfig):
            raise TypeError(
                f"{module_class.__name__} config must be a DockerModuleConfig subclass, "
                f"got {config_class.__name__}"
            )
        config = config_class(**kwargs)

        self._module_class = module_class
        self.config = config
        self._args = args
        self._kwargs = kwargs
        self._running = threading.Event()
        self._is_built = threading.Event()
        self._reconnected = False  # True when reusing a pre-existing container
        self.remote_name = module_class.__name__
        # Derive container name from image + class name: "my-registry/foo:v2" → "dimos_myclass_foo_v2"
        image_ref = config.docker_image.rsplit("/", 1)[-1]
        self._container_name = (
            config.docker_container_name
            or f"dimos_{module_class.__name__.lower()}_{image_ref.replace(':', '_')}"
        )

        self.rpc = LCMRPC(
            rpc_timeouts=self.config.rpc_timeouts,
            default_rpc_timeout=self.config.default_rpc_timeout,
        )
        self.rpcs = set(module_class.rpcs.keys())
        self._unsub_fns: list[Callable[[], None]] = []

    def build(self) -> None:
        """Build/pull docker image, launch container, wait for RPC readiness.

        Idempotent — safe to call multiple times. Has no RPC timeout since
        this runs host-side (not via RPC to a worker process).
        """
        if self._is_built.is_set():
            return

        config = self.config
        try:
            if config.docker_file is not None:
                current_hash = _compute_build_hash(config)
                stored_hash = _get_image_build_hash(config)
                if current_hash != stored_hash:
                    logger.info(f"Building {config.docker_image}")
                    build_image(config)
            elif not image_exists(config):
                logger.info(f"Pulling {config.docker_image}")
                pull = subprocess.run(
                    [config.docker_bin, "pull", config.docker_image],
                    timeout=config.docker_pull_timeout,
                    check=False,
                )
                if pull.returncode != 0:
                    raise RuntimeError(f"Failed to pull image '{config.docker_image}'.")

            reconnect = False
            if _is_container_running(config, self._container_name):
                if config.docker_reconnect_container:
                    logger.info(f"Reconnecting to running container: {self._container_name}")
                    reconnect = True
                    self._reconnected = True
                else:
                    logger.info(f"Stopping existing container: {self._container_name}")
                    _run(
                        [config.docker_bin, "stop", self._container_name],
                        timeout=DOCKER_STOP_TIMEOUT,
                    )

            if not reconnect:
                _remove_container(config, self._container_name)
                cmd = self._build_docker_run_command()
                logger.info(f"Starting docker container: {self._container_name}")
                r = _run(cmd, timeout=DOCKER_RUN_TIMEOUT)
                if r.returncode != 0:
                    raise RuntimeError(
                        f"Failed to start container.\nSTDOUT:\n{r.stdout}\nSTDERR:\n{r.stderr}"
                    )
            self.rpc.start()
            self._running.set()
            # docker run -d returns before Module.__init__ finishes in the container,
            # so we poll until the RPC server is reachable before returning.
            self._wait_for_rpc()
            self._is_built.set()
        except Exception:
            with suppress(Exception):
                self._cleanup()
            raise

    def start(self) -> None:
        """Invoke the remote module's start() RPC."""
        try:
            self.rpc.call_sync(f"{self.remote_name}/start", ([], {}))
        except Exception:
            with suppress(Exception):
                self.stop()
            raise

    def stop(self) -> None:
        """Gracefully stop the Docker container and clean up resources."""
        if not self._running.is_set():
            return
        self._running.clear()  # claim shutdown before any side-effects
        if not self._reconnected:
            with suppress(Exception):
                self.rpc.call_nowait(f"{self.remote_name}/stop", ([], {}))
        self._cleanup()

    def _cleanup(self) -> None:
        """Release all resources. Idempotent — safe to call from partial init or after stop()."""
        with suppress(Exception):
            self.rpc.stop()
        for unsub in self._unsub_fns:
            with suppress(Exception):
                unsub()
        self._unsub_fns.clear()
        if not self._reconnected:
            with suppress(Exception):
                _run(
                    [self.config.docker_bin, "stop", self._container_name],
                    timeout=DOCKER_STOP_TIMEOUT,
                )
            with suppress(Exception):
                _remove_container(self.config, self._container_name)
        self._running.clear()
        logger.info(f"Cleaned up container handle: {self._container_name}")

    def status(self) -> dict[str, Any]:
        cfg = self.config
        return {
            "module": self.remote_name,
            "container_name": self._container_name,
            "image": cfg.docker_image,
            "running": self._running.is_set() and _is_container_running(cfg, self._container_name),
        }

    def is_running(self) -> bool:
        return self._running.is_set() and _is_container_running(self.config, self._container_name)

    def tail_logs(self, n: int = 200) -> str:
        return _tail_logs(self.config, self._container_name, n=n)

    def set_transport(self, stream_name: str, transport: Any) -> bool:
        """Forward to the container's Module.set_transport RPC."""
        result, _ = self.rpc.call_sync(
            f"{self.remote_name}/set_transport",
            ([stream_name, transport], {}),
        )
        return bool(result)

    def __getattr__(self, name: str) -> Any:
        rpcs = self.__dict__.get("rpcs")
        if rpcs is not None and name in rpcs:
            original_method = getattr(self._module_class, name, None)
            return RpcCall(
                original_method,
                self.rpc,
                name,
                self.remote_name,
                self._unsub_fns,
                None,
            )
        raise AttributeError(f"{name} not found on {type(self).__name__}")

    # Docker command building (split into focused helpers for readability)

    def _build_docker_run_command(self) -> list[str]:
        """Build the complete `docker run` command."""
        cfg = self.config
        self._validate_config(cfg)

        cmd = [cfg.docker_bin, "run", "-d"]
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
                "DockerModuleProxy not using host network. LCM multicast requires --network=host. "
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

        module_name = self._module_class.__module__
        if module_name == "__main__":
            raise RuntimeError(
                f"Cannot deploy {self._module_class.__name__} from __main__. "
                "Run with `python -m` or set docker_command explicitly."
            )
        module_path = f"{module_name}.{self._module_class.__qualname__}"
        # Filter out docker-specific kwargs (paths, etc.) - only pass module config
        kwargs = {"config": _extract_module_config(cfg)}
        payload = {"module_path": module_path, "args": list(self._args), "kwargs": kwargs}
        # DimOS base image entrypoint already runs "dimos.core.docker_module run"
        try:
            payload_json = json.dumps(payload, separators=(",", ":"))
        except TypeError as e:
            raise TypeError(
                f"Cannot serialize DockerModuleProxy payload to JSON: {e}\n"
                f"Ensure all constructor args/kwargs for {self._module_class.__name__} are "
                f"JSON-serializable, or use docker_command to bypass automatic payload generation."
            ) from e
        return ["--payload", payload_json]

    def _wait_for_rpc(self) -> None:
        """Poll until the container's RPC server is reachable."""
        cfg = self.config
        start_time = time.time()

        logger.info(f"Waiting for {self.remote_name} to be ready...")

        while (time.time() - start_time) < cfg.docker_startup_timeout:
            if not _is_container_running(cfg, self._container_name):
                logs = _tail_logs(cfg, self._container_name)
                raise RuntimeError(f"Container died during startup:\n{logs}")

            try:
                self.rpc.call_sync(
                    f"{self.remote_name}/get_skills",
                    ([], {}),
                    rpc_timeout=3.0,  # short timeout for polling readiness
                )
                elapsed = time.time() - start_time
                logger.info(f"{self.remote_name} ready ({elapsed:.1f}s)")
                return
            except (TimeoutError, ConnectionError, OSError):
                time.sleep(cfg.docker_poll_interval)

        logs = _tail_logs(cfg, self._container_name)
        raise RuntimeError(
            f"Timeout waiting for {self.remote_name} after {cfg.docker_startup_timeout:.1f}s:\n{logs}"
        )


class DockerModuleInner:
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


def _run(cmd: list[str], *, timeout: float | None = None) -> subprocess.CompletedProcess[str]:
    logger.debug(f"exec: {' '.join(cmd)}")
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=False)


def _remove_container(cfg: DockerModuleConfig, name: str) -> None:
    _run([cfg.docker_bin, "rm", "-f", name], timeout=DOCKER_CMD_TIMEOUT)


def _is_container_running(cfg: DockerModuleConfig, name: str) -> bool:
    r = _run(
        [cfg.docker_bin, "inspect", "-f", "{{.State.Running}}", name],
        timeout=DOCKER_STATUS_TIMEOUT,
    )
    return r.returncode == 0 and r.stdout.strip() == "true"


def _tail_logs(cfg: DockerModuleConfig, name: str, n: int = LOG_TAIL_LINES) -> str:
    r = _run([cfg.docker_bin, "logs", "--tail", str(n), name], timeout=DOCKER_CMD_TIMEOUT)
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
            level = "debug" if k.startswith("_") else "warning"
            getattr(logger, level)(f"Config field '{k}' not JSON-serializable, skipping")
    return out


_BUILD_HASH_LABEL = "dimos.build.hash"

# the way of detecting already-converted Dockerfiles (UUID ensures uniqueness)
DIMOS_SENTINEL = "DIMOS-MODULE-CONVERSION-427593ae-c6e8-4cf1-9b2d-ee81a420a5dc"

# Footer appended to Dockerfiles for DimOS module conversion
DIMOS_FOOTER = f"""
# ==== {DIMOS_SENTINEL} ====
# Copy DimOS source from build context
COPY dimos /dimos/source/dimos/
COPY pyproject.toml /dimos/source/
COPY docker/python/module-install.sh /tmp/module-install.sh

# Install DimOS and create entrypoint
RUN bash /tmp/module-install.sh /dimos/source && rm /tmp/module-install.sh

ENTRYPOINT ["/dimos/entrypoint.sh"]
"""


def _convert_dockerfile(dockerfile: Path) -> Path:
    """Append DimOS footer to Dockerfile. Returns path to converted file."""
    content = dockerfile.read_text()

    # Already converted?
    if DIMOS_SENTINEL in content:
        return dockerfile

    logger.info(f"Converting {dockerfile.name} to DimOS format")

    converted = dockerfile.parent / f".{dockerfile.name}.ignore"
    converted.write_text(content.rstrip() + "\n" + DIMOS_FOOTER.lstrip("\n"))
    return converted


def _compute_build_hash(cfg: DockerModuleConfig) -> str:
    """Hash Dockerfile contents and build args."""
    if cfg.docker_file is None:
        raise ValueError("docker_file is required for computing build hash")
    digest = hashlib.sha256()
    digest.update(cfg.docker_file.read_bytes())
    for key, val in sorted(cfg.docker_build_args.items()):
        digest.update(f"{key}={val}".encode())
    for arg in cfg.docker_build_extra_args:
        digest.update(arg.encode())
    return digest.hexdigest()


def _get_image_build_hash(cfg: DockerModuleConfig) -> str | None:
    """Read the build hash label from an existing Docker image."""
    r = subprocess.run(
        [
            cfg.docker_bin,
            "image",
            "inspect",
            "-f",
            '{{index .Config.Labels "' + _BUILD_HASH_LABEL + '"}}',
            cfg.docker_image,
        ],
        capture_output=True,
        text=True,
        timeout=DOCKER_CMD_TIMEOUT,
        check=False,
    )
    if r.returncode != 0:
        return None
    value = r.stdout.strip()
    # docker prints "<no value>" when the label is missing
    return value if value and value != "<no value>" else None


def build_image(cfg: DockerModuleConfig) -> None:
    """Build Docker image using footer mode conversion."""
    if cfg.docker_file is None:
        raise ValueError("docker_file is required for building Docker images")

    build_hash = _compute_build_hash(cfg)
    dockerfile = _convert_dockerfile(cfg.docker_file)

    context = cfg.docker_build_context or cfg.docker_file.parent
    cmd = [cfg.docker_bin, "build", "-t", cfg.docker_image, "-f", str(dockerfile)]
    cmd.extend(["--label", f"{_BUILD_HASH_LABEL}={build_hash}"])
    for k, v in cfg.docker_build_args.items():
        cmd.extend(["--build-arg", f"{k}={v}"])
    cmd.extend(cfg.docker_build_extra_args)
    cmd.append(str(context))

    logger.info(f"Building Docker image: {cfg.docker_image}")
    # Stream stdout to terminal so the user sees build progress, but capture
    # stderr separately so we can include it in the error message on failure.
    result = subprocess.run(cmd, text=True, stderr=subprocess.PIPE)
    if result.returncode != 0:
        raise RuntimeError(
            f"Docker build failed with exit code {result.returncode}\nSTDERR:\n{result.stderr}"
        )


def image_exists(cfg: DockerModuleConfig) -> bool:
    """Check if the configured Docker image exists locally."""
    r = subprocess.run(
        [cfg.docker_bin, "image", "inspect", cfg.docker_image],
        capture_output=True,
        text=True,
        timeout=DOCKER_CMD_TIMEOUT,
        check=False,
    )
    return r.returncode == 0


def _install_signal_handlers(runner: DockerModuleInner) -> None:
    def shutdown(_sig: int, _frame: Any) -> None:
        runner.stop()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)


def _cli_run(payload_json: str) -> None:
    payload = json.loads(payload_json)
    runner = DockerModuleInner(
        payload["module_path"],
        payload.get("args", []),
        payload.get("kwargs", {}),
    )
    _install_signal_handlers(runner)
    runner.start()
    runner.wait()


# Container-side entrypoint: invoked as `python -m dimos.core.docker_module run --payload '...'`
# by the generated entrypoint.sh inside Docker containers (see docker/python/module-install.sh).
# This is what makes `DockerModuleInner` actually run — without it, containers would have no
# way to bootstrap the module from the JSON payload that `DockerModuleProxy` passes via `docker run`.
def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="dimos.core.docker_module")
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
    "DIMOS_FOOTER",
    "DockerModuleConfig",
    "DockerModuleInner",
    "DockerModuleProxy",
    "build_image",
    "image_exists",
]
