"""Rootless Podman lifecycle for one isolated LIBERO-PRO trial."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
import secrets
import subprocess
from uuid import uuid4

from dimos.benchmark.libero_pro.assets import PreparedAssets
from dimos.benchmark.libero_pro.models import LiberoTaskManifest
from dimos.constants import DIMOS_PROJECT_ROOT


def _image_source_digest() -> str:
    """Identify the exact local sources copied into the trial image."""
    sources = [
        path
        for root in (
            DIMOS_PROJECT_ROOT / "docker" / "libero-pro",
            DIMOS_PROJECT_ROOT / "dimos" / "benchmark" / "libero_pro" / "proto",
        )
        for path in root.rglob("*")
        if path.is_file()
        and (path.name == "Dockerfile" or path.suffix in {".proto", ".py", ".pyi", ".txt", ".yaml"})
    ]
    digest = hashlib.sha256()
    for path in sorted(sources):
        digest.update(path.relative_to(DIMOS_PROJECT_ROOT).as_posix().encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()[:12]


IMAGE = f"localhost/dimos-libero-pro:{_image_source_digest()}"
POLICY_CONTAINER_PORT = 50051
CONTROL_CONTAINER_PORT = 50052


class PodmanError(RuntimeError):
    """Podman could not build or run the LIBERO-PRO image."""


@dataclass(frozen=True)
class ContainerEndpoints:
    policy: str
    control: str
    control_token: str


class LiberoPodmanContainer:
    def __init__(
        self,
        manifest: LiberoTaskManifest,
        assets: PreparedAssets,
        *,
        artifact_dir: Path,
    ) -> None:
        self.manifest = manifest
        self.assets = assets
        self.artifact_dir = artifact_dir
        self.name = f"dimos-libero-{uuid4().hex}"
        self.token = secrets.token_urlsafe(32)
        self._running = False

    @staticmethod
    def ensure_image() -> None:
        exists = subprocess.run(
            ["podman", "image", "exists", IMAGE],
            check=False,
        )
        if exists.returncode == 0:
            return
        _run(
            [
                "podman",
                "build",
                "--tag",
                IMAGE,
                "--file",
                str(DIMOS_PROJECT_ROOT / "docker" / "libero-pro" / "Dockerfile"),
                str(DIMOS_PROJECT_ROOT),
            ]
        )

    def start(self) -> ContainerEndpoints:
        if self._running:
            raise RuntimeError("LIBERO-PRO container is already running")
        self.artifact_dir.mkdir(parents=True, exist_ok=True)
        manifest_path = self.artifact_dir / "task.json"
        manifest_path.write_text(self.manifest.model_dump_json(indent=2) + "\n")
        _run(
            [
                "podman",
                "run",
                "--detach",
                "--rm",
                "--name",
                self.name,
                "--publish",
                f"127.0.0.1::{POLICY_CONTAINER_PORT}",
                "--publish",
                f"127.0.0.1::{CONTROL_CONTAINER_PORT}",
                "--env",
                f"DIMOS_LIBERO_CONTROL_TOKEN={self.token}",
                "--volume",
                f"{self.assets.bddl}:/task/task.bddl:ro,Z",
                "--volume",
                f"{self.assets.init_states}:/task/init_states.pruned_init:ro,Z",
                "--volume",
                f"{manifest_path}:/task/task.json:ro,Z",
                IMAGE,
            ]
        )
        self._running = True
        return ContainerEndpoints(
            policy=f"127.0.0.1:{_published_port(self.name, POLICY_CONTAINER_PORT)}",
            control=f"127.0.0.1:{_published_port(self.name, CONTROL_CONTAINER_PORT)}",
            control_token=self.token,
        )

    def stop(self) -> None:
        if not self._running:
            return
        logs = subprocess.run(
            ["podman", "logs", self.name],
            check=False,
            capture_output=True,
            text=True,
        )
        (self.artifact_dir / "container.log").write_text(
            logs.stdout + logs.stderr,
            encoding="utf-8",
        )
        subprocess.run(["podman", "stop", "--time", "2", self.name], check=False)
        subprocess.run(["podman", "rm", "--force", self.name], check=False)
        self._running = False

    def __enter__(self) -> LiberoPodmanContainer:
        self.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self.stop()


def _published_port(name: str, container_port: int) -> int:
    output = _run(["podman", "port", name, f"{container_port}/tcp"])
    try:
        return int(output.rsplit(":", 1)[1])
    except (IndexError, ValueError) as exc:
        raise PodmanError(f"invalid Podman port mapping: {output!r}") from exc


def _run(command: list[str]) -> str:
    try:
        result = subprocess.run(command, check=True, capture_output=True, text=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        stderr = getattr(exc, "stderr", "")
        raise PodmanError(f"Podman command failed: {stderr or exc}") from exc
    return result.stdout.strip()
