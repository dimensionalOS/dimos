# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Supervisor-owned OCI and DimOS lifecycle for one external VLN-CE attempt."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
from typing import Any, Literal, cast

from pydantic import JsonValue

from dimos.benchmark.vlnce_r2r.blueprint import vlnce_r2r_eval_blueprint
from dimos.benchmark.vlnce_r2r.models import VlnceTaskManifest
from dimos.benchmark.vlnce_r2r.preparation import PreparationReceipt
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.porcelain.dimos import Dimos


class ExternalRuntimeError(RuntimeError):
    """The owned OCI/DimOS runtime could not honor its attempt contract."""


class VlnceExternalRuntime:
    """Launch one private benchmark container and its public DimOS blueprint."""

    def __init__(
        self,
        *,
        case: VlnceTaskManifest,
        attempt_id: str,
        attempt_path: Path,
        preparation: PreparationReceipt,
        image_id: str,
        render: Literal["none", "native"] = "none",
    ) -> None:
        source = case.source
        task = case.task
        self.case = case
        self.source = source
        self.task = task
        self.interaction = case.interaction
        self.attempt_id = attempt_id
        self.attempt_path = attempt_path
        self.preparation = preparation
        self.image_id = image_id
        self.render = render
        self.memory_path = attempt_path / "live-memory" / "recording.db"
        self.private_dir = attempt_path / "runtime-private"
        self.public_dir = Path(tempfile.gettempdir()) / "dimos-vlnce-uds" / self.attempt_id
        self.result_dir = attempt_path / "terminal-private"
        self.work_dir = attempt_path / "runtime-work"
        self.cdi_dir = attempt_path / "runtime-cdi"
        self.render_staging_dir = attempt_path / "runtime-render"
        self.render_path = attempt_path / "native-render.mp4"
        self.socket_path = self.public_dir / "gateway.sock"
        self.result_path = self.result_dir / "vlnce-result.v1.json"
        self.log_path = attempt_path / "oci-runtime.log"
        self._process: subprocess.Popen[bytes] | None = None
        self._log_handle: Any | None = None
        self._coordinator: ModuleCoordinator | None = None
        self._app: Dimos | None = None
        self._render_evidence: dict[str, JsonValue] | None = None
        self._previous_config = {
            "configure_system": global_config.configure_system,
            "n_workers": global_config.n_workers,
            "robot_model": global_config.robot_model,
            "transport": global_config.transport,
        }

    def private_case(self) -> dict[str, JsonValue]:
        """Return the exact private binding consumed before Habitat reset."""

        return cast(
            "dict[str, JsonValue]",
            {
                "schema_version": "vlnce-private-case.v1",
                "attempt_id": self.attempt_id,
                "case_id": self.case.case_id,
                "episode_id": self.source.episode_id,
                "episode_sha256": self.source.episode_sha256,
                "scene_id": self.source.scene_id,
                "split": self.source.split,
                "instruction": self.task.prompt,
                "timeout_seconds": self.interaction.timeout_seconds,
            },
        )

    def start(self, readiness_timeout_seconds: float = 180.0) -> dict[str, JsonValue]:
        """Start OCI, public blueprint, handshake, initial observation, and memory."""

        self._make_directories()
        private_case = self.private_case()
        private_path = self.private_dir / "private-case.json"
        _write_json(private_path, private_case)
        cdi_global_args, cdi_run_args = self._prepare_cdi()
        command = self._container_command(cdi_global_args, cdi_run_args)
        self._log_handle = self.log_path.open("xb")
        self._process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=self._log_handle,
            stderr=subprocess.STDOUT,
        )
        deadline = time.monotonic() + readiness_timeout_seconds
        while not self.socket_path.exists():
            self._raise_if_container_exited("before publishing its public socket")
            if time.monotonic() >= deadline:
                raise TimeoutError("VLN-CE container did not publish its public socket")
            time.sleep(0.05)

        global_config.update(robot_model="vlnce_habitat_cylinder")
        blueprint = vlnce_r2r_eval_blueprint(
            socket_path=self.socket_path,
            recording_path=self.memory_path,
        )
        self._coordinator = ModuleCoordinator.build(blueprint)
        self._coordinator.start_rpc_service()
        self._app = Dimos.connect(timeout=readiness_timeout_seconds)
        connection = cast("Any", self._app.VlnceConnection)
        connection.wait_ready(readiness_timeout_seconds)
        modules = self._coordinator.list_modules()
        required = {
            "VlnceConnection",
            "ReplanningAStarPlanner",
            "SpatialMemory",
            "NavigationSkillContainer",
            "VlnceObservationRecorder",
        }
        available = {module.class_name for module in modules}
        missing = sorted(required - available)
        if missing:
            raise ExternalRuntimeError("public blueprint is missing: " + ", ".join(missing))
        return cast(
            "dict[str, JsonValue]",
            {
                "schema_version": "1.0",
                "image_id": self.image_id,
                "container_name": self.container_name,
                "socket": str(self.socket_path),
                "module_count": len(modules),
                "required_modules": sorted(required),
                "handshake_ready": True,
                "container_isolation": {
                    "network": "none",
                    "no_new_privileges": True,
                },
                "public_diagnostics": connection.public_diagnostics(),
            },
        )

    def begin(self, memory_timeout_seconds: float = 10.0) -> None:
        """Cross the scored start barrier and expose the first recorded observation."""

        if self._app is None:
            raise ExternalRuntimeError("public DimOS interface is unavailable")
        cast("Any", self._app.VlnceConnection).begin()
        deadline = time.monotonic() + memory_timeout_seconds
        while not self.memory_path.is_file():
            self._raise_if_container_exited("while recording its initial observation")
            if time.monotonic() >= deadline:
                raise TimeoutError("initial public observation was not recorded")
            time.sleep(0.05)

    def public_evidence(self) -> dict[str, JsonValue]:
        """Capture terminal evidence available through the public DimOS interface."""

        if self._app is None:
            raise ExternalRuntimeError("public DimOS interface is unavailable")
        return cast(
            "dict[str, JsonValue]",
            cast("Any", self._app.VlnceConnection).public_diagnostics(),
        )

    @property
    def container_name(self) -> str:
        return "dimos-vlnce-{}".format(
            "".join(character if character.isalnum() else "-" for character in self.attempt_id)
        )[:120]

    def healthy(self) -> bool:
        if self.result_path.is_file():
            return True
        return (
            self._process is not None
            and self._process.poll() is None
            and self._coordinator is not None
            and self._coordinator.ping() == "pong"
        )

    def result_bytes(self) -> bytes | None:
        if not self.result_path.is_file():
            return None
        payload = self.result_path.read_bytes()
        if self._process is None:
            raise ExternalRuntimeError("terminal result appeared without an owned container")
        try:
            exit_code = self._process.wait(timeout=10.0)
        except subprocess.TimeoutExpired as error:
            raise ExternalRuntimeError("container did not exit after terminal result") from error
        if exit_code != 0:
            raise ExternalRuntimeError(f"container exited {exit_code} after terminal result")
        return payload

    def cancel_motion(self) -> None:
        if self._app is not None and hasattr(self._app, "ReplanningAStarPlanner"):
            cast("Any", self._app.ReplanningAStarPlanner).cancel_goal()

    def render_evidence(self) -> dict[str, JsonValue] | None:
        """Return sanitized presentation evidence after the container has stopped."""

        return self._render_evidence

    def close(self) -> None:
        errors: list[str] = []
        if self._app is not None and not self.result_path.exists():
            try:
                cast("Any", self._app.VlnceConnection).cancel()
            except Exception as error:
                errors.append(f"gateway-cancel: {type(error).__name__}: {error}")
        for name, resource in (("porcelain", self._app), ("dimos", self._coordinator)):
            if resource is None:
                continue
            try:
                resource.stop()
            except Exception as error:
                errors.append(f"{name}: {type(error).__name__}: {error}")
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=5.0)
        self._finalize_render_artifacts()
        if self._log_handle is not None:
            self._log_handle.close()
        try:
            if self.socket_path.exists():
                self.socket_path.unlink()
            if self.public_dir.exists():
                self.public_dir.rmdir()
        except OSError as error:
            errors.append(f"public-uds: {type(error).__name__}: {error}")
        global_config.update(**self._previous_config)
        if errors:
            raise ExternalRuntimeError("; ".join(errors))

    def _make_directories(self) -> None:
        self.public_dir.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        for path in (
            self.memory_path.parent,
            self.private_dir,
            self.public_dir,
            self.result_dir,
            self.work_dir,
            self.cdi_dir,
        ):
            path.mkdir(mode=0o700, parents=True, exist_ok=False)
        if self.render == "native":
            self.render_staging_dir.mkdir(mode=0o700, parents=True, exist_ok=False)

    def _prepare_cdi(self) -> tuple[list[str], list[str]]:
        nvidia_ctk = shutil.which("nvidia-ctk")
        if nvidia_ctk is None:
            return [], ["--device", "nvidia.com/gpu=all"]
        specification = self.cdi_dir / "nvidia.json"
        result = subprocess.run(
            [
                nvidia_ctk,
                "cdi",
                "generate",
                "--format=json",
                f"--output={specification}",
            ],
            capture_output=True,
            check=False,
        )
        if result.returncode != 0:
            raise ExternalRuntimeError("could not generate the attempt-local NVIDIA CDI spec")
        _remove_missing_cdi_devices(specification)
        return _cdi_arguments(self.cdi_dir)

    def _container_command(
        self,
        cdi_global_args: list[str],
        cdi_run_args: list[str],
    ) -> list[str]:
        episode_root = self.preparation.assets[self.source.episode_asset_id].root
        scene_root = self.preparation.assets[self.source.scene_asset_id].root
        dataset_path = Path("/dataset") / self.source.episode_path
        command = [
            "podman",
            *cdi_global_args,
            "run",
            *cdi_run_args,
            "--rm",
            "--name",
            self.container_name,
            "--network",
            "none",
            "--security-opt",
            "no-new-privileges",
            "--volume",
            f"{scene_root}:/assets/mp3d:ro",
            "--volume",
            f"{episode_root}:/dataset:ro",
            "--volume",
            f"{self.private_dir}:/private:ro",
            "--volume",
            f"{self.public_dir}:/public:rw",
            "--volume",
            f"{self.result_dir}:/result:rw",
            "--volume",
            f"{self.work_dir}:/work:rw",
        ]
        if self.render == "native":
            command.extend(
                [
                    "--volume",
                    f"{self.render_staging_dir}:/render:rw",
                ]
            )
        command.extend(
            [
                self.image_id,
                "serve",
                "--private-case",
                "/private/private-case.json",
                "--dataset",
                str(dataset_path),
                "--scenes-dir",
                "/assets",
                "--work-dir",
                "/work",
                "--socket",
                "/public/gateway.sock",
                "--result",
                "/result/vlnce-result.v1.json",
            ]
        )
        if self.render == "native":
            command.extend(
                [
                    "--render-output",
                    "/render/native-render.mp4",
                    "--render-metadata",
                    "/render/native-render.v1.json",
                ]
            )
        return command

    def _finalize_render_artifacts(self) -> None:
        if self.render != "native" or self._render_evidence is not None:
            return
        staged_video = self.render_staging_dir / "native-render.mp4"
        staged_metadata = self.render_staging_dir / "native-render.v1.json"
        if staged_video.is_file():
            os.replace(staged_video, self.render_path)
        if staged_metadata.is_file():
            try:
                payload = json.loads(staged_metadata.read_text(encoding="utf-8"))
            except (OSError, ValueError) as error:
                payload = {
                    "schema_version": "native-render.v1",
                    "status": "failed",
                    "diagnostic": f"invalid renderer metadata: {type(error).__name__}",
                }
            staged_metadata.unlink(missing_ok=True)
        else:
            payload = {
                "schema_version": "native-render.v1",
                "status": "failed",
                "diagnostic": "renderer did not publish metadata",
            }
        if not isinstance(payload, dict):
            payload = {
                "schema_version": "native-render.v1",
                "status": "failed",
                "diagnostic": "renderer metadata was not a JSON object",
            }
        if payload.get("status") == "completed" and not self.render_path.is_file():
            payload["status"] = "failed"
            payload["diagnostic"] = "renderer reported completion without a video"
        if payload.get("status") != "completed" and self.render_path.is_file():
            self.render_path.unlink()
        self._render_evidence = cast("dict[str, JsonValue]", payload)
        if self.render_staging_dir.exists():
            try:
                self.render_staging_dir.rmdir()
            except OSError:
                pass

    def _raise_if_container_exited(self, stage: str) -> None:
        if self._process is not None and self._process.poll() is not None:
            raise ExternalRuntimeError(
                f"VLN-CE container exited {self._process.returncode} {stage}; see {self.log_path}"
            )


def preparation_evidence(
    receipt: PreparationReceipt,
    image_id: str,
) -> dict[str, JsonValue]:
    """Serialize only public preparation identities and cache outcomes."""

    return cast(
        "dict[str, JsonValue]",
        {
            "schema_version": "1.0",
            "image_id": image_id,
            "episode_sha256": receipt.episode.episode_sha256,
            "assets": {
                asset_id: {
                    "archive_sha256": asset.archive_sha256,
                    "required_file_sha256": dict(asset.required_file_sha256),
                    "cache_hit": asset.cache_hit,
                }
                for asset_id, asset in receipt.assets.items()
            },
        },
    )


def _cdi_arguments(specification_directory: Path) -> tuple[list[str], list[str]]:
    """Place CDI directory and device flags on their respective Podman parsers."""

    return (
        ["--cdi-spec-dir", str(specification_directory)],
        ["--device", "nvidia.com/gpu=all"],
    )


def _remove_missing_cdi_devices(specification: Path) -> None:
    """Remove NVML-discovered DRM nodes that are absent from the host namespace."""

    document = json.loads(specification.read_text(encoding="utf-8"))
    edits = [document.get("containerEdits", {})]
    edits.extend(device.get("containerEdits", {}) for device in document.get("devices", []))
    missing_names: set[str] = set()
    for edit in edits:
        nodes = edit.get("deviceNodes", [])
        retained = []
        for node in nodes:
            path = Path(node.get("hostPath", node.get("path", "")))
            if path.is_absolute() and path.exists():
                retained.append(node)
            else:
                missing_names.add(path.name)
        edit["deviceNodes"] = retained
    for edit in edits:
        for hook in edit.get("hooks", []):
            args = hook.get("args", [])
            retained_args = []
            index = 0
            while index < len(args):
                if (
                    args[index] == "--link"
                    and index + 1 < len(args)
                    and any(f"../{name}::" in args[index + 1] for name in missing_names)
                ):
                    index += 2
                    continue
                retained_args.append(args[index])
                index += 1
            hook["args"] = retained_args
    specification.write_text(
        json.dumps(document, sort_keys=True, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )


def _write_json(path: Path, value: dict[str, JsonValue]) -> None:
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC, 0o600)
    try:
        data = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        remaining = memoryview(data.encode("utf-8") + b"\n")
        while remaining:
            remaining = remaining[os.write(descriptor, remaining) :]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
