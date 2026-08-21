"""Build and run the locked outer LIBERO autoresearch container."""

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
import os
from pathlib import Path
import subprocess
from typing import Final

from dimos.benchmark.libero_pro.podman import LiberoPodmanContainer
from dimos.constants import CACHE_DIR, DIMOS_PROJECT_ROOT

_IMAGE_TAG: Final = "localhost/dimos-libero-eval:build"
_CONTAINER_ROOT: Final = Path("/opt/dimos")
_SOCKET_TARGET: Final = Path("/run/podman/podman.sock")
_FORWARDED_ENV: Final = (
    "OPENAI_API_KEY",
    "OPENAI_BASE_URL",
    "HF_TOKEN",
    "HUGGING_FACE_HUB_TOKEN",
    "EVO_RESULT_PATH",
    "EVO_TRACES_DIR",
    "EVO_EXPERIMENT_ID",
)


class ContainerEvaluationError(RuntimeError):
    """The locked evaluation container could not be built or started."""


def _run(command: Sequence[str]) -> None:
    try:
        subprocess.run(command, check=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        raise ContainerEvaluationError(f"{command[0]} command failed") from exc


def _podman_socket(environ: Mapping[str, str]) -> Path:
    runtime = Path(environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"))
    socket = runtime / "podman" / "podman.sock"
    if not socket.is_socket():
        raise ContainerEvaluationError(
            f"rootless Podman socket is unavailable at {socket}; "
            "run `systemctl --user enable --now podman.socket`"
        )
    return socket


def _build_image(iid_path: Path) -> str:
    LiberoPodmanContainer.ensure_image()
    iid_path.unlink(missing_ok=True)
    _run(
        (
            "podman",
            "build",
            "--iidfile",
            str(iid_path),
            "--tag",
            _IMAGE_TAG,
            "--file",
            str(DIMOS_PROJECT_ROOT / "docker" / "libero-eval" / "Dockerfile"),
            str(DIMOS_PROJECT_ROOT),
        )
    )
    try:
        image_id = iid_path.read_text(encoding="utf-8").strip()
    finally:
        iid_path.unlink(missing_ok=True)
    if not image_id.startswith("sha256:"):
        raise ContainerEvaluationError(f"Podman returned an invalid image ID: {image_id!r}")
    return image_id


def _generate_cdi_spec(scratch: Path) -> Path:
    cdi_directory = scratch / "cdi"
    cdi_directory.mkdir(parents=True, exist_ok=True)
    specification = cdi_directory / "nvidia.yaml"
    _run(("nvidia-ctk", "cdi", "generate", f"--output={specification}"))
    if not specification.is_file():
        raise ContainerEvaluationError("nvidia-ctk did not produce a CDI specification")
    return cdi_directory


def _candidate_panel(path: Path) -> Path:
    resolved = path.resolve()
    try:
        relative = resolved.relative_to(DIMOS_PROJECT_ROOT.resolve())
    except ValueError as exc:
        raise ContainerEvaluationError("panel must be a checked-in candidate path") from exc
    return _CONTAINER_ROOT / relative


def _mount_directories(
    output: Path,
    environ: Mapping[str, str],
) -> tuple[Path, ...]:
    directories = {output, CACHE_DIR.resolve()}
    for name in ("EVO_RESULT_PATH", "EVO_TRACES_DIR"):
        value = environ.get(name)
        if value:
            path = Path(value).resolve()
            directories.add(path.parent if name == "EVO_RESULT_PATH" else path)
    for path in directories:
        if path == Path("/"):
            raise ContainerEvaluationError("refusing to mount the host filesystem root")
        path.mkdir(parents=True, exist_ok=True)
    return tuple(
        path
        for path in sorted(directories, key=lambda item: (len(item.parts), str(item)))
        if not any(parent != path and path.is_relative_to(parent) for parent in directories)
    )


def _container_environment(environ: Mapping[str, str]) -> dict[str, str]:
    for name in ("EVO_RESULT_PATH", "EVO_TRACES_DIR"):
        value = environ.get(name)
        if value and not Path(value).is_absolute():
            raise ContainerEvaluationError(f"{name} must be an absolute path")
    return {
        "CONTAINER_HOST": f"unix://{_SOCKET_TARGET}",
        "TMPDIR": "/runner-tmp",
        "TMP": "/runner-tmp",
        "TEMP": "/runner-tmp",
        "XDG_CACHE_HOME": str(CACHE_DIR.resolve().parent),
    }


def _container_command(
    *,
    image_id: str,
    cdi_directory: Path,
    socket: Path,
    output: Path,
    scratch: Path,
    environ: Mapping[str, str],
    inner: Sequence[str],
) -> tuple[str, ...]:
    command = [
        "podman",
        "--cdi-spec-dir",
        str(cdi_directory),
        "run",
        "--rm",
        "--network",
        "host",
        "--device",
        "nvidia.com/gpu=all",
        "--security-opt",
        "label=disable",
        "--volume",
        f"{socket}:{_SOCKET_TARGET}:rw",
        "--volume",
        f"{scratch}:/runner-tmp:rw",
    ]
    for path in _mount_directories(output, environ):
        command.extend(("--volume", f"{path}:{path}:rw"))
    for name in sorted(_FORWARDED_ENV):
        if environ.get(name):
            command.extend(("--env", name))
    for name, value in sorted(_container_environment(environ).items()):
        command.extend(("--env", f"{name}={value}"))
    command.extend((image_id, *inner))
    return tuple(command)


def _inside_check() -> None:
    # Keep the host launcher lightweight; the full runtime exists inside the built image.
    from dimos.benchmark.evaluation.runtime import _pi_paths

    if DIMOS_PROJECT_ROOT.resolve() != _CONTAINER_ROOT:
        raise ContainerEvaluationError(
            f"candidate import resolved to {DIMOS_PROJECT_ROOT}, expected {_CONTAINER_ROOT}"
        )
    missing = [str(path) for path in _pi_paths() if not path.is_file()]
    if missing:
        raise ContainerEvaluationError(f"locked Pi build is incomplete: {', '.join(missing)}")
    _run(("podman", "info", "--format", "{{.Host.Security.Rootless}}"))
    _run(("nvidia-smi", "-L"))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="mode", required=True)
    check = subparsers.add_parser("check", help="verify the locked runtime without a panel")
    check.add_argument("--output", type=Path, required=True)
    run = subparsers.add_parser("run", help="run the native autoresearch panel")
    run.add_argument("--panel", type=Path, required=True)
    run.add_argument("--output", type=Path, required=True)
    run.add_argument("--json", action="store_true")
    subparsers.add_parser("_inside-check", help=argparse.SUPPRESS)
    return parser


def main(argv: Sequence[str] | None = None) -> None:
    arguments = _parser().parse_args(argv)
    if arguments.mode == "_inside-check":
        _inside_check()
        return
    environ = dict(os.environ)
    output = arguments.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    socket = _podman_socket(environ)
    scratch = output / ".runner-tmp-host"
    scratch.mkdir(exist_ok=True)
    image_id = _build_image(output / f".runner-image-id-{os.getpid()}")
    cdi_directory = _generate_cdi_spec(scratch)
    if arguments.mode == "check":
        inner: list[str] = [
            str(_CONTAINER_ROOT / ".venv" / "bin" / "python"),
            "-m",
            "dimos.benchmark.libero_pro.container_runner",
            "_inside-check",
        ]
    else:
        inner = [
            str(_CONTAINER_ROOT / ".venv" / "bin" / "python"),
            "-m",
            "dimos.benchmark.libero_pro.autoresearch",
            "--panel",
            str(_candidate_panel(arguments.panel)),
            "--output",
            str(output),
        ]
        if arguments.json:
            inner.append("--json")
    _run(
        _container_command(
            image_id=image_id,
            cdi_directory=cdi_directory,
            socket=socket,
            output=output,
            scratch=scratch,
            environ=environ,
            inner=inner,
        )
    )


if __name__ == "__main__":
    main()
