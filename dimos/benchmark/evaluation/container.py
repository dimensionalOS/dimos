"""Build and run the locked outer environment for built-in Evaluations."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Mapping, Sequence
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
from typing import Final

from dimos.constants import CACHE_DIR, DIMOS_PROJECT_ROOT

_IMAGE_TAG: Final = "localhost/dimos-evaluation:build"
_CONTAINER_ROOT: Final = Path("/opt/dimos")
_SOCKET_TARGET: Final = Path("/run/podman/podman.sock")
_INSIDE_ENV: Final = "DIMOS_EVALUATION_CONTAINER"
_ENVIRONMENT_NAME: Final = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_DEFAULT_FORWARDED_ENV: Final = (
    "OPENAI_BASE_URL",
    "HF_TOKEN",
    "HUGGING_FACE_HUB_TOKEN",
)

CommandFactory = Callable[[Path], Sequence[str]]


class ContainerEvaluationError(RuntimeError):
    """The locked evaluation container could not be built or started."""


def inside_container(environ: Mapping[str, str] | None = None) -> bool:
    """Return whether execution is already inside the locked evaluation image."""
    return (os.environ if environ is None else environ).get(_INSIDE_ENV) == "1"


def _checked_run(command: Sequence[str]) -> None:
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
    iid_path.unlink(missing_ok=True)
    _checked_run(
        (
            "podman",
            "build",
            "--iidfile",
            str(iid_path),
            "--tag",
            _IMAGE_TAG,
            "--file",
            str(DIMOS_PROJECT_ROOT / "docker" / "evaluation" / "Dockerfile"),
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
    _checked_run(("nvidia-ctk", "cdi", "generate", f"--output={specification}"))
    if not specification.is_file():
        raise ContainerEvaluationError("nvidia-ctk did not produce a CDI specification")
    return cdi_directory


def candidate_path(path: Path) -> Path:
    """Translate a checked-in host path to the source baked into the image."""
    resolved = path.expanduser().resolve()
    try:
        relative = resolved.relative_to(DIMOS_PROJECT_ROOT.resolve())
    except ValueError as exc:
        raise ContainerEvaluationError("candidate input must be a checked-in path") from exc
    return _CONTAINER_ROOT / relative


def _validate_output(output: Path) -> None:
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise FileExistsError(f"Output must be absent or an empty directory: {output}")


def _environment_mounts(environ: Mapping[str, str]) -> tuple[Path, ...]:
    paths: list[Path] = []
    for name in ("EVO_RESULT_PATH", "EVO_TRACES_DIR"):
        value = environ.get(name)
        if value is None:
            continue
        path = Path(value)
        if not path.is_absolute():
            raise ContainerEvaluationError(f"{name} must be an absolute path")
        paths.append(path.parent if name == "EVO_RESULT_PATH" else path)
    return tuple(paths)


def _mount_arguments(
    *,
    read_only: Sequence[Path],
    writable: Sequence[Path],
) -> tuple[str, ...]:
    mounts = {path.resolve(): "ro" for path in read_only}
    mounts.update((path.resolve(), "rw") for path in writable)
    arguments: list[str] = []
    for path, mode in sorted(mounts.items(), key=lambda item: (len(item[0].parts), str(item[0]))):
        if path == Path("/"):
            raise ContainerEvaluationError("refusing to mount the host filesystem root")
        if mode == "ro":
            if not path.is_dir():
                raise ContainerEvaluationError(f"read-only input directory is unavailable: {path}")
        else:
            path.mkdir(parents=True, exist_ok=True)
        arguments.extend(("--volume", f"{path}:{path}:{mode}"))
    return tuple(arguments)


def _container_environment() -> dict[str, str]:
    return {
        _INSIDE_ENV: "1",
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
    scratch: Path,
    read_only: Sequence[Path],
    writable: Sequence[Path],
    forwarded_environment: Sequence[str],
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
        *_mount_arguments(read_only=read_only, writable=writable),
    ]
    for name in sorted(set(forwarded_environment)):
        if _ENVIRONMENT_NAME.fullmatch(name) is None:
            raise ContainerEvaluationError(f"invalid environment variable name: {name!r}")
        if environ.get(name):
            command.extend(("--env", name))
    for name, value in sorted(_container_environment().items()):
        command.extend(("--env", f"{name}={value}"))
    command.extend((image_id, *inner))
    return tuple(command)


def run_in_container(
    command: CommandFactory,
    *,
    output: Path,
    input_directories: Sequence[Path],
    forwarded_environment: Sequence[str],
    environ: Mapping[str, str] | None = None,
) -> int:
    """Run one command in the locked image and publish its complete output."""
    host_environment = dict(os.environ if environ is None else environ)
    output = output.expanduser().resolve()
    _validate_output(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}-container-", dir=output.parent))
    result = staging / "result"
    scratch = staging / "tmp"
    scratch.mkdir()
    try:
        socket = _podman_socket(host_environment)
        image_id = _build_image(staging / "image.iid")
        cdi_directory = _generate_cdi_spec(scratch)
        writable = (
            staging,
            CACHE_DIR.resolve(),
            *_environment_mounts(host_environment),
        )
        invocation = _container_command(
            image_id=image_id,
            cdi_directory=cdi_directory,
            socket=socket,
            scratch=scratch,
            read_only=input_directories,
            writable=writable,
            forwarded_environment=(*_DEFAULT_FORWARDED_ENV, *forwarded_environment),
            environ=host_environment,
            inner=tuple(command(result)),
        )
        try:
            completed = subprocess.run(invocation, check=False)
        except OSError as exc:
            raise ContainerEvaluationError("podman command failed") from exc
        if result.exists():
            if not result.is_dir():
                raise ContainerEvaluationError("evaluation output is not a directory")
            if output.exists():
                output.rmdir()
            os.replace(result, output)
        elif completed.returncode == 0:
            raise ContainerEvaluationError("evaluation container completed without output")
        return completed.returncode
    finally:
        shutil.rmtree(staging, ignore_errors=True)


def run_evaluation(
    specification: Path,
    *,
    output: Path,
    api_key_env: str,
    json_output: bool,
    quiet: bool,
) -> int:
    """Run one public Evaluation command in the locked image."""
    specification = specification.expanduser().resolve()

    def command(result: Path) -> Sequence[str]:
        inner = [
            str(_CONTAINER_ROOT / ".venv" / "bin" / "dimos"),
            "eval",
            "_inside-run",
            str(specification),
            "--api-key-env",
            api_key_env,
            "--output",
            str(result),
            "--display-output",
            str(output),
        ]
        if json_output:
            inner.append("--json")
        if quiet:
            inner.append("--quiet")
        return inner

    return run_in_container(
        command,
        output=output,
        input_directories=(specification.parent,),
        forwarded_environment=(api_key_env,),
    )


def _inside_check() -> None:
    # The full runtime is intentionally imported only inside the built image.
    from dimos.benchmark.evaluation.runtime import _pi_paths

    if DIMOS_PROJECT_ROOT.resolve() != _CONTAINER_ROOT:
        raise ContainerEvaluationError(
            f"candidate import resolved to {DIMOS_PROJECT_ROOT}, expected {_CONTAINER_ROOT}"
        )
    missing = [str(path) for path in _pi_paths() if not path.is_file()]
    if missing:
        raise ContainerEvaluationError(f"locked Pi build is incomplete: {', '.join(missing)}")
    _checked_run(("podman", "info", "--format", "{{.Host.Security.Rootless}}"))
    _checked_run(("nvidia-smi", "-L"))


def check_environment(workspace: Path, environ: Mapping[str, str] | None = None) -> None:
    """Build and verify the generic image without starting an Evaluation."""
    host_environment = dict(os.environ if environ is None else environ)
    workspace = workspace.expanduser().resolve()
    workspace.mkdir(parents=True, exist_ok=True)
    scratch = workspace / "tmp"
    scratch.mkdir(exist_ok=True)
    socket = _podman_socket(host_environment)
    image_id = _build_image(workspace / "image.iid")
    cdi_directory = _generate_cdi_spec(scratch)
    command = _container_command(
        image_id=image_id,
        cdi_directory=cdi_directory,
        socket=socket,
        scratch=scratch,
        read_only=(),
        writable=(CACHE_DIR.resolve(),),
        forwarded_environment=(),
        environ=host_environment,
        inner=(
            str(_CONTAINER_ROOT / ".venv" / "bin" / "python"),
            "-m",
            "dimos.benchmark.evaluation.container",
            "_inside-check",
        ),
    )
    _checked_run(command)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="mode", required=True)
    check = subparsers.add_parser("check", help="verify the locked evaluation environment")
    check.add_argument("--workspace", type=Path, required=True)
    subparsers.add_parser("_inside-check", help=argparse.SUPPRESS)
    return parser


def main(argv: Sequence[str] | None = None) -> None:
    arguments = _parser().parse_args(argv)
    if arguments.mode == "_inside-check":
        _inside_check()
    else:
        check_environment(arguments.workspace)


if __name__ == "__main__":
    main()
