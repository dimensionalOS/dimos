"""Tests for the shared locked Evaluation launcher."""

from pathlib import Path
import subprocess

import pytest
from pytest_mock import MockerFixture

from dimos.benchmark.evaluation import container


def test_container_command_uses_immutable_image_and_explicit_authority(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    cache = tmp_path / "cache" / "dimos"
    monkeypatch.setattr(container, "CACHE_DIR", cache)
    input_directory = tmp_path / "input"
    input_directory.mkdir()
    staging = tmp_path / "staging"
    scratch = staging / "tmp"
    socket = tmp_path / "podman.sock"
    environ = {
        "OPENAI_API_KEY": "secret",
        "UNRELATED_SECRET": "do-not-forward",
    }

    command = container._container_command(
        image_id="sha256:candidate",
        cdi_directory=tmp_path / "cdi",
        socket=socket,
        scratch=scratch,
        read_only=(input_directory,),
        writable=(staging, cache),
        forwarded_environment=("OPENAI_API_KEY",),
        environ=environ,
        inner=("python", "-m", "benchmark"),
    )

    rendered = " ".join(command)
    assert "--network host" in rendered
    assert f"--cdi-spec-dir {tmp_path / 'cdi'}" in rendered
    assert "--device nvidia.com/gpu=all" in rendered
    assert f"{socket}:/run/podman/podman.sock:rw" in rendered
    assert f"{scratch}:/runner-tmp:rw" in rendered
    assert f"{input_directory}:{input_directory}:ro" in rendered
    assert f"{staging}:{staging}:rw" in rendered
    assert f"{cache}:{cache}:rw" in rendered
    assert "--env OPENAI_API_KEY" in rendered
    assert "DIMOS_EVALUATION_CONTAINER=1" in rendered
    assert "secret" not in rendered
    assert "UNRELATED_SECRET" not in rendered
    assert command[-4:] == ("sha256:candidate", "python", "-m", "benchmark")


def test_mount_arguments_rejects_host_root(tmp_path: Path) -> None:
    with pytest.raises(container.ContainerEvaluationError, match="filesystem root"):
        container._mount_arguments(read_only=(Path("/"),), writable=(tmp_path,))


def test_candidate_input_must_be_baked_into_image(tmp_path: Path) -> None:
    with pytest.raises(container.ContainerEvaluationError, match="checked-in"):
        container.candidate_path(tmp_path / "panel.json")


def test_build_image_runs_by_returned_image_id(mocker: MockerFixture, tmp_path: Path) -> None:
    def write_iid(command: tuple[str, ...]) -> None:
        Path(command[command.index("--iidfile") + 1]).write_text("sha256:exact\n")

    run = mocker.patch.object(container, "_checked_run", side_effect=write_iid)
    iid = tmp_path / "iid"

    image_id = container._build_image(iid)

    assert image_id == "sha256:exact"
    assert not iid.exists()
    assert "docker/evaluation/Dockerfile" in " ".join(run.call_args.args[0])


def test_generate_cdi_spec_is_scoped_to_scratch(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    def write_spec(command: tuple[str, ...]) -> None:
        output = next(argument for argument in command if argument.startswith("--output="))
        Path(output.removeprefix("--output=")).write_text("cdiVersion: 0.7.0\n")

    run = mocker.patch.object(container, "_checked_run", side_effect=write_spec)

    directory = container._generate_cdi_spec(tmp_path)

    assert directory == tmp_path / "cdi"
    assert (directory / "nvidia.yaml").is_file()
    assert run.call_args.args[0][:3] == ("nvidia-ctk", "cdi", "generate")


def test_recorded_failure_output_is_published(
    mocker: MockerFixture,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(container, "CACHE_DIR", tmp_path / "cache")
    mocker.patch.object(container, "_podman_socket", return_value=tmp_path / "podman.sock")
    mocker.patch.object(container, "_build_image", return_value="sha256:exact")
    mocker.patch.object(container, "_generate_cdi_spec", return_value=tmp_path / "cdi")
    mocker.patch.object(
        container.subprocess,
        "run",
        return_value=subprocess.CompletedProcess(("podman",), 1),
    )
    output = tmp_path / "output"

    def command(result: Path) -> tuple[str, ...]:
        result.mkdir()
        (result / "run.json").write_text('{"status":"failed"}\n')
        return ("evaluate", str(result))

    exit_code = container.run_in_container(
        command,
        output=output,
        input_directories=(),
        forwarded_environment=(),
        environ={},
    )

    assert exit_code == 1
    assert (output / "run.json").read_text() == '{"status":"failed"}\n'
    assert not tuple(tmp_path.glob(".output-container-*"))


def test_success_without_output_is_rejected(
    mocker: MockerFixture,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(container, "CACHE_DIR", tmp_path / "cache")
    mocker.patch.object(container, "_podman_socket", return_value=tmp_path / "podman.sock")
    mocker.patch.object(container, "_build_image", return_value="sha256:exact")
    mocker.patch.object(container, "_generate_cdi_spec", return_value=tmp_path / "cdi")
    mocker.patch.object(
        container.subprocess,
        "run",
        return_value=subprocess.CompletedProcess(("podman",), 0),
    )

    with pytest.raises(container.ContainerEvaluationError, match="without output"):
        container.run_in_container(
            lambda _result: ("evaluate",),
            output=tmp_path / "output",
            input_directories=(),
            forwarded_environment=(),
            environ={},
        )


def test_command_failure_does_not_render_arguments(mocker: MockerFixture) -> None:
    mocker.patch.object(
        container.subprocess,
        "run",
        side_effect=subprocess.CalledProcessError(1, ("podman", "--env", "TOKEN=secret")),
    )

    with pytest.raises(container.ContainerEvaluationError) as error:
        container._checked_run(("podman", "--env", "TOKEN=secret"))

    assert str(error.value) == "podman command failed"
    assert "secret" not in str(error.value)
