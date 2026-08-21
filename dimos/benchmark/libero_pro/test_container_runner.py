"""Tests for the locked outer LIBERO evaluation launcher."""

from pathlib import Path
import subprocess

import pytest

from dimos.benchmark.libero_pro import container_runner


def test_container_command_uses_immutable_image_and_explicit_authority(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    cache = tmp_path / "cache" / "dimos"
    monkeypatch.setattr(container_runner, "CACHE_DIR", cache)
    output = tmp_path / "output"
    scratch = output / ".runner-tmp-host"
    socket = tmp_path / "podman.sock"
    environ = {
        "OPENAI_API_KEY": "secret",
        "UNRELATED_SECRET": "do-not-forward",
        "EVO_RESULT_PATH": str(output / "result.json"),
    }

    command = container_runner._container_command(
        image_id="sha256:candidate",
        cdi_directory=tmp_path / "cdi",
        socket=socket,
        output=output,
        scratch=scratch,
        environ=environ,
        inner=("python", "-m", "benchmark"),
    )

    rendered = " ".join(command)
    assert "--network host" in rendered
    assert f"--cdi-spec-dir {tmp_path / 'cdi'}" in rendered
    assert "--device nvidia.com/gpu=all" in rendered
    assert f"{socket}:/run/podman/podman.sock:rw" in rendered
    assert f"{scratch}:/runner-tmp:rw" in rendered
    assert f"{output}:{output}:rw" in rendered
    assert f"{cache}:{cache}:rw" in rendered
    assert "--env OPENAI_API_KEY" in rendered
    assert "secret" not in rendered
    assert "UNRELATED_SECRET" not in rendered
    assert command[-4:] == ("sha256:candidate", "python", "-m", "benchmark")


def test_mount_directories_deduplicates_nested_evo_paths(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    cache = tmp_path / "cache"
    monkeypatch.setattr(container_runner, "CACHE_DIR", cache)
    output = tmp_path / "output"

    mounts = container_runner._mount_directories(
        output,
        {
            "EVO_RESULT_PATH": str(output / "evo" / "result.json"),
            "EVO_TRACES_DIR": str(output / "evo" / "traces"),
        },
    )

    assert mounts == (cache, output)


def test_mount_directories_rejects_host_root(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(container_runner, "CACHE_DIR", tmp_path / "cache")

    with pytest.raises(container_runner.ContainerEvaluationError, match="filesystem root"):
        container_runner._mount_directories(Path("/"), {})


def test_candidate_panel_must_be_baked_into_image(tmp_path: Path) -> None:
    with pytest.raises(container_runner.ContainerEvaluationError, match="checked-in"):
        container_runner._candidate_panel(tmp_path / "panel.json")


def test_build_image_runs_by_returned_image_id(mocker, tmp_path: Path) -> None:
    ensure = mocker.patch.object(container_runner.LiberoPodmanContainer, "ensure_image")

    def write_iid(command: tuple[str, ...]) -> None:
        Path(command[command.index("--iidfile") + 1]).write_text("sha256:exact\n")

    run = mocker.patch.object(container_runner, "_run", side_effect=write_iid)
    iid = tmp_path / "iid"

    image_id = container_runner._build_image(iid)

    assert image_id == "sha256:exact"
    assert not iid.exists()
    ensure.assert_called_once_with()
    assert "--iidfile" in run.call_args.args[0]


def test_generate_cdi_spec_is_scoped_to_scratch(mocker, tmp_path: Path) -> None:
    def write_spec(command: tuple[str, ...]) -> None:
        output = next(argument for argument in command if argument.startswith("--output="))
        Path(output.removeprefix("--output=")).write_text("cdiVersion: 0.7.0\n")

    run = mocker.patch.object(container_runner, "_run", side_effect=write_spec)

    directory = container_runner._generate_cdi_spec(tmp_path)

    assert directory == tmp_path / "cdi"
    assert (directory / "nvidia.yaml").is_file()
    assert run.call_args.args[0][:3] == ("nvidia-ctk", "cdi", "generate")


def test_command_failure_does_not_render_arguments(mocker) -> None:
    mocker.patch.object(
        container_runner.subprocess,
        "run",
        side_effect=subprocess.CalledProcessError(1, ("podman", "--env", "TOKEN=secret")),
    )

    with pytest.raises(container_runner.ContainerEvaluationError) as error:
        container_runner._run(("podman", "--env", "TOKEN=secret"))

    assert str(error.value) == "podman command failed"
    assert "secret" not in str(error.value)
