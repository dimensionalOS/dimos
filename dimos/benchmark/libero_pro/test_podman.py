"""Hermetic tests for the rootless Podman lifecycle."""

from pathlib import Path
from subprocess import CompletedProcess

from pytest_mock import MockerFixture

from dimos.benchmark.libero_pro.assets import PreparedAssets
from dimos.benchmark.libero_pro.models import LiberoTaskManifest
from dimos.benchmark.libero_pro.podman import IMAGE, LiberoPodmanContainer

CASE = Path(__file__).parent / "cases" / "goal-task-0-single-trial" / "task.json"


def test_container_uses_podman_with_isolated_ports_and_read_only_assets(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    manifest = LiberoTaskManifest.model_validate_json(CASE.read_bytes())
    assets = PreparedAssets(bddl=tmp_path / "task.bddl", init_states=tmp_path / "states.pt")
    assets.bddl.touch()
    assets.init_states.touch()
    calls: list[list[str]] = []

    def run(command: list[str], **_kwargs: object) -> CompletedProcess[str]:
        calls.append(command)
        if command[1] == "port":
            port = "41001" if command[-1] == "50051/tcp" else "41002"
            return CompletedProcess(command, 0, f"127.0.0.1:{port}\n", "")
        return CompletedProcess(command, 0, "container-id\n", "")

    mocker.patch("subprocess.run", side_effect=run)
    container = LiberoPodmanContainer(manifest, assets, artifact_dir=tmp_path / "artifacts")

    endpoints = container.start()
    container.stop()

    run_command = calls[0]
    assert run_command[:3] == ["podman", "run", "--detach"]
    assert "--rm" in run_command
    assert "127.0.0.1::50051" in run_command
    assert "127.0.0.1::50052" in run_command
    assert f"{assets.bddl}:/task/task.bddl:ro,Z" in run_command
    assert f"{assets.init_states}:/task/init_states.pruned_init:ro,Z" in run_command
    assert run_command[-1] == IMAGE
    assert endpoints.policy == "127.0.0.1:41001"
    assert endpoints.control == "127.0.0.1:41002"
    assert endpoints.control_token
    assert calls[-2][:3] == ["podman", "stop", "--time"]
    assert calls[-1][:3] == ["podman", "rm", "--force"]


def test_existing_image_skips_build(mocker: MockerFixture) -> None:
    run = mocker.patch(
        "subprocess.run",
        return_value=CompletedProcess(["podman"], 0, "", ""),
    )

    LiberoPodmanContainer.ensure_image()

    run.assert_called_once_with(["podman", "image", "exists", IMAGE], check=False)
