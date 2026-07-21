import os
from pathlib import Path
import subprocess
import sys

import pytest

from dimos.benchmark.spatial.pi_baseline.podman import PodmanRun, RootlessPodman
from dimos.benchmark.spatial.pi_baseline.topology import TopologyError, pin_runtime_topology


def topology(tmp_path: Path):
    paths = [tmp_path / name for name in ("input", "workspace", "output", "private")]
    for path in paths:
        path.mkdir()
    return pin_runtime_topology(
        input_dir=paths[0], workspace_dir=paths[1], output_dir=paths[2], private_dir=paths[3]
    )


def test_podman_mounts_use_validated_absolute_paths(tmp_path: Path) -> None:
    pinned = topology(tmp_path)
    request = PodmanRun("registry.example/pi@sha256:" + "a" * 64, "run-1", pinned)
    command = RootlessPodman().command(request)

    assert f"{pinned.input.path}:/input:ro,rprivate" in command
    assert f"{pinned.workspace.path}:/work:rw,rprivate" in command
    assert not any(argument.startswith("/proc/") for argument in command)
    pinned.close()


def test_replacement_does_not_change_pinned_identity(tmp_path: Path) -> None:
    pinned = topology(tmp_path)
    original = pinned.workspace.path
    replacement = tmp_path / "replacement"
    original.rename(tmp_path / "old-workspace")
    replacement.mkdir()
    os.symlink(replacement, original)

    pinned.verify()
    assert os.fstat(pinned.workspace.fd).st_ino != replacement.stat().st_ino
    pinned.close()


def test_inherited_fd_reads_pinned_directory_after_path_replacement(tmp_path: Path) -> None:
    workspace = tmp_path / "workspace"
    workspace.mkdir()
    (workspace / "sentinel").write_text("pinned", encoding="utf-8")
    for path in (tmp_path / "input", tmp_path / "output", tmp_path / "private"):
        path.mkdir()
    pinned = pin_runtime_topology(
        input_dir=tmp_path / "input",
        workspace_dir=workspace,
        output_dir=tmp_path / "output",
        private_dir=tmp_path / "private",
    )
    try:
        child = subprocess.Popen(
            [
                sys.executable,
                "-c",
                "import sys; sys.stdin.readline(); "
                f"print(open('/proc/self/fd/{pinned.workspace.fd}/sentinel').read())",
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            pass_fds=(pinned.workspace.fd,),
        )
        workspace.rename(tmp_path / "old-workspace")
        replacement = tmp_path / "replacement"
        replacement.mkdir()
        (replacement / "sentinel").write_text("replacement", encoding="utf-8")
        os.symlink(replacement, workspace)
        stdout, stderr = child.communicate("go\n", timeout=10)
        assert child.returncode == 0, stderr
        assert stdout.strip() == "pinned"
    finally:
        pinned.close()


def test_ancestor_and_duplicate_roots_are_rejected(tmp_path: Path) -> None:
    root = tmp_path / "root"
    root.mkdir()
    with pytest.raises(TopologyError):
        pin_runtime_topology(
            input_dir=root,
            workspace_dir=root / "workspace",
            output_dir=tmp_path / "output",
            private_dir=tmp_path / "private",
        )


def test_symlinked_ancestor_is_rejected(tmp_path: Path) -> None:
    real = tmp_path / "real"
    real.mkdir()
    for name in ("input", "workspace", "output", "private"):
        (real / name).mkdir()
    alias = tmp_path / "alias"
    alias.symlink_to(real, target_is_directory=True)

    with pytest.raises(TopologyError):
        pin_runtime_topology(
            input_dir=alias / "input",
            workspace_dir=alias / "workspace",
            output_dir=alias / "output",
            private_dir=alias / "private",
        )


def test_non_directory_root_is_rejected(tmp_path: Path) -> None:
    input_file = tmp_path / "input"
    input_file.write_text("not a directory", encoding="utf-8")
    for name in ("workspace", "output", "private"):
        (tmp_path / name).mkdir()

    with pytest.raises(TopologyError):
        pin_runtime_topology(
            input_dir=input_file,
            workspace_dir=tmp_path / "workspace",
            output_dir=tmp_path / "output",
            private_dir=tmp_path / "private",
        )
