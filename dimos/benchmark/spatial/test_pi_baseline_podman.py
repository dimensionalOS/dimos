from io import BytesIO, StringIO, TextIOWrapper
from pathlib import Path
import subprocess
from threading import Event
from unittest.mock import Mock, patch

import pytest

import dimos.benchmark.spatial.pi_baseline.podman as podman_module
from dimos.benchmark.spatial.pi_baseline.podman import (
    PodmanInfrastructureError,
    PodmanRun,
    PodmanSecurityError,
    RootlessPodman,
)
from dimos.benchmark.spatial.pi_baseline.topology import pin_runtime_topology


@pytest.fixture(autouse=True)
def _mock_cancellable_command(
    monkeypatch: pytest.MonkeyPatch, request: pytest.FixtureRequest
) -> None:
    if request.node.name.startswith(
        (
            "test_cancellation_during_each_runtime_command",
            "test_podman_timeout",
            "test_podman_drains",
        )
    ):
        return

    def run(
        command: list[str],
        *,
        check: bool,
        timeout: float,
        cancel_requested: Event,
        operation: podman_module.PodmanOperation,
        **_: object,
    ) -> subprocess.CompletedProcess[str]:
        if cancel_requested.is_set():
            raise podman_module.ExecutionInterrupted
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        if check and isinstance(result.returncode, int) and result.returncode:
            raise subprocess.CalledProcessError(
                result.returncode, command, output=result.stdout, stderr=result.stderr
            )
        if command[1:2] == ["inspect"]:
            result.stdout = "true\n"
        elif _is_bootstrap(command):
            result.stdout = ""
            result.stderr = ""
        elif _is_readiness(command):
            script = command[-1]
            assert "#!/agent/venv/bin/python\\n" in script
            assert "mount_options('/input')" in script
            assert "mount_options('/work')" in script
            assert "mount_options('/agent')" in script
            assert "/input/.pi_probe" not in script
            result.stdout = "READY\n"
            result.stderr = ""
        return result

    monkeypatch.setattr(podman_module, "_run_command", run)


def request(tmp_path: Path) -> PodmanRun:
    paths = [tmp_path / name for name in ("in", "work", "out", "private")]
    for path in paths:
        path.mkdir()
    return PodmanRun(
        "registry.example/pi@sha256:" + "a" * 64,
        "run-1",
        pin_runtime_topology(
            input_dir=paths[0], workspace_dir=paths[1], output_dir=paths[2], private_dir=paths[3]
        ),
    )


def _is_bootstrap(command: list[str]) -> bool:
    return (
        command[1:2] == ["exec"]
        and "python -m venv --system-site-packages /agent/venv" in command[-1]
    )


def _is_readiness(command: list[str]) -> bool:
    return command[1:2] == ["exec"] and "print('READY')" in command[-1]


def test_command_has_sandbox_and_only_expected_mounts(tmp_path: Path) -> None:
    command = RootlessPodman().command(request(tmp_path))
    rendered = " ".join(command)
    assert "--cap-drop=ALL" in command
    assert "--security-opt=no-new-privileges" in command
    assert "--userns=keep-id:uid=1000,gid=1000" in command
    assert "/agent:rw,size=512m,mode=1777,exec,nosuid,nodev" in command
    assert not any(
        argument.startswith("/agent:") and ("uid=1000" in argument or "gid=1000" in argument)
        for argument in command
    )
    assert not any(argument.startswith("--network") for argument in command)
    assert "--tmpfs" in command
    assert "/tmp:rw,size=64m,mode=1777" in command
    assert ":/input:ro,rprivate" in rendered and ":/work:rw,rprivate" in rendered
    assert "docker.sock" not in rendered
    assert "sha256:" in rendered


def test_commands_use_validated_absolute_path_mounts(tmp_path: Path) -> None:
    podman = RootlessPodman()
    runtime_request = request(tmp_path)

    command = podman.command(runtime_request)
    create_command = podman.persistent(runtime_request, Event()).create_command()
    expected_input = f"{runtime_request.topology.input.path}:/input:ro,rprivate"
    expected_workspace = f"{runtime_request.topology.workspace.path}:/work:rw,rprivate"

    for rendered_command in (command, create_command):
        assert "--userns=keep-id:uid=1000,gid=1000" in rendered_command
        assert expected_input in rendered_command
        assert expected_workspace in rendered_command
        assert not any(argument.startswith("/proc/") for argument in rendered_command)


def test_create_entrypoint_override_precedes_image_and_keeper_follows_image(tmp_path: Path) -> None:
    runtime_request = request(tmp_path)
    command = RootlessPodman().persistent(runtime_request, Event()).create_command()
    image_index = command.index(runtime_request.image)
    assert command.index("--entrypoint") < image_index
    assert command[image_index + 1 :] == ["-c", "exec sleep infinity"]


def test_requires_digest_and_distinct_workspace(tmp_path: Path) -> None:
    with pytest.raises(PodmanSecurityError):
        RootlessPodman().command(PodmanRun("image:latest", "run-1", request(tmp_path).topology))


def test_podman_timeout_is_typed_and_does_not_retain_command_details(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class HangingProcess:
        def poll(self) -> None:
            return None

        def terminate(self) -> None:
            return None

        def wait(self, timeout: float) -> int:
            return 0

    monkeypatch.setattr(podman_module.subprocess, "Popen", lambda *args, **kwargs: HangingProcess())
    monotonic = iter((0.0, 2.0))
    monkeypatch.setattr(podman_module.time, "monotonic", lambda: next(monotonic))

    command = ["podman", "exec", "/private/credential", "secret-output"]
    with pytest.raises(podman_module.PodmanTimeoutError) as raised:
        podman_module._run_command(
            command,
            check=False,
            timeout=1.0,
            cancel_requested=Event(),
            operation="exec",
        )
    assert raised.value.operation == "exec"
    assert str(raised.value) == "podman operation timed out"
    assert "/private/credential" not in str(raised.value)
    assert "secret-output" not in str(raised.value)


def test_podman_drains_and_bounds_stdout_and_stderr_concurrently(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class FinishedProcess:
        returncode = 0
        stdout = StringIO("o" * 100_000)
        stderr = StringIO("e" * 100_000)

        def poll(self) -> int:
            return 0

        def wait(self, timeout: float) -> int:
            return 0

    monkeypatch.setattr(
        podman_module.subprocess, "Popen", lambda *args, **kwargs: FinishedProcess()
    )
    result = podman_module._run_command(
        ["podman", "exec"],
        check=False,
        timeout=1.0,
        capture_limit=128,
        cancel_requested=Event(),
        operation="exec",
    )
    assert len(result.stdout.encode()) <= 128
    assert len(result.stderr.encode()) <= 128
    assert result.stdout_truncated is True
    assert result.stderr_truncated is True


def test_podman_drains_invalid_utf8_with_replacement(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class FinishedProcess:
        returncode = 0

        def __init__(self, *, encoding: str, errors: str) -> None:
            self.stdout = TextIOWrapper(
                BytesIO(b"before\xf5after"), encoding=encoding, errors=errors
            )
            self.stderr = StringIO("")

        def poll(self) -> int:
            return 0

        def wait(self, timeout: float) -> int:
            return 0

    monkeypatch.setattr(
        podman_module.subprocess,
        "Popen",
        lambda *args, **kwargs: FinishedProcess(
            encoding=kwargs.get("encoding", "utf-8"),
            errors=kwargs.get("errors", "strict"),
        ),
    )

    result = podman_module._run_command(
        ["podman", "exec"],
        check=False,
        timeout=1.0,
        cancel_requested=Event(),
        operation="exec",
    )

    assert result.stdout == "before\ufffdafter"
    assert result.stdout_truncated is False


@pytest.mark.parametrize(
    ("controller_deadline", "timeout", "expected_source"),
    [(-1.0, 10.0, "controller_deadline"), (10.0, 1.0, "command_cap")],
)
def test_podman_timeout_composes_controller_deadline_and_command_cap(
    monkeypatch: pytest.MonkeyPatch,
    controller_deadline: float,
    timeout: float,
    expected_source: str,
) -> None:
    class HangingProcess:
        stdout = None
        stderr = None

        def poll(self) -> None:
            return None

        def terminate(self) -> None:
            return None

        def wait(self, timeout: float) -> int:
            return 0

    monkeypatch.setattr(podman_module.subprocess, "Popen", lambda *args, **kwargs: HangingProcess())
    monotonic = iter((0.0, 2.0))
    monkeypatch.setattr(podman_module.time, "monotonic", lambda: next(monotonic))
    with pytest.raises(podman_module.PodmanTimeoutError) as raised:
        podman_module._run_command(
            ["podman", "exec"],
            check=False,
            timeout=timeout,
            controller_deadline=controller_deadline,
            cancel_requested=Event(),
            operation="exec",
        )
    assert raised.value.deadline_source == expected_source


def test_exec_timeout_poisons_case_and_forces_removal(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    runtime_request = request(tmp_path)
    case = RootlessPodman().persistent(runtime_request, Event())
    case._started = True  # type: ignore[attr-defined]
    operations: list[str] = []

    def command(*args: object, **kwargs: object) -> podman_module.PodmanCompletedProcess:
        operation = kwargs["operation"]
        assert isinstance(operation, str)
        operations.append(operation)
        if operation == "inspect":
            return podman_module.PodmanCompletedProcess([], 0, "true\n", "")
        if operation == "exec":
            raise podman_module.PodmanTimeoutError("exec", "command_cap")
        return podman_module.PodmanCompletedProcess(
            [], 1 if operation == "exists_cleanup" else 0, "", ""
        )

    monkeypatch.setattr(podman_module, "_run_command", command)
    with pytest.raises(podman_module.PodmanTimeoutError):
        case.exec("sleep forever")
    assert case.poisoned
    assert "remove_cleanup" in operations
    assert "exists_cleanup" in operations
    with pytest.raises(RuntimeError, match="not running"):
        case.exec("should not run")


@pytest.mark.parametrize("stage", ["info", "create", "start", "exec", "logs"])
def test_cancellation_during_each_runtime_command_terminates_client(
    monkeypatch: pytest.MonkeyPatch, stage: str, tmp_path: Path
) -> None:
    cancel_requested = Event()
    started: list[str] = []
    cleanup: list[list[str]] = []

    class Client:
        returncode = None
        stdout = None
        stderr = None

        def __init__(self, command: list[str], **__: object) -> None:
            self.command = command
            self.stage = command[1]
            self.returncode = 0
            started.append(self.stage)
            if command[1:3] == ["container", "exists"]:
                self.returncode = 1
            if self.stage == stage and not (_is_bootstrap(command) or _is_readiness(command)):
                self.returncode = None
                cancel_requested.set()

        def poll(self) -> int | None:
            return self.returncode

        def terminate(self) -> None:
            self.returncode = -15

        def wait(self, timeout: float) -> int:
            return self.returncode or 0

        def communicate(self) -> tuple[str, str]:
            if _is_readiness(self.command):
                return ("READY\n", "")
            return ("true\n" if self.stage in {"info", "inspect"} else "", "")

    monkeypatch.setattr(podman_module.subprocess, "Popen", Client)

    def cleanup_run(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        cleanup.append(command)
        if command[1:3] == ["container", "exists"]:
            return subprocess.CompletedProcess(command, 1, "", "")
        return subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(podman_module.subprocess, "run", cleanup_run)
    adapter = RootlessPodman()
    case = adapter.persistent(request(tmp_path), cancel_requested)
    if stage in {"info", "create", "start"}:
        with pytest.raises(podman_module.ExecutionInterrupted):
            with case:
                pass
    else:
        with case as active:
            with pytest.raises(podman_module.ExecutionInterrupted):
                if stage == "exec":
                    active.exec("sleep infinity")
                else:
                    active.logs()

    assert stage in started
    assert "rm" in started
    assert "container" in started


@patch("dimos.benchmark.spatial.pi_baseline.podman.subprocess.run")
def test_failure_unconditionally_removes_container(mock_run: Mock, tmp_path: Path) -> None:
    mock_run.side_effect = [Mock(stdout="true\n"), RuntimeError("failed"), Mock()]
    with pytest.raises(RuntimeError):
        RootlessPodman().run(request(tmp_path), Event())
    assert mock_run.call_args_list[-1].args[0][1:3] == ["rm", "--force"]


@patch("dimos.benchmark.spatial.pi_baseline.podman.subprocess.run")
def test_persistent_case_reuses_one_container_and_collects_logs(
    mock_run: Mock, tmp_path: Path
) -> None:
    def result(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        if command[1:3] == ["container", "exists"]:
            return subprocess.CompletedProcess(command, 1, "", "")
        stdout = (
            "true\n"
            if command[1:2] in (["info"], ["inspect"])
            or _is_bootstrap(command)
            or _is_readiness(command)
            else "output\n"
        )
        return subprocess.CompletedProcess(command, 0, stdout, "")

    mock_run.side_effect = result
    adapter = RootlessPodman()
    with adapter.persistent(request(tmp_path), Event()) as case:
        case.exec("python analysis.py")
        case.exec("uv pip install package")
        logs = case.logs()

    commands = [call.args[0] for call in mock_run.call_args_list]
    assert sum(command[1:2] == ["create"] for command in commands) == 1
    assert sum(command[1:2] == ["start"] for command in commands) == 1
    exec_commands = [
        command
        for command in commands
        if command[1:2] == ["exec"] and not _is_bootstrap(command) and not _is_readiness(command)
    ]
    assert len(exec_commands) == 2
    assert all(command[-3:-1] == ["sh", "-c"] for command in exec_commands)
    create_command = next(command for command in commands if command[1:2] == ["create"])
    assert "/tmp:rw,size=64m,mode=1777" in create_command
    assert "/agent:rw,size=512m,mode=1777,exec,nosuid,nodev" in create_command
    assert not any(
        argument.startswith("/agent:") and ("uid=1000" in argument or "gid=1000" in argument)
        for argument in create_command
    )
    assert "--http-proxy=false" in create_command
    bootstrap_index = next(i for i, command in enumerate(commands) if _is_bootstrap(command))
    readiness_index = next(i for i, command in enumerate(commands) if _is_readiness(command))
    assert bootstrap_index < readiness_index
    assert commands[readiness_index][-2] == "-c"
    assert sum(argument == "--volume" for argument in create_command) == 2
    assert "docker.sock" not in " ".join(create_command)
    assert all(command[0] == "podman" for command in commands)
    assert logs.stdout == "output\n"
    assert commands[-2][1:2] == ["rm"]
    assert commands[-2][2:5] == ["--force", "--time", "0"]
    assert commands[-1][1:3] == ["container", "exists"]


@patch("dimos.benchmark.spatial.pi_baseline.podman.subprocess.run")
def test_persistent_timeout_immediately_removes_container(mock_run: Mock, tmp_path: Path) -> None:
    def responder(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        if command[1:2] == ["info"] or command[1:2] == ["inspect"]:
            return subprocess.CompletedProcess(command, 0, "true\n", "")
        if _is_bootstrap(command):
            return subprocess.CompletedProcess(command, 0, "", "")
        if _is_readiness(command):
            return subprocess.CompletedProcess(command, 0, "READY\n", "")
            return subprocess.CompletedProcess(command, 0, "", "")
        if command[1:2] == ["exec"]:
            raise podman_module.PodmanTimeoutError("exec", "command_cap")
        if command[1:3] == ["container", "exists"]:
            return subprocess.CompletedProcess(command, 1, "", "")
        return subprocess.CompletedProcess(command, 0, "", "")

    mock_run.side_effect = responder
    with pytest.raises(TimeoutError):
        with RootlessPodman().persistent(request(tmp_path), Event()) as case:
            case.exec("long analysis")
    assert mock_run.call_args_list[-2].args[0][1:2] == ["rm"]
    assert mock_run.call_args_list[-2].args[0][2:5] == ["--force", "--time", "0"]
    assert mock_run.call_args_list[-1].args[0][1:3] == ["container", "exists"]


@patch("dimos.benchmark.spatial.pi_baseline.podman.subprocess.run")
@patch("dimos.benchmark.spatial.pi_baseline.podman._CLEANUP_TIMEOUT_SECONDS", 0.01)
def test_persistent_cleanup_reports_unremovable_container(mock_run: Mock, tmp_path: Path) -> None:
    def responder(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        if command[1:2] == ["info"]:
            return subprocess.CompletedProcess(command, 0, "true\n", "")
        if command[1:3] == ["container", "exists"]:
            return subprocess.CompletedProcess(command, 0, "", "")
        return subprocess.CompletedProcess(command, 0, "", "")

    mock_run.side_effect = responder
    with pytest.raises(RuntimeError, match="failed to remove container"):
        with RootlessPodman().persistent(request(tmp_path), Event()):
            pass


@patch("dimos.benchmark.spatial.pi_baseline.podman.subprocess.run")
def test_persistent_exec_propagates_nonzero_result_without_host_exception(
    mock_run: Mock, tmp_path: Path
) -> None:
    def responder(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        if command[1:2] == ["info"]:
            return subprocess.CompletedProcess(command, 0, "true\n", "")
        if _is_bootstrap(command):
            return subprocess.CompletedProcess(command, 0, "", "")
        if _is_readiness(command):
            return subprocess.CompletedProcess(command, 0, "READY\n", "")
            return subprocess.CompletedProcess(command, 0, "", "")
        if command[1:2] == ["exec"]:
            return subprocess.CompletedProcess(command, 17, "partial output\n", "analysis failed\n")
        if command[1:2] == ["inspect"]:
            return subprocess.CompletedProcess(command, 0, "true\n", "")
        if command[1:3] == ["container", "exists"]:
            return subprocess.CompletedProcess(command, 1, "", "")
        return subprocess.CompletedProcess(command, 0, "", "")

    mock_run.side_effect = responder
    with RootlessPodman().persistent(request(tmp_path), Event()) as case:
        completed = case.exec("false")

    assert completed.returncode == 17
    assert completed.stdout == "partial output\n"
    assert completed.stderr == "analysis failed\n"
    exec_call = next(call for call in mock_run.call_args_list if call.args[0][1:2] == ["exec"])
    assert exec_call.kwargs["check"] is False


def _lifecycle_result(
    command: list[str], *, inspect: subprocess.CompletedProcess[str], exec_code: int = 0
) -> subprocess.CompletedProcess[str]:
    operation = command[1:2]
    if operation == ["info"]:
        return subprocess.CompletedProcess(command, 0, "true\n", "")
    if operation == ["inspect"]:
        return inspect
    if operation == ["exec"]:
        if _is_bootstrap(command):
            return subprocess.CompletedProcess(command, 0, "", "")
        if _is_readiness(command):
            return subprocess.CompletedProcess(command, 0, "READY\n", "")
        return subprocess.CompletedProcess(command, exec_code, "secret stdout", "secret stderr")
    if command[1:3] == ["container", "exists"]:
        return subprocess.CompletedProcess(command, 1, "", "")
    return subprocess.CompletedProcess(command, 0, "", "")


def test_start_running_false_is_terminal_and_cleans_up(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    calls: list[str] = []

    def run(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        calls.append(command[1])
        return _lifecycle_result(
            command, inspect=subprocess.CompletedProcess(command, 0, "false\n", "")
        )

    monkeypatch.setattr(podman_module, "_run_command", run)
    case = RootlessPodman().persistent(request(tmp_path), Event())
    with pytest.raises(PodmanInfrastructureError) as raised:
        with case:
            pytest.fail("stopped container must not enter")
    assert raised.value.operation == "inspect"
    assert case._started is False  # type: ignore[attr-defined]
    assert "rm" in calls and "container" in calls


@pytest.mark.parametrize(
    ("stdout", "returncode"),
    [("TRUE\n", 0), ("True\n", 0), ("", 0), (" \t\n", 0), ("true false\n", 0), ("true\n", 125)],
)
def test_non_exact_true_inspect_output_is_terminal(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, stdout: str, returncode: int
) -> None:
    def run(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        if command[1:2] == ["inspect"]:
            return subprocess.CompletedProcess(command, returncode, stdout, "spoof")
        return _lifecycle_result(
            command, inspect=subprocess.CompletedProcess(command, 0, "true\n", "")
        )

    monkeypatch.setattr(podman_module, "_run_command", run)
    case = RootlessPodman().persistent(request(tmp_path), Event())
    with pytest.raises(PodmanInfrastructureError) as raised:
        with case:
            pass
    assert raised.value.operation == "inspect"
    assert raised.value.returncode == returncode


@pytest.mark.parametrize("stdout", ["true", "true\n"])
def test_inspect_running_output_accepts_normalized_true(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, stdout: str
) -> None:
    def run(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        return _lifecycle_result(
            command, inspect=subprocess.CompletedProcess(command, 0, stdout, "")
        )

    monkeypatch.setattr(podman_module, "_run_command", run)
    with RootlessPodman().persistent(request(tmp_path), Event()):
        pass


def test_readiness_probe_has_executable_probe_and_mount_semantics() -> None:
    script = podman_module._readiness_script()
    assert "agent.write_text('#!/agent/venv/bin/python\\n" in script
    assert "def mount_options(mountpoint):" in script
    assert "unescape_mountinfo" in script
    assert "mount_options('/input')" in script
    assert "Path('/input/.pi_probe')" not in script


def test_readiness_probe_checks_container_identity_and_concrete_dimos_module() -> None:
    script = podman_module._readiness_script()
    assert "os.getuid() == 1000 and os.getgid() == 1000" in script
    assert "import dimos.core.module" in script
    assert "module_file = dimos.core.module.__file__" in script
    assert "module_file is not None" in script
    assert "assert Path(origin).is_relative_to(global_site)" in script
    assert "CapEff:" in script and "NoNewPrivs:" in script
    assert "mount_type('/agent') == 'tmpfs'" in script
    assert "agent_stat.st_mode & 0o1777 == 0o1777" in script
    assert "private_stat.st_uid == 1000 and private_stat.st_gid == 1000" in script
    assert "private_stat.st_mode & 0o777 == 0o700" in script
    assert "not any(Path('/work').iterdir())" in script
    assert any(
        "umask 077" in constant
        for constant in podman_module.PersistentPodmanCase._bootstrap_agent_environment.__code__.co_consts
        if isinstance(constant, str)
    )


@pytest.mark.parametrize("readiness_code", [1, 125])
def test_readiness_nonzero_is_terminal_and_cleans_up(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, readiness_code: int
) -> None:
    def run(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        if _is_readiness(command):
            return subprocess.CompletedProcess(command, readiness_code, "", "not ready")
        return _lifecycle_result(
            command, inspect=subprocess.CompletedProcess(command, 0, "true\n", "")
        )

    monkeypatch.setattr(podman_module, "_run_command", run)
    case = RootlessPodman().persistent(request(tmp_path), Event())
    with pytest.raises(PodmanInfrastructureError) as raised:
        with case:
            pass
    assert raised.value.operation == "readiness"
    assert raised.value.returncode == readiness_code
    assert case._started is False  # type: ignore[attr-defined]


@pytest.mark.parametrize("phase", ["pre", "post"])
def test_exec_stopped_container_is_terminal_and_cleans_up(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, phase: str
) -> None:
    inspect_calls = 0

    def run(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        nonlocal inspect_calls
        if command[1:2] == ["inspect"]:
            inspect_calls += 1
            running = inspect_calls < (2 if phase == "pre" else 3)
            return subprocess.CompletedProcess(command, 0, "true\n" if running else "false\n", "")
        return _lifecycle_result(
            command, inspect=subprocess.CompletedProcess(command, 0, "true\n", "")
        )

    monkeypatch.setattr(podman_module, "_run_command", run)
    case = RootlessPodman().persistent(request(tmp_path), Event())
    with pytest.raises(PodmanInfrastructureError):
        with case as active:
            active.exec("printf safe")
    assert case.poisoned


def test_user_exec_125_is_terminal(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    def run(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        return _lifecycle_result(
            command,
            inspect=subprocess.CompletedProcess(command, 0, "true\n", ""),
            exec_code=125,
        )

    monkeypatch.setattr(podman_module, "_run_command", run)
    with pytest.raises(PodmanInfrastructureError) as raised:
        with RootlessPodman().persistent(request(tmp_path), Event()) as case:
            case.exec("user command")
    assert raised.value.operation == "exec"
    assert raised.value.returncode == 125


def test_user_exec_oserror_is_terminal_safe_and_cleaned_up(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    operations: list[str] = []

    def run(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        operation = "exists_cleanup" if command[1:3] == ["container", "exists"] else command[1]
        operations.append(operation)
        if command[1:2] == ["exec"] and not _is_bootstrap(command) and not _is_readiness(command):
            raise OSError("secret command/stdout/stderr/path")
        return _lifecycle_result(
            command, inspect=subprocess.CompletedProcess(command, 0, "true\n", "")
        )

    monkeypatch.setattr(podman_module, "_run_command", run)
    case = RootlessPodman().persistent(request(tmp_path), Event())
    with pytest.raises(PodmanInfrastructureError) as raised:
        with case as active:
            active.exec("user command")
    assert raised.value.operation == "exec"
    assert raised.value.returncode is None
    assert str(raised.value) == "container_runtime_failed"
    assert case.poisoned
    assert case._started is False  # type: ignore[attr-defined]
    assert "rm" in operations and "exists_cleanup" in operations


@pytest.mark.parametrize("exit_code", [17, 126, 127])
def test_ordinary_inner_exit_codes_are_recoverable(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, exit_code: int
) -> None:
    def run(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        return _lifecycle_result(
            command,
            inspect=subprocess.CompletedProcess(command, 0, "true\n", ""),
            exec_code=exit_code,
        )

    monkeypatch.setattr(podman_module, "_run_command", run)
    with RootlessPodman().persistent(request(tmp_path), Event()) as case:
        assert case.exec("ordinary").returncode == exit_code


def test_infrastructure_exception_is_safe_and_bounded() -> None:
    error = PodmanInfrastructureError("exec", 125)
    assert str(error) == "container_runtime_failed"
    assert error.reason == "container_runtime_failed"
    assert error.operation == "exec"
    assert error.returncode == 125
    assert vars(error) == {"operation": "exec", "returncode": 125}
    assert "exec" not in str(error) and "125" not in str(error)
    assert "command" not in str(error) and "stdout" not in str(error) and "stderr" not in str(error)
    assert "path" not in str(error)
