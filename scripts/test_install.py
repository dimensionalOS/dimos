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

"""Hermetic installer journeys; no package manager or network access.

Run with: pytest -c /dev/null scripts/test_install.py
"""

import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys

import pytest

INSTALLER = Path(__file__).with_name("install.sh").resolve()
BASH = os.environ.get("INSTALLER_BASH", "/bin/bash")

# Mock only external tools. The real Bash installer and timeout supervisor run.
FAKE_TOOL = r"""
import json
import os
from pathlib import Path
import sys

name = Path(sys.argv[0]).name
args = sys.argv[1:]
with open(os.environ["COMMAND_LOG"], "a") as log:
    log.write(json.dumps([name, args, os.getcwd()]) + "\n")
if os.environ.get("FAIL_TOOL") == name:
    print("injected failure: " + name, file=sys.stderr)
    sys.exit(42)
if name == "uname":
    print(os.environ.get("TEST_OS", "Linux") if args == ["-s"] else os.environ.get("TEST_ARCH", "x86_64"))
elif name == "sw_vers":
    print("14.0")
elif name == "sysctl":
    print(17179869184 if "hw.memsize" in args else 67108864)
elif name == "df":
    print("Filesystem size used avail")
    print("disk 100 10 90")
elif name == "nvidia-smi":
    if os.environ.get("TEST_GPU") != "1": sys.exit(1)
    print("NVIDIA GPU CUDA Version: 12.8")
elif name == "nix":
    if args == ["--version"]:
        print("nix 2.35")
    elif "--command" in args:
        command = args[args.index("--command") + 1:]
        os.execvp(command[0], command)
elif name == "grep":
    if "/etc/os-release" in args: sys.exit(0)
    os.execv("/usr/bin/grep", ["/usr/bin/grep", *args])
elif name == "dpkg-query":
    if os.environ.get("TEST_PACKAGES_MISSING") == "1": sys.exit(1)
    print("install ok installed")
elif name == "brew":
    if os.environ.get("TEST_BREW_PREFIX"):
        assert os.environ["PATH"].startswith(os.environ["TEST_BREW_PREFIX"])
    if args[:2] == ["list", "--versions"]: sys.exit(1)
elif name == "curl":
    if "astral.sh/uv/install.sh" in args[-1]:
        print("#!/bin/sh\ncp \"$FAKE_UV\" \"$HOME/.local/bin/uv\"\nchmod +x \"$HOME/.local/bin/uv\"")
    elif "Homebrew" in args[-1]:
        print('test "${NONINTERACTIVE:-}" = 1\ntouch "$HOME/brew-installed"')
    elif "-o" in args:
        Path(args[args.index("-o") + 1]).write_text("flake")
elif name == "sudo":
    os.execvp(args[0], args)
elif name == "uv":
    if args == ["--version"]:
        print("uv " + ("0.9.25" if ".local/bin" in sys.argv[0] else os.environ.get("TEST_UV_VERSION", "0.9.25")))
    elif args[0] in ("venv", "sync"):
        bindir = Path(".venv/bin")
        bindir.mkdir(parents=True, exist_ok=True)
        for executable in ("python", "dimos"):
            target = bindir / executable
            target.write_text(Path(os.environ["FAKE_UV"]).read_text())
            target.chmod(0o755)
elif name == "git":
    if "clone" in args:
        (Path(args[-1]) / ".git").mkdir(parents=True)
    elif "rev-parse" in args:
        print("abc123" if "--short" in args else "true")
elif name == "python":
    if args[:1] != ["-c"]:
        os.execv(os.environ["REAL_PYTHON"], [os.environ["REAL_PYTHON"], *args])
"""


@pytest.fixture
def sandbox(tmp_path, monkeypatch):
    bindir = tmp_path / "bin"
    bindir.mkdir()
    home = tmp_path / "home"
    (home / ".local/bin").mkdir(parents=True)
    for key in list(os.environ):
        if key.startswith(("DIMOS_", "UV_")):
            monkeypatch.delenv(key)
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.setenv("PATH", f"{bindir}:/usr/bin:/bin")
    monkeypatch.setenv("COMMAND_LOG", str(tmp_path / "commands.jsonl"))
    monkeypatch.setenv("REAL_PYTHON", sys.executable)
    monkeypatch.setenv("FAKE_UV", str(bindir / "uv-template"))
    monkeypatch.setenv("DIMOS_PROJECT_DIR", str(tmp_path / "project"))
    for name in (
        "uv",
        "uv-template",
        "uname",
        "sw_vers",
        "sysctl",
        "df",
        "nvidia-smi",
        "nix",
        "grep",
        "dpkg-query",
        "brew",
        "curl",
        "git",
        "sudo",
        "apt-get",
    ):
        target = bindir / name
        target.write_text(f"#!{sys.executable}\n" + FAKE_TOOL)
        target.chmod(0o755)
    return tmp_path


def invoke(*args, streamed=False):
    command = [BASH, "-s", "--"] if streamed else [BASH, str(INSTALLER)]
    return subprocess.run(
        [*command, *args],
        input=INSTALLER.read_text() if streamed else "",
        text=True,
        capture_output=True,
        timeout=20,
        start_new_session=True,
    )


def shell(code):
    return subprocess.run(
        [BASH, "-c", f"source {shlex.quote(str(INSTALLER))}\n{code}"],
        text=True,
        capture_output=True,
        timeout=20,
        start_new_session=True,
    )


def commands(sandbox, name):
    entries = (sandbox / "commands.jsonl").read_text().splitlines()
    return [args for tool, args, _ in map(json.loads, entries) if tool == name]


@pytest.mark.parametrize("streamed", [False, True])
@pytest.mark.parametrize("mode", ["library", "dev"])
def test_install_and_rerun(sandbox, mode, streamed):
    args = (
        "--non-interactive",
        "--mode",
        mode,
        "--no-nix",
        "--no-sysctl",
        "--skip-tests",
        "--extras",
        "base",
        "--no-cuda",
    )
    first = invoke(*args, streamed=streamed)
    assert first.returncode == 0, first.stdout + first.stderr
    second = invoke(*args, streamed=streamed)
    assert second.returncode == 0, second.stdout + second.stderr
    assert "installation verified" in second.stdout
    assert ["--help"] in commands(sandbox, "dimos")
    assert ["list"] in commands(sandbox, "dimos")
    assert not any("pull" in args for args in commands(sandbox, "git"))


@pytest.mark.parametrize("streamed", [False, True])
def test_dependency_failure_cannot_report_success(sandbox, monkeypatch, streamed):
    monkeypatch.setenv("FAIL_TOOL", "uv")
    # Exercise the install command rather than the initial version probe.
    result = (
        shell(
            'INSTALL_DIR="$DIMOS_PROJECT_DIR"; mkdir -p "$INSTALL_DIR"; EXTRAS=base; NON_INTERACTIVE=1; do_install_library'
        )
        if not streamed
        else invoke("--non-interactive", "--no-nix")
    )
    assert result.returncode != 0
    assert "installation complete" not in result.stdout
    assert "injected failure: uv" in result.stderr


@pytest.mark.parametrize("extra", ["scene", "cuda"])
def test_arm_rejects_explicit_unavailable_capability(sandbox, monkeypatch, extra):
    monkeypatch.setenv("TEST_ARCH", "aarch64")
    result = invoke("--dry-run", "--no-nix", "--extras", extra)
    assert result.returncode != 0
    assert not any("pip" in args or "sync" in args for args in commands(sandbox, "uv"))


@pytest.mark.parametrize(
    "arch,system,backend,scene",
    [
        ("x86_64", "Linux", "cpu", True),
        ("aarch64", "Linux", "cpu", False),
        ("arm64", "Darwin", "cpu", True),
    ],
)
def test_all_selects_platform_dependencies(sandbox, monkeypatch, arch, system, backend, scene):
    monkeypatch.setenv("TEST_ARCH", arch)
    monkeypatch.setenv("TEST_OS", system)
    result = invoke("--dry-run", "--no-nix", "--mode", "dev", "--no-cuda")
    assert result.returncode == 0, result.stdout + result.stderr
    selection = next(line for line in result.stdout.splitlines() if "installing extras:" in line)
    selected = selection.split(": ", 1)[1].split(",")
    assert backend in selected
    assert "cuda" not in selected
    assert ("scene" in selected) == scene


def test_explicit_extras_used_in_developer_install(sandbox):
    result = invoke(
        "--non-interactive",
        "--no-nix",
        "--no-sysctl",
        "--skip-tests",
        "--mode",
        "dev",
        "--extras",
        "drone,cpu",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    sync = next(args for args in commands(sandbox, "uv") if args[0] == "sync")
    assert sync == [
        "sync",
        "--locked",
        "--python",
        "3.12",
        "--group",
        "tests",
        "--group",
        "lint",
        "--extra",
        "drone",
        "--extra",
        "cpu",
    ]


@pytest.mark.parametrize("no_cuda,backend", [(False, "cuda"), (True, "cpu")])
def test_nvidia_backend_selection(sandbox, monkeypatch, no_cuda, backend):
    monkeypatch.setenv("TEST_GPU", "1")
    extra_args = ["--no-cuda"] if no_cuda else []
    result = invoke("--dry-run", "--no-nix", "--extras", "base", *extra_args)
    assert result.returncode == 0, result.stdout + result.stderr
    assert f"installing extras: base,{backend}" in result.stdout


def test_paths_are_not_executed_as_shell_code(sandbox, monkeypatch):
    destination = sandbox / "a project's $(touch PWNED) folder"
    monkeypatch.setenv("DIMOS_PROJECT_DIR", str(destination))
    result = invoke(
        "--non-interactive", "--no-nix", "--no-sysctl", "--skip-tests", "--extras", "base"
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert (destination / ".venv/bin/python").exists()
    assert not (sandbox / "PWNED").exists()


def test_dry_run_never_invokes_mutating_commands(sandbox, monkeypatch):
    (sandbox / "bin/uv").unlink()
    monkeypatch.setenv("TEST_PACKAGES_MISSING", "1")
    result = invoke("--dry-run", "--no-nix")
    assert result.returncode == 0, result.stdout + result.stderr
    assert not (sandbox / "project").exists()
    assert commands(sandbox, "curl") == []
    assert commands(sandbox, "apt-get") == []


def test_old_uv_is_replaced_before_installation(sandbox, monkeypatch):
    monkeypatch.setenv("TEST_UV_VERSION", "0.9.24")
    result = invoke(
        "--non-interactive", "--no-nix", "--no-sysctl", "--skip-tests", "--extras", "base"
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert (sandbox / "home/.local/bin/uv").exists()
    assert any("astral.sh/uv/install.sh" in " ".join(args) for args in commands(sandbox, "curl"))


@pytest.mark.parametrize(
    "args,message",
    [
        (["--mode"], "requires a value"),
        (["--mode", "wrong"], "invalid mode"),
        (["--use-nix", "--no-nix"], "cannot be combined"),
        (["--bogus"], "unknown option"),
    ],
)
def test_invalid_arguments_fail_before_setup(sandbox, args, message):
    result = invoke(*args)
    assert result.returncode != 0
    assert message in result.stderr
    assert not (sandbox / "commands.jsonl").exists()


def test_no_terminal_explains_noninteractive_option(sandbox):
    result = invoke()
    assert result.returncode != 0
    assert "use --non-interactive" in result.stderr


def test_verification_failure_is_fatal(sandbox, monkeypatch):
    monkeypatch.setenv("FAIL_TOOL", "dimos")
    result = invoke(
        "--non-interactive", "--no-nix", "--no-sysctl", "--skip-tests", "--extras", "base"
    )
    assert result.returncode == 42
    assert "injected failure: dimos" in result.stderr
    assert "installation complete" not in result.stdout


def test_worktree_is_reused_without_pulling(sandbox):
    project = sandbox / "project"
    project.mkdir()
    (project / ".git").write_text("gitdir: /some/worktree")
    result = invoke("--dry-run", "--no-nix", "--mode", "dev")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "using existing checkout" in result.stdout
    assert not any("clone" in args or "pull" in args for args in commands(sandbox, "git"))


def test_nix_reuses_existing_virtualenv(sandbox):
    args = ("--non-interactive", "--use-nix", "--no-sysctl", "--skip-tests", "--extras", "base")
    first = invoke(*args)
    assert first.returncode == 0, first.stdout + first.stderr
    second = invoke(*args)
    assert second.returncode == 0, second.stdout + second.stderr
    assert len([args for args in commands(sandbox, "uv") if args[0] == "venv"]) == 1
    assert any(".venv/bin/dimos" in args for args in commands(sandbox, "nix"))


def test_failed_nix_verification_is_fatal(sandbox, monkeypatch):
    monkeypatch.setenv("FAIL_TOOL", "nix")
    result = shell('INSTALL_DIR="$HOME"; USE_NIX=1; verify_nix_develop')
    assert result.returncode != 0
    assert "nix develop verification failed" in result.stderr


def test_declining_nix_does_not_select_it(sandbox):
    result = shell(
        'HAS_NIX=0; USE_NIX=1; DETECTED_OS=ubuntu; prompt_confirm() { return 1; }; prompt_setup_method; printf "method=%s nix=%s\\n" "$SETUP_METHOD" "$USE_NIX"'
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "method=system nix=0" in result.stdout


def test_bounded_command_times_out_and_reaps_process(sandbox):
    project = sandbox / "project"
    (project / ".venv/bin").mkdir(parents=True)
    (project / ".venv/bin/python").symlink_to(sys.executable)
    code = "import os, signal; open('pid', 'w').write(str(os.getpid())); signal.pause()"
    result = shell(
        f'INSTALL_DIR="$DIMOS_PROJECT_DIR"; run_bounded 0.2 .venv/bin/python -c {shlex.quote(code)}'
    )
    assert result.returncode == 124, result.stdout + result.stderr
    pid = int((project / "pid").read_text())
    with pytest.raises(ProcessLookupError):
        os.kill(pid, signal.SIGCONT)


@pytest.mark.parametrize(
    "arch,prefix", [("arm64", "/opt/homebrew/bin:"), ("x86_64", "/usr/local/bin:")]
)
def test_fresh_homebrew_initializes_path_and_noninteractive_mode(
    sandbox, monkeypatch, arch, prefix
):
    monkeypatch.setenv("TEST_BREW_PREFIX", prefix)
    result = shell(f"""
DETECTED_OS=macos; DETECTED_ARCH={arch}; NON_INTERACTIVE=1
has_cmd() {{
    if [[ "$1" == brew && ! -f "$HOME/brew-installed" ]]; then return 1; fi
    command -v "$1" >/dev/null
}}
install_system_deps
""")
    assert result.returncode == 0, result.stdout + result.stderr
    assert (sandbox / "home/brew-installed").exists()
    assert any(args[0] == "install" for args in commands(sandbox, "brew"))


def test_system_package_failure_stops_installation(sandbox, monkeypatch):
    monkeypatch.setenv("TEST_PACKAGES_MISSING", "1")
    monkeypatch.setenv("FAIL_TOOL", "apt-get")
    result = shell("DETECTED_OS=ubuntu; NON_INTERACTIVE=1; install_system_deps")
    assert result.returncode == 42
    assert "system dependencies ready" not in result.stdout
    assert "injected failure: apt-get" in result.stderr


def test_no_cuda_overrides_explicit_cuda(sandbox):
    result = invoke("--dry-run", "--no-nix", "--extras", "cuda", "--no-cuda")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "installing extras: cpu" in result.stdout
