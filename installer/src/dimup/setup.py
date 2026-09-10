# Copyright 2026 Dimensional Inc.
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

"""Machine prerequisites. Python belongs to uv, native libraries to the OS."""

import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile

from dimup.process import Runner, SetupError, executable

APT_PACKAGES = (
    "build-essential",
    "cmake",
    "ninja-build",
    "pkg-config",
    "git",
    "git-lfs",
    "curl",
    "ca-certificates",
    "portaudio19-dev",
    "libsndfile1-dev",
    "libturbojpeg0-dev",
    "ffmpeg",
    "libgl1",
    "libegl1",
    "libglib2.0-0",
    "libusb-1.0-0-dev",
    "libssl-dev",
    "clang",
    "libclang-dev",
)
BREW_PACKAGES = (
    "cmake",
    "ninja",
    "pkg-config",
    "git",
    "git-lfs",
    "portaudio",
    "libsndfile",
    "jpeg-turbo",
    "ffmpeg",
    "libusb",
    "openssl@3",
    "deno",
)


def supported_platform() -> str | None:
    system, machine = platform.system(), platform.machine()
    if system == "Linux" and machine == "x86_64":
        release = platform.freedesktop_os_release()
        if release.get("ID") == "ubuntu" and release.get("VERSION_ID") in {"22.04", "24.04"}:
            return "ubuntu"
    if system == "Darwin" and machine == "arm64" and int(platform.mac_ver()[0].split(".")[0]) >= 14:
        return "macos"
    return None


def install_script(runner: Runner, url: str, args: list[str], env: dict[str, str]) -> None:
    with tempfile.TemporaryDirectory(prefix="dimup-") as directory:
        script = Path(directory) / "install.sh"
        runner.run(
            "Download installer",
            [
                "curl",
                "--fail",
                "--show-error",
                "--location",
                "--proto",
                "=https",
                "--tlsv1.2",
                "--connect-timeout",
                "30",
                "--max-time",
                "180",
                url,
                "--output",
                str(script),
            ],
        )
        runner.run("Install tool", ["bash", str(script), *args], env=env)


def available(name: str) -> bool:
    try:
        executable(name)
    except SetupError:
        return False
    return True


def prepare(runner: Runner) -> None:
    target = supported_platform()
    if target is None:
        print(
            f"Automatic setup is not available for {platform.system()} {platform.machine()}.\n"
            "Prepare these prerequisites manually with your system package manager:\n"
            "  Tools: uv, Git, Git LFS, Cargo/Rust, Nix, Deno, C/C++ compiler,\n"
            "         CMake, Ninja, pkg-config, curl, and CA certificates.\n"
            "  Libraries and development headers: PortAudio, libsndfile, libjpeg-turbo,\n"
            "         FFmpeg, OpenGL/EGL, GLib, libusb, OpenSSL, and Clang/libclang.\n"
            "Enable nix-command and flakes in Nix, and put the tools on PATH.\n"
            "Once ready, continue with: dimup init my-robot\n"
            "Automatic setup is available on Ubuntu 22.04/24.04 x86_64 and Apple Silicon macOS 14+."
        )
        return
    env = dict(os.environ)
    env["PATH"] = os.pathsep.join(
        [
            str(Path.home() / ".local/bin"),
            str(Path.home() / ".cargo/bin"),
            "/opt/homebrew/bin",
            env.get("PATH", ""),
        ]
    )
    # Authenticate before redirecting installer output so password prompts remain visible.
    if os.geteuid() != 0:
        result = subprocess.run(
            ["sudo", "-v"] if sys.stdin.isatty() else ["sudo", "-n", "true"], check=False
        )
        if result.returncode:
            raise SetupError(
                "Machine setup needs administrator access. Run sudo -v, then dimup setup."
            )
    if target == "ubuntu":
        prefix = [] if os.geteuid() == 0 else ["sudo"]
        runner.run("Refresh Ubuntu packages", [*prefix, "apt-get", "update"])
        runner.run(
            "Install host prerequisites", [*prefix, "apt-get", "install", "-y", *APT_PACKAGES]
        )
    else:
        runner.run(
            "Check Xcode Command Line Tools (install with xcode-select --install)",
            ["xcode-select", "-p"],
        )
        if not available("brew"):
            install_script(
                runner,
                "https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh",
                [],
                {**env, "NONINTERACTIVE": "1"},
            )
        runner.run(
            "Install host prerequisites", [executable("brew"), "install", *BREW_PACKAGES], env=env
        )
    if not available("cargo"):
        install_script(
            runner, "https://sh.rustup.rs", ["-y", "--no-modify-path", "--profile", "minimal"], env
        )
    if not available("deno"):
        install_script(
            runner,
            "https://deno.land/install.sh",
            ["-y", "--no-modify-path"],
            {**env, "DENO_INSTALL": str(Path.home() / ".local")},
        )
    if not available("nix"):
        install_script(runner, "https://nixos.org/nix/install", ["--daemon", "--yes"], env)
    for name in ("uv", "cargo", "deno", "git", "cmake", "pkg-config"):
        runner.run(f"Check {name}", [executable(name), "--version"])
    runner.run(
        "Check Nix store",
        [executable("nix"), "--extra-experimental-features", "nix-command flakes", "store", "info"],
    )
    print(f"Machine setup complete. Log: {runner.log}")
