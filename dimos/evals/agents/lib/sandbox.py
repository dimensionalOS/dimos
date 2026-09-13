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

"""Selected recording files and Linux isolation for Pi tools."""

import json
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile

from dimos.evals.agents.lib.plain_recording import plain_recording
from dimos.evals.types import RunningEnvironment


def sandbox_command(inputs: Path, workspace: Path) -> list[str]:
    """Linux namespaces: ordinary system tools, selected inputs, private scratch."""
    return [
        "/usr/bin/bwrap", "--die-with-parent", "--new-session", "--unshare-all", "--clearenv",
        "--ro-bind", "/usr", "/usr", "--tmpfs", "/usr/local",
        "--symlink", "usr/bin", "/bin", "--symlink", "usr/lib", "/lib",
        "--symlink", "usr/lib64", "/lib64", "--proc", "/proc", "--dev", "/dev",
        "--tmpfs", "/tmp", "--ro-bind", str(inputs), "/input",
        "--bind", str(workspace), "/workspace", "--chdir", "/workspace",
        "--setenv", "PATH", "/usr/bin:/bin", "--setenv", "HOME", "/workspace",
        "--setenv", "LANG", "C.UTF-8", "/bin/bash", "--noprofile", "--norc", "-c",
    ]  # fmt: skip


def preflight() -> None:
    if not Path("/usr/bin/bwrap").is_file():
        raise RuntimeError("Tool isolation requires Linux bubblewrap at /usr/bin/bwrap")
    with tempfile.TemporaryDirectory(prefix="eval-sandbox-preflight-") as directory:
        root = Path(directory)
        result = subprocess.run(
            [
                *sandbox_command(root, root),
                "! command -v dimos && test -x /usr/bin/rg && python3 -I -c "
                "'import importlib.util; assert importlib.util.find_spec(\"dimos\") is None'",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )
    if result.returncode:
        raise RuntimeError(f"Tool isolation preflight failed: {result.stderr.strip()}")


def prepare_files(env: RunningEnvironment, run_dir: Path) -> dict[str, Path]:
    if env.mcp_url:
        raise ValueError("The sandbox cannot attach to DimOS MCP")
    plain_recording(env.streams, run_dir / "input")
    (run_dir / "workspace").mkdir()
    return {"manifest": Path("/input/manifest.json")}


GUIDANCE = (
    "Tools run in an isolated Linux environment. Read /input/manifest.json for "
    "the selected sensor files and timestamps. /input is read-only; write work under "
    "/workspace. Ordinary Bash, grep, coreutils and system Python are available. "
    "There is no network or robotics framework. Selected camera PNGs are attached "
    "to this message in manifest order. Use only these observations."
)


def extension(run_dir: Path, tools: tuple[str, ...]) -> Path:
    prefix = sandbox_command(run_dir / "input", run_dir / "workspace")
    (run_dir / "sandbox.json").write_text(json.dumps({"command": prefix, "tools": tools}))
    target = run_dir / "sandbox.js"
    shutil.copyfile(Path(__file__).with_suffix(".js"), target)
    # Pi's native grep spawns rg; both rg and its filesystem operations must be isolated.
    binary = run_dir / ".pi-agent" / "bin" / "rg"
    binary.parent.mkdir(parents=True, exist_ok=True)
    binary.write_text(
        "#!/bin/bash\nexec " + shlex.join(prefix) + ' \'exec /usr/bin/rg "$@"\' rg "$@"\n'
    )
    binary.chmod(0o755)
    return target
