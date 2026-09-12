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

"""A model with one Bash tool, isolated from DimOS and host services."""

from __future__ import annotations

from collections.abc import Generator
import json
from pathlib import Path
import shlex
import subprocess
import tempfile
from typing import Any

from dimos.evals.agents.lib.plain_recording import plain_recording
from dimos.evals.agents.pi import PiAdapter, PiAdapterConfig
from dimos.evals.environments.base import Environment
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


class BashOnlyConfig(PiAdapterConfig):
    tools: tuple[str, ...] = ("bash",)
    builtin_guidance: bool = False


class BashOnly(PiAdapter):
    """Pi supplies the stock model loop; only sandboxed Bash reaches the model.

    The shell has no network, host home, repository, credentials or DimOS.
    Recorded inputs use ordinary files. Images are attached to the initial
    model message; no image-reading tool or task-specific shell helper is added.
    """

    config: BashOnlyConfig

    def _check_policy(self) -> None:
        if self.config.tools != ("bash",) or self.config.skills or self.config.modules:
            raise ValueError("BashOnly requires tools=[bash], no skills and no modules")
        if self.config.builtin_guidance:
            raise ValueError("BashOnly cannot enable DimOS guidance")

    def preflight(self, environment: Environment) -> None:
        self._check_policy()
        if environment.has_robot:
            raise ValueError("BashOnly supports recordings; live tasks need a vendor SDK bridge")
        super().preflight(environment)
        if not Path("/usr/bin/bwrap").is_file():
            raise RuntimeError("BashOnly requires Linux bubblewrap at /usr/bin/bwrap")
        with tempfile.TemporaryDirectory(prefix="eval-bash-preflight-") as directory:
            root = Path(directory)
            result = subprocess.run(
                [
                    *sandbox_command(root, root),
                    "! command -v dimos && python3 -I -c "
                    "'import importlib.util; assert importlib.util.find_spec(\"dimos\") is None'",
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )
        if result.returncode:
            raise RuntimeError(f"BashOnly isolation preflight failed: {result.stderr.strip()}")

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        return ("bash",)

    def _prepare_files(self, env: RunningEnvironment, run_dir: Path) -> dict[str, Path]:
        self._check_policy()
        if env.mcp_url:
            raise ValueError("BashOnly cannot attach to DimOS MCP")
        plain_recording(env.streams, run_dir / "input")
        (run_dir / "workspace").mkdir()
        return {"manifest": Path("/input/manifest.json")}

    def _write_system_prompt(self, files: dict[str, Path], mcp_url: str, run_dir: Path) -> str:
        prompt = super()._write_system_prompt(files, "", run_dir)
        prompt += (
            "\n\nBash runs in an isolated Linux environment. Read /input/manifest.json for "
            "the selected sensor files and timestamps. /input is read-only; write work under "
            "/workspace. Ordinary Bash, grep, coreutils and system Python are available. "
            "There is no network or robotics framework. Selected camera PNGs are attached "
            "to this message in manifest order. Use only these observations."
        )
        (run_dir / "system-prompt.txt").write_text(prompt)
        return prompt

    def _build_process_env(self, run_dir: Path) -> dict[str, str]:
        return {
            k: v
            for k, v in super()._build_process_env(run_dir).items()
            if not k.startswith("DIMOS_")
        }

    def _build_pi_command(self, inputs: str, system_prompt: str, run_dir: Path) -> list[str]:
        self._check_policy()
        prefix = shlex.join(sandbox_command(run_dir / "input", run_dir / "workspace"))
        extension = run_dir / "sandbox.mjs"
        extension.write_text(
            'import { writeFileSync } from "node:fs";\n'
            'import { createBashToolDefinition } from "@earendil-works/pi-coding-agent";\n'
            'const quote = s => "\'" + s.replaceAll("\'", "\'\\"\'\\"\'") + "\'";\n'
            "export default function (pi) {\n"
            "  pi.registerTool(createBashToolDefinition(process.cwd(), {\n"
            '    shellPath: "/bin/bash", exposeSessionEnvironment: false,\n'
            "    spawnHook: ({ command, cwd }) => ({\n"
            f"      command: {json.dumps(prefix)} + ' ' + quote(command), cwd,\n"
            '      env: { PATH: "/usr/bin:/bin", HOME: "/tmp" },\n'
            "    }),\n"
            "  }));\n"
            f"  writeFileSync({json.dumps(str(run_dir / 'sandbox-ready'))}, 'ready');\n"
            "}\n"
        )
        command = super()._build_pi_command(inputs, system_prompt, run_dir)
        tool_index = command.index("--tools")
        del command[tool_index : tool_index + 2]
        # Explicit extensions still load with ambient extension discovery disabled.
        return [
            *command[:-1],
            "--no-builtin-tools",
            "--extension",
            str(extension),
            inputs,
            *(f"@{path}" for path in sorted((run_dir / "input").glob("*.png"))),
        ]

    def _process_events(
        self, proc: subprocess.Popen[bytes], run_dir: Path, deadline: float
    ) -> Generator[dict[str, Any], None, None]:
        yield from super()._process_events(proc, run_dir, deadline)
        if not (run_dir / "sandbox-ready").is_file():
            raise RuntimeError("Bash sandbox extension failed to load; see pi-stderr.txt")
