# Copyright 2025 Dimensional Inc.
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

import asyncio
import json
import logging
import re
import shutil
import subprocess
from typing import Optional

from aiohttp import ClientSession

SESSION_LINE_RE = re.compile(r"^(.+?)\s+\[Created\s+(.+?)\s+ago\](.*)$")


class ZellijManager:
    def __init__(
        self,
        *,
        log: logging.Logger,
        session_name: str,
        port: str,
        terminal_commands: dict[str, str],
        zellij_layout: str | None = None,
        token: str | None = None,
    ):
        # TODO: add https_key_path and https_cert_path
        self.log = log
        self.session_name = session_name + "-" + str(hash(json.dumps(terminal_commands)))
        self.port = port
        self.token = token
        self.terminal_commands = terminal_commands
        self.zellij_layout = zellij_layout
        self.enabled = zellij_layout or (terminal_commands and len(terminal_commands.keys()) > 0)
        if self.enabled and shutil.which("zellij") is None:
            self.log.error(
                "zellij executable not found; cannot open terminals in Dashboard %s", session_name
            )
            self.enabled = False

        if self.zellij_layout and terminal_commands:
            self.log.warning(
                "Both zellij_layout and terminal_commands are set; ignoring terminal_commands in favor of zellij_layout"
            )
            self.terminal_commands = {}
        self.log.info(f"Zellij enabled? {self.enabled}")
        if self.enabled:
            ZellijManager.init_zellij_session(
                self.log, self.session_name, self.terminal_commands, self.zellij_layout
            )

    @staticmethod
    def parse_zellij_sessions(output: str):
        sessions = []
        for line in output.strip().splitlines():
            line = line.strip()
            if not line:
                continue
            match = SESSION_LINE_RE.match(line)
            if match:
                session_name = match.group(1).strip()
                created_ago = match.group(2).strip()
                additional = match.group(3).strip()
                sessions.append(
                    {
                        "name": session_name,
                        "createdAgo": created_ago,
                        "status": additional or "active",
                        "raw": line,
                    }
                )
        return sessions

    @staticmethod
    def is_server_online() -> bool:
        try:
            result = subprocess.run(
                ["zellij", "web", "--status"],
                check=True,
                capture_output=True,
                text=True,
            )
        except FileNotFoundError as exc:
            raise RuntimeError("zellij not found on PATH") from exc

        # Example offline: "Web server is offline, checked: http://127.0.0.1:8082"
        return "online" in result.stdout.lower()

    async def start_zellij_server(
        self, zellij_token_holder: dict
    ) -> asyncio.subprocess.Process | None:
        if ZellijManager.is_server_online():
            return None

        cmd = [
            "zellij",
            "web",
            "--port",
            str(self.port),
        ]
        self.log.info("Zellij not detected, starting: %s", " ".join(cmd))

        token_re = re.compile(
            r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}", re.IGNORECASE
        )

        async def capture_token():
            if zellij_token_holder["token"]:
                return
            proc = await asyncio.create_subprocess_exec(
                *["zellij", "web", "--create-token"],
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            streams = [proc.stdout, proc.stderr]
            for stream in streams:
                if stream is None:
                    continue
                try:
                    while True:
                        line = await asyncio.wait_for(stream.readline(), timeout=3)
                        if not line:
                            break
                        text = line.decode(errors="ignore")
                        match = token_re.search(text)
                        if match:
                            zellij_token_holder["token"] = match.group(0)
                            self.log.info("Discovered zellij web token")
                            return
                except asyncio.TimeoutError:
                    continue

        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            await capture_token()
            return proc
        except FileNotFoundError:
            self.log.error("zellij executable not found; please install zellij.")
        except Exception as exc:  # pragma: no cover - runtime failure
            self.log.error("Failed to start zellij web: %s", exc)
        return None

    async def run_zellij_list_sessions(self) -> dict:
        proc = await asyncio.create_subprocess_shell(
            "zellij list-sessions --no-formatting",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        stdout, stderr = await proc.communicate()
        stdout_text = stdout.decode()
        stderr_text = stderr.decode()

        if stderr_text:
            self.log.warning("zellij stderr: %s", stderr_text.strip())

        if proc.returncode != 0:
            if proc.returncode == 1 and not stdout_text.strip():
                return {
                    "success": True,
                    "sessions": [],
                    "count": 0,
                    "message": "No active zellij sessions found",
                }
            raise RuntimeError(
                f"zellij list-sessions failed (code {proc.returncode}): {stderr_text or stdout_text}"
            )

        sessions = ZellijManager.parse_zellij_sessions(stdout_text)
        self.log.info("Found %s sessions", len(sessions))
        return {"success": True, "sessions": sessions, "count": len(sessions)}

    def init_zellij_session(
        self: logging.Logger,
        session_name: str,
        terminal_commands: dict[str, str],
        zellij_layout: str | None = None,
    ):
        #
        # stop old session (if any)
        #
        try:
            self.info(f"Killing old session {session_name}")
            subprocess.run(
                ["zellij", "kill-session", session_name],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError:
            self.error("zellij executable not found; cannot manage session %s", session_name)
            # break
        except Exception as exc:
            self.warning("Unable to kill session %s: %s", session_name, exc)
        #
        # write layout to tmp file
        #
        zellij_path = (
            f"/tmp/.zellij_layout.{session_name}.{hash(json.dumps(terminal_commands))}.kdl"
        )
        print(f"""zellij_path = {zellij_path}""")
        try:
            self.info(f"Writing zellij layout {session_name}")
            if not zellij_layout:
                files_to_run = []
                for command in terminal_commands.values():
                    sanitized_command = re.sub(r"[^A-Za-z\s_\-\=\*]", "", command)
                    file_path = f"/tmp/{sanitized_command}.sh"
                    with open(file_path, "w") as file:
                        file.write(f"""
                            . .envrc
                            . source.ignore.sh
                            . ./venv/bin/activate
                            {command}
                        """)
                    files_to_run.append(file_path)
                zellij_layout = (
                    """
                    layout {
                        """
                    + "\n".join(
                        f"""pane command=\"zsh\" {{
                                args "{file_path}"
                            }}"""
                        for file_path in files_to_run
                    )
                    + """
                    }
                """
                )
            with open(zellij_path, "w") as file:
                file.write(zellij_layout)
        except Exception as exc:
            self.error("Failed to write zellij layout: %s", exc)
            return

        #
        # start the current session with web sharing and layout
        #
        try:
            subprocess.Popen(
                # zellij attach --create-background my-session-name options --default-layout
                # ["zellij", "attach", "--create-background", session_name, "options", "--web-sharing=on",],
                [
                    "zellij",
                    "attach",
                    "--force-run-commands",
                    "--create-background",
                    session_name,
                    "options",
                    "--web-sharing=on",
                    "--default-layout",
                    zellij_path,
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        except Exception as exc:
            self.error("Failed to start zellij session %s: %s", session_name, exc)
