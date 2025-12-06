import re
import asyncio
from aiohttp import ClientSession
import logging
from typing import Optional

SESSION_LINE_RE = re.compile(r"^(.+?)\s+\[Created\s+(.+?)\s+ago\](.*)$")
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

async def is_zellij_running(zellij_url: str, session: ClientSession) -> bool:
    # TODO: make this check faster
    try:
        async with session.get(f"{zellij_url}/", timeout=2) as resp:
            return resp.status < 500
    except Exception:
        return False

async def start_zellij_process(zellij_port: str, log: logging.Logger, zellij_token_holder: dict) -> Optional[asyncio.subprocess.Process]:
    cmd = ["zellij", "web", "--port", str(zellij_port), ]
    log.info("Zellij not detected, starting: %s", " ".join(cmd))

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
                        log.info("Discovered zellij web token")
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
        log.error("zellij executable not found; please install zellij.")
    except Exception as exc:  # pragma: no cover - runtime failure
        log.error("Failed to start zellij web: %s", exc)
    return None

async def run_zellij_list_sessions(log: logging.Logger) -> dict:
    proc = await asyncio.create_subprocess_shell(
        "zellij list-sessions --no-formatting",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    stdout, stderr = await proc.communicate()
    stdout_text = stdout.decode()
    stderr_text = stderr.decode()

    if stderr_text:
        log.warning("zellij stderr: %s", stderr_text.strip())

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

    sessions = parse_zellij_sessions(stdout_text)
    log.info("Found %s sessions", len(sessions))
    return {"success": True, "sessions": sessions, "count": len(sessions)}