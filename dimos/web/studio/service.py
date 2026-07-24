"""Core services for the local DimOS Studio workbench."""

from __future__ import annotations

import ast
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
from time import perf_counter
from typing import Any

import requests

from dimos.agents.mcp.mcp_adapter import McpAdapter, McpError
from dimos.constants import CONFIG_DIR, DIMOS_PROJECT_ROOT
from dimos.core.run_registry import get_most_recent

from .models import StudioSettings

DEFAULT_SETTINGS_PATH = CONFIG_DIR / "dimos-studio.json"
DEFAULT_SKILL_PATH = (
    DIMOS_PROJECT_ROOT
    / "extensions"
    / "go2-studio-agent"
    / "src"
    / "dimos_go2_studio"
    / "skills.py"
)
STUDIO_LOG_PATH = DIMOS_PROJECT_ROOT / "logs" / "dimos-studio-launch.log"
HARDWARE_CONFIRMATION = "START GO2"
TELEOP_CONFIRMATION = "START TELEOP"
TELEOP_URL = "https://teleop.dimensionalos.com/"
TELEOP_BLUEPRINT = "teleop-hosted-go2-transport"
TELEOP_KEYCHAIN_SERVICE = "com.dimos.studio.teleop"
TELEOP_KEYCHAIN_ACCOUNT = "hosted-broker"


class StudioService:
    """Owns persistent settings and safe access to DimOS runtime controls."""

    def __init__(
        self,
        settings_path: Path = DEFAULT_SETTINGS_PATH,
        skill_path: Path = DEFAULT_SKILL_PATH,
    ) -> None:
        self.settings_path = settings_path
        self.skill_path = skill_path
        self._launcher_processes: list[subprocess.Popen[str]] = []

    def load_settings(self) -> StudioSettings:
        try:
            payload = json.loads(self.settings_path.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            return StudioSettings()
        return StudioSettings.model_validate(payload)

    def save_settings(self, settings: StudioSettings) -> StudioSettings:
        self.settings_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = self.settings_path.with_suffix(".tmp")
        temporary_path.write_text(
            settings.model_dump_json(indent=2),
            encoding="utf-8",
        )
        temporary_path.replace(self.settings_path)
        return settings

    def robot_check(self, robot_ip: str, signal_port: int) -> dict[str, Any]:
        started = perf_counter()
        try:
            with socket.create_connection((robot_ip, signal_port), timeout=2.5):
                latency_ms = round((perf_counter() - started) * 1000, 1)
        except OSError as exc:
            return {
                "reachable": False,
                "latency_ms": None,
                "message": f"无法连接 {robot_ip}:{signal_port}: {exc}",
            }
        quality = "good" if latency_ms < 80 else "slow" if latency_ms < 200 else "poor"
        return {
            "reachable": True,
            "latency_ms": latency_ms,
            "quality": quality,
            "message": f"Go2 信令端口可达, 握手耗时 {latency_ms} ms",
        }

    def discover_robots(self, timeout_seconds: int = 12) -> dict[str, Any]:
        """Discover Go2 robots on the LAN without opening a control session."""

        command = [
            self._dimos_executable(),
            "go2tool",
            "discover",
            "--lan",
            "--timeout",
            str(timeout_seconds),
        ]
        try:
            result = subprocess.run(
                command,
                cwd=DIMOS_PROJECT_ROOT,
                capture_output=True,
                text=True,
                timeout=timeout_seconds + 8,
            )
        except subprocess.TimeoutExpired as exc:
            raise TimeoutError("机器狗发现超时, 请确认 Go2 已开机且电脑在同一网络。") from exc
        output = (result.stdout + result.stderr).strip()
        robots = self._parse_discovery_output(output)
        if result.returncode != 0 and not robots:
            raise ValueError(output or "机器狗发现失败。")
        if not robots:
            return {
                "found": False,
                "robots": [],
                "message": "没有发现机器狗, 请检查开机状态和电脑网络。",
            }
        return {
            "found": True,
            "robots": robots,
            "message": f"发现 {len(robots)} 台机器狗。",
        }

    def teleop_key_status(self) -> dict[str, Any]:
        key = self._load_teleop_key()
        return {
            "configured": bool(key),
            "storage": "macOS Keychain",
            "message": "Teleop 密钥已保存" if key else "尚未保存 Teleop 密钥",
        }

    def save_teleop_key(self, api_key: str) -> dict[str, Any]:
        key = api_key.strip()
        if not key.startswith("dtk_"):
            raise ValueError("这不是 Dimensional Teleop 密钥; 密钥应以 dtk_ 开头。")
        security = shutil.which("security")
        if security is None:
            raise ValueError("当前系统没有 macOS Keychain 工具, 无法安全保存密钥。")
        result = subprocess.run(
            [
                security,
                "add-generic-password",
                "-U",
                "-s",
                TELEOP_KEYCHAIN_SERVICE,
                "-a",
                TELEOP_KEYCHAIN_ACCOUNT,
                "-w",
                key,
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode != 0:
            raise ValueError(result.stderr.strip() or "Teleop 密钥保存失败。")
        return {
            "configured": True,
            "storage": "macOS Keychain",
            "message": "Teleop 密钥已安全保存。",
        }

    def runtime_status(self) -> dict[str, Any]:
        self._launcher_processes = [
            process for process in self._launcher_processes if process.poll() is None
        ]
        entry = get_most_recent(alive_only=True)
        if entry is None:
            if self._launcher_processes:
                process = self._launcher_processes[-1]
                return {
                    "running": False,
                    "starting": True,
                    "pid": process.pid,
                    "message": "DimOS 正在启动并连接机器狗…",
                    "log_tail": self._tail(STUDIO_LOG_PATH),
                }
            return {
                "running": False,
                "starting": False,
                "message": "DimOS 当前未运行",
                "log_tail": self._tail(STUDIO_LOG_PATH),
            }
        return {
            "running": True,
            "starting": False,
            "run_id": entry.run_id,
            "pid": entry.pid,
            "blueprint": entry.blueprint,
            "started_at": entry.started_at,
            "log_dir": entry.log_dir,
            "mcp_url": McpAdapter.from_run_entry(entry).url,
            "message": "DimOS 正在运行",
            "log_tail": self._tail(Path(entry.log_dir) / "main.jsonl"),
        }

    def command_preview(self, mode: str) -> list[str]:
        return self._build_command(self.load_settings(), mode)

    def start_runtime(self, mode: str, confirmation: str) -> dict[str, Any]:
        settings = self.load_settings()
        status = self.runtime_status()
        if status["running"] or status.get("starting"):
            raise ValueError("已有 DimOS 实例正在运行或启动, 请先停止它。")
        if mode == "hardware":
            if settings.movement_locked:
                raise ValueError("运动锁仍然开启。请先在“模型与参数”里关闭运动锁。")
            if confirmation != HARDWARE_CONFIRMATION:
                raise ValueError(f"真实机器启动需要输入确认词: {HARDWARE_CONFIRMATION}")
        if mode == "hosted_teleop":
            if confirmation != TELEOP_CONFIRMATION:
                raise ValueError(f"官方遥控启动需要输入确认词: {TELEOP_CONFIRMATION}")
            if not self._load_teleop_key():
                raise ValueError("尚未保存 Teleop 密钥。请先粘贴并保存密钥。")

        command = self._build_command(settings, mode)
        environment = self._build_environment(settings, mode)
        STUDIO_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        log_handle = STUDIO_LOG_PATH.open("a", encoding="utf-8")
        log_handle.write(f"\n[{datetime.now(timezone.utc).isoformat()}] {' '.join(command)}\n")
        log_handle.flush()
        try:
            process = subprocess.Popen(
                command,
                cwd=DIMOS_PROJECT_ROOT,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                env=environment,
            )
        finally:
            log_handle.close()
        self._launcher_processes.append(process)
        return {
            "starting": True,
            "pid": process.pid,
            "mode": mode,
            "command": command,
            "message": "启动请求已发送。连接真实 Go2 可能需要约 10-90 秒。",
        }

    def stop_runtime(self) -> dict[str, Any]:
        entry = get_most_recent(alive_only=True)
        if entry is None and self._launcher_processes:
            stopped_pids: list[int] = []
            for process in self._launcher_processes:
                if process.poll() is not None:
                    continue
                stopped_pids.append(process.pid)
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
                except ProcessLookupError:
                    pass
            self._launcher_processes = []
            return {
                "stopped": True,
                "pids": stopped_pids,
                "message": "DimOS 启动进程已停止。",
            }
        if entry is None:
            return {"stopped": True, "message": "DimOS 当前未运行。"}
        try:
            result = subprocess.run(
                [self._dimos_executable(), "stop"],
                cwd=DIMOS_PROJECT_ROOT,
                capture_output=True,
                text=True,
                timeout=20,
            )
        except subprocess.TimeoutExpired as exc:
            raise ValueError("停止 DimOS 超时, 请查看日志。") from exc
        output = (result.stdout + result.stderr).strip()
        if result.returncode != 0:
            raise ValueError(output or "停止 DimOS 失败")
        return {"stopped": True, "message": output or "DimOS 已停止"}

    def list_mcp_tools(self) -> list[dict[str, Any]]:
        adapter = McpAdapter.from_run_entry(timeout=5)
        try:
            return adapter.list_tools()
        except (requests.RequestException, McpError) as exc:
            raise ValueError(f"MCP 暂不可用: {exc}") from exc

    def send_agent_message(self, message: str) -> str:
        adapter = McpAdapter.from_run_entry(timeout=30)
        try:
            return adapter.call_tool_text("agent_send", {"message": message})
        except (requests.RequestException, McpError) as exc:
            raise ValueError(f"Agent 暂不可用: {exc}") from exc

    def read_skill_source(self) -> dict[str, Any]:
        source = self.skill_path.read_text(encoding="utf-8")
        validation = validate_skill_source(source)
        return {
            "path": str(self.skill_path),
            "source": source,
            **validation,
        }

    def save_skill_source(self, source: str) -> dict[str, Any]:
        validation = validate_skill_source(source)
        if not validation["valid"]:
            raise ValueError("\n".join(validation["errors"]))
        self.skill_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = self.skill_path.with_suffix(".tmp")
        temporary_path.write_text(source, encoding="utf-8")
        temporary_path.replace(self.skill_path)
        return {
            "saved": True,
            "path": str(self.skill_path),
            **validation,
            "message": "Skill 已保存; 重启 DimOS 后生效。",
        }

    def _build_command(self, settings: StudioSettings, mode: str) -> list[str]:
        if mode == "hosted_teleop":
            return [
                self._dimos_executable(),
                "--robot-ip",
                settings.robot_ip,
                "--viewer",
                "none",
                "run",
                TELEOP_BLUEPRINT,
                "--option",
                "go2connection.auto_stand=false",
            ]
        command = [self._dimos_executable()]
        if mode == "simulation":
            command.extend(["--simulation", "mujoco"])
        else:
            command.extend(["--robot-ip", settings.robot_ip])
        command.extend(["--viewer", settings.viewer])
        if settings.viewer == "rerun":
            command.extend(["--rerun-open", "web"])
        command.extend(["--detection-model", settings.detection_model])
        command.extend(["--nerf-speed", str(settings.navigation_speed_scale)])
        command.append(
            "--obstacle-avoidance" if settings.obstacle_avoidance else "--no-obstacle-avoidance"
        )
        command.extend(
            [
                "run",
                settings.blueprint,
                "--daemon",
                "--option",
                f"mcpclient.model={settings.agent_model}",
                "--option",
                f"mcpclient.system_prompt={settings.system_prompt}",
            ]
        )
        if mode == "hardware_readonly":
            command.extend(
                [
                    "--option",
                    "go2connection.movement_enabled=false",
                    "--option",
                    "go2connection.auto_stand=false",
                ]
            )
        return command

    def _build_environment(self, settings: StudioSettings, mode: str) -> dict[str, str]:
        environment = os.environ.copy()
        if mode != "hosted_teleop":
            return environment
        key = self._load_teleop_key()
        if not key:
            raise ValueError("尚未保存 Teleop 密钥。")
        environment["TRANSPORTS__BROKER__API_KEY"] = key
        environment["TRANSPORTS__BROKER__ROBOT_NAME"] = settings.robot_name
        no_proxy = [
            item.strip()
            for item in environment.get("NO_PROXY", environment.get("no_proxy", "")).split(",")
            if item.strip()
        ]
        for target in (settings.robot_ip, "localhost", "127.0.0.1"):
            if target not in no_proxy:
                no_proxy.append(target)
        environment["NO_PROXY"] = ",".join(no_proxy)
        environment["no_proxy"] = environment["NO_PROXY"]
        return environment

    @staticmethod
    def _parse_discovery_output(output: str) -> list[dict[str, str]]:
        robots: list[dict[str, str]] = []
        for line in output.splitlines():
            match = re.match(
                r"^\s*(LAN|BLE)\s+(\S+)\s+"
                r"(\d{1,3}(?:\.\d{1,3}){3})\s+(\S+)\s+(\S+)\s*$",
                line,
            )
            if not match:
                continue
            source, name, ip_address, mac_address, serial_number = match.groups()
            robots.append(
                {
                    "source": source,
                    "name": "" if name == "-" else name,
                    "ip": ip_address,
                    "mac": "" if mac_address == "-" else mac_address,
                    "serial": "" if serial_number == "-" else serial_number,
                }
            )
        return robots

    @staticmethod
    def _load_teleop_key() -> str:
        security = shutil.which("security")
        if security is None:
            return ""
        try:
            result = subprocess.run(
                [
                    security,
                    "find-generic-password",
                    "-s",
                    TELEOP_KEYCHAIN_SERVICE,
                    "-a",
                    TELEOP_KEYCHAIN_ACCOUNT,
                    "-w",
                ],
                capture_output=True,
                text=True,
                timeout=5,
            )
        except (OSError, subprocess.TimeoutExpired):
            return ""
        return result.stdout.strip() if result.returncode == 0 else ""

    @staticmethod
    def _dimos_executable() -> str:
        candidate = DIMOS_PROJECT_ROOT / ".venv" / "bin" / "dimos"
        return str(candidate) if candidate.exists() else "dimos"

    @staticmethod
    def _tail(path: Path, line_count: int = 40) -> str:
        try:
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            return ""
        return "\n".join(lines[-line_count:])


def validate_skill_source(source: str) -> dict[str, Any]:
    """Validate syntax and the public DimOS @skill contract without executing code."""

    errors: list[str] = []
    skills: list[str] = []
    try:
        tree = ast.parse(source)
    except SyntaxError as exc:
        return {
            "valid": False,
            "errors": [f"第 {exc.lineno} 行语法错误: {exc.msg}"],
            "skills": [],
        }

    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        is_skill = any(
            (isinstance(decorator, ast.Name) and decorator.id == "skill")
            or (
                isinstance(decorator, ast.Call)
                and isinstance(decorator.func, ast.Name)
                and decorator.func.id == "skill"
            )
            for decorator in node.decorator_list
        )
        if not is_skill:
            continue
        skills.append(node.name)
        if ast.get_docstring(node) is None:
            errors.append(f"{node.name}: @skill 必须有说明文档。")
        if node.returns is None:
            errors.append(f"{node.name}: @skill 必须声明返回类型。")
        arguments = [*node.args.posonlyargs, *node.args.args, *node.args.kwonlyargs]
        for argument in arguments:
            if argument.arg != "self" and argument.annotation is None:
                errors.append(f"{node.name}: 参数 {argument.arg} 必须声明类型。")

    if not skills:
        errors.append("没有找到任何 @skill 方法。")
    return {"valid": not errors, "errors": errors, "skills": skills}
