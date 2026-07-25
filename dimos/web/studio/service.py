# ruff: noqa: RUF001
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

from .mission import MissionController, MissionRecord
from .models import StudioSettings
from .stage2 import StageTwoControl

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
LIGHTWEIGHT_MCP_BLUEPRINT = "dimos-go2-studio.go2"
TELEOP_KEYCHAIN_SERVICE = "com.dimos.studio.teleop"
TELEOP_KEYCHAIN_ACCOUNT = "hosted-broker"
MISSION_CONFIRMATION = "START MISSION"


class StudioService:
    """Owns persistent settings and safe access to DimOS runtime controls."""

    def __init__(
        self,
        settings_path: Path = DEFAULT_SETTINGS_PATH,
        skill_path: Path = DEFAULT_SKILL_PATH,
        stage2_control: StageTwoControl | None = None,
    ) -> None:
        self.settings_path = settings_path
        self.skill_path = skill_path
        self.mission_path = settings_path.with_name("dimos-studio-mission.json")
        self._legacy_mission_controller: MissionController | None = None
        self.stage2_control = stage2_control or StageTwoControl(
            state_path=settings_path.with_name("dimos-studio-stage2.json")
        )
        self._launcher_processes: list[subprocess.Popen[str]] = []

    @property
    def mission_controller(self) -> MissionController:
        """Load the frozen legacy controller only for direct rollback use."""

        if self._legacy_mission_controller is None:
            self._legacy_mission_controller = MissionController(self._load_mission())
        return self._legacy_mission_controller

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

    def request_remote_stop(self) -> dict[str, str]:
        """Call movement cancellation tools directly, without LLM mediation."""

        adapter = McpAdapter.from_run_entry(timeout=5)
        results: dict[str, str] = {}
        errors: list[str] = []
        for tool_name in ("end_exploration", "stop_navigation"):
            try:
                results[tool_name] = adapter.call_tool_text(tool_name, {})
            except (requests.RequestException, McpError) as exc:
                errors.append(f"{tool_name}: {exc}")
        if errors:
            raise ValueError("; ".join(errors))
        return results

    def mission_status(self) -> dict[str, Any]:
        mission = self.mission_controller.mission
        return {
            "mission": mission.model_dump(mode="json") if mission else None,
            "policy": self.mission_controller.policy.model_dump(mode="json"),
        }

    def stage2_status(self) -> dict[str, Any]:
        return self.stage2_control.status()

    def stage2_navigate(
        self,
        instruction_id: str,
        destination: str,
    ) -> dict[str, Any]:
        return self.stage2_control.navigate(instruction_id, destination)

    def stage2_confirm_current_place(
        self,
        name: str,
        aliases: list[str],
    ) -> dict[str, Any]:
        return self.stage2_control.confirm_current_place(name, aliases)

    def stage2_cancel(self, task_id: str) -> dict[str, Any]:
        return self.stage2_control.cancel(task_id)

    def stage2_stop_all(self) -> dict[str, Any]:
        return self.stage2_control.stop_all()

    def stage2_reply(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self.stage2_control.record_reply(payload)

    def create_mission(self, objective: str) -> dict[str, Any]:
        mission = self.mission_controller.create(objective)
        self._save_mission(mission)
        return self.mission_status()

    def start_mission(self, confirmation: str) -> dict[str, Any]:
        if confirmation != MISSION_CONFIRMATION:
            raise ValueError(f"自主任务需要输入确认词: {MISSION_CONFIRMATION}")
        status = self.runtime_status()
        settings = self.load_settings()
        mission = self.mission_controller.start(
            runtime_running=bool(status.get("running")),
            movement_locked=settings.movement_locked,
        )
        self._save_mission(mission)
        try:
            agent_result = self.send_agent_message(self._mission_prompt(mission))
        except ValueError as exc:
            mission = self.mission_controller.pause(f"Agent 启动失败：{exc}")
            self._save_mission(mission)
            raise
        return {**self.mission_status(), "agent_result": agent_result}

    def pause_mission(self, reason: str = "") -> dict[str, Any]:
        mission = self.mission_controller.pause(reason or "操作员暂停")
        self._save_mission(mission)
        return {
            **self.mission_status(),
            **self._best_effort_agent_stop("暂停当前自主任务"),
        }

    def resume_mission(self, confirmation: str) -> dict[str, Any]:
        if confirmation != MISSION_CONFIRMATION:
            raise ValueError(f"恢复自主任务需要输入确认词: {MISSION_CONFIRMATION}")
        status = self.runtime_status()
        settings = self.load_settings()
        mission = self.mission_controller.resume(
            runtime_running=bool(status.get("running")),
            movement_locked=settings.movement_locked,
        )
        self._save_mission(mission)
        try:
            agent_result = self.send_agent_message(self._mission_prompt(mission))
        except ValueError as exc:
            mission = self.mission_controller.pause(f"Agent 恢复失败：{exc}")
            self._save_mission(mission)
            raise
        return {**self.mission_status(), "agent_result": agent_result}

    def stop_mission(self, reason: str = "") -> dict[str, Any]:
        mission = self.mission_controller.stop(reason or "操作员停止")
        self._save_mission(mission)
        return {
            **self.mission_status(),
            **self._best_effort_agent_stop("停止当前自主任务"),
        }

    def emergency_stop_mission(self, reason: str = "") -> dict[str, Any]:
        """Record E-STOP first, then request all Agent movement to stop.

        The returned confirmation flag matters: a local state update alone is
        not proof that the physical robot received the stop request.
        """

        mission = self.mission_controller.emergency_stop(reason or "操作员触发急停")
        self._save_mission(mission)
        return {
            **self.mission_status(),
            **self._best_effort_agent_stop("急停：立刻停止探索和导航"),
        }

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
            command.extend(["--rerun-open", "native", "--no-rerun-web"])
        command.extend(["--detection-model", settings.detection_model])
        command.extend(["--nerf-speed", str(settings.navigation_speed_scale)])
        command.append(
            "--obstacle-avoidance" if settings.obstacle_avoidance else "--no-obstacle-avoidance"
        )
        command.extend(["run", settings.blueprint])
        if settings.blueprint != LIGHTWEIGHT_MCP_BLUEPRINT:
            command.append("--daemon")
        if settings.blueprint != LIGHTWEIGHT_MCP_BLUEPRINT:
            command.extend(
                [
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
        if mode != "hosted_teleop":
            return environment
        key = self._load_teleop_key()
        if not key:
            raise ValueError("尚未保存 Teleop 密钥。")
        environment["TRANSPORTS__BROKER__API_KEY"] = key
        environment["TRANSPORTS__BROKER__ROBOT_NAME"] = settings.robot_name
        return environment

    def _best_effort_agent_stop(self, command: str) -> dict[str, Any]:
        try:
            result = self.request_remote_stop()
        except ValueError as exc:
            return {
                "remote_stop_confirmed": False,
                "remote_stop_error": f"{command}: {exc}",
            }
        return {
            "remote_stop_confirmed": True,
            "remote_stop_result": result,
        }

    @staticmethod
    def _mission_prompt(mission: MissionRecord) -> str:
        policy = mission.policy
        return f"""\
执行一个观察优先的 Go2 任务：{mission.objective}

必须遵守以下顺序和硬限制：
1. 先观察相机和语义空间记忆，再调用 begin_exploration 探索未知区域。
2. 通过视觉提出“门”候选；至少用多个独立画面确认，不确定就继续观察并报告。
3. 找到可信的门后，使用现有导航技能接近，但必须在 {policy.door_standoff_m:.1f} 米外停下。
4. 最大速度 {policy.max_speed_mps:.1f} m/s，探索半径不超过 {policy.exploration_radius_m:.0f} 米。
5. 人员进入 {policy.person_safety_radius_m:.1f} 米内时，立即停止导航和探索；连续安全
   {policy.clear_before_resume_s:.0f} 秒后才可恢复。
6. 连接异常、感知过期、电量低于 {policy.minimum_battery_percent:.0f}%、
   人工接管或连续 {policy.maximum_navigation_failures} 次导航失败时立即停止并报告。
7. 不能倒车，不能修改或绕过安全参数，不能直接发送 cmd_vel。

每次阶段变化都用简短中文报告：当前阶段、看到的证据、下一步和停止原因。
"""

    def _load_mission(self) -> MissionRecord | None:
        try:
            return MissionRecord.model_validate_json(
                self.mission_path.read_text(encoding="utf-8")
            )
        except (FileNotFoundError, ValueError, OSError):
            return None

    def _save_mission(self, mission: MissionRecord) -> None:
        self.mission_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = self.mission_path.with_suffix(".tmp")
        temporary_path.write_text(
            mission.model_dump_json(indent=2),
            encoding="utf-8",
        )
        temporary_path.replace(self.mission_path)

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
