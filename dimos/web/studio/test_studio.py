# ruff: noqa: RUF001
"""Tests for the safe, local DimOS Studio API."""

from pathlib import Path

from fastapi.testclient import TestClient
from pytest import MonkeyPatch

from dimos.web.studio.app import create_app
from dimos.web.studio.service import StudioService, validate_skill_source

VALID_SKILL = """\
from dimos.agents.annotation import skill
from dimos.core.module import Module

class TestSkills(Module):
    @skill
    def hello(self, name: str) -> str:
        \"\"\"Say hello without moving a robot.\"\"\"
        return f"hello {name}"
"""


def test_validate_skill_source() -> None:
    result = validate_skill_source(VALID_SKILL)
    assert result == {"valid": True, "errors": [], "skills": ["hello"]}


def test_reject_skill_without_annotations() -> None:
    result = validate_skill_source(VALID_SKILL.replace("name: str", "name").replace(" -> str", ""))
    assert result["valid"] is False
    assert len(result["errors"]) == 2


def test_studio_settings_and_skill_round_trip(tmp_path: Path) -> None:
    skill_path = tmp_path / "skills.py"
    skill_path.write_text(VALID_SKILL)
    app = create_app(tmp_path / "settings.json", skill_path)
    client = TestClient(app)

    assert client.get("/api/health").json()["ok"] is True
    settings = client.get("/api/settings").json()
    settings["agent_model"] = "ollama:qwen3:8b"
    assert client.put("/api/settings", json=settings).status_code == 200
    assert client.get("/api/settings").json()["agent_model"] == "ollama:qwen3:8b"

    skill = client.get("/api/skill").json()
    assert skill["skills"] == ["hello"]
    assert client.put("/api/skill", json={"source": skill["source"]}).status_code == 200


def test_hardware_start_requires_unlock_and_confirmation(tmp_path: Path) -> None:
    service = StudioService(
        settings_path=tmp_path / "settings.json",
        skill_path=tmp_path / "skills.py",
    )
    service.save_settings(service.load_settings())

    try:
        service.start_runtime("hardware", "START GO2")
    except ValueError as exc:
        assert "运动锁" in str(exc)
    else:
        raise AssertionError("hardware start must stay locked by default")


def test_command_preview_applies_model_and_speed_settings(tmp_path: Path) -> None:
    service = StudioService(
        settings_path=tmp_path / "settings.json",
        skill_path=tmp_path / "skills.py",
    )
    settings = service.load_settings().model_copy(
        update={"agent_model": "ollama:qwen3:8b", "navigation_speed_scale": 0.25}
    )
    service.save_settings(settings)

    command = service.command_preview("simulation")

    assert command[:3] == [service._dimos_executable(), "--simulation", "mujoco"]
    assert command[command.index("--nerf-speed") + 1] == "0.25"
    assert "mcpclient.model=ollama:qwen3:8b" in command


def test_readonly_hardware_command_disables_motion(tmp_path: Path) -> None:
    service = StudioService(
        settings_path=tmp_path / "settings.json",
        skill_path=tmp_path / "skills.py",
    )

    command = service.command_preview("hardware_readonly")

    assert "--simulation" not in command
    assert "go2connection.movement_enabled=false" in command
    assert "go2connection.auto_stand=false" in command


def test_parse_go2_discovery_output() -> None:
    output = """\
SOURCE NAME           IP              MAC                 SERIAL
LAN    -              30.201.217.128  -                   B42D1000PC4C1M86
"""

    robots = StudioService._parse_discovery_output(output)

    assert robots == [
        {
            "source": "LAN",
            "name": "",
            "ip": "30.201.217.128",
            "mac": "",
            "serial": "B42D1000PC4C1M86",
        }
    ]


def test_hosted_teleop_uses_keychain_key_only_in_environment(
    tmp_path: Path,
    monkeypatch: MonkeyPatch,
) -> None:
    service = StudioService(
        settings_path=tmp_path / "settings.json",
        skill_path=tmp_path / "skills.py",
    )
    settings = service.load_settings().model_copy(
        update={"robot_ip": "30.201.217.128", "robot_name": "Go2_Test"}
    )
    monkeypatch.setattr(service, "_load_teleop_key", lambda: "dtk_test_secret")

    command = service._build_command(settings, "hosted_teleop")
    environment = service._build_environment(settings, "hosted_teleop")

    assert "teleop-hosted-go2-transport" in command
    assert "go2connection.auto_stand=false" in command
    assert all("dtk_test_secret" not in argument for argument in command)
    assert environment["TRANSPORTS__BROKER__API_KEY"] == "dtk_test_secret"
    assert environment["TRANSPORTS__BROKER__ROBOT_NAME"] == "Go2_Test"
    assert "30.201.217.128" in environment["NO_PROXY"].split(",")


def test_mission_api_create_and_status(tmp_path: Path) -> None:
    skill_path = tmp_path / "skills.py"
    skill_path.write_text(VALID_SKILL)
    app = create_app(tmp_path / "settings.json", skill_path)
    client = TestClient(app)

    initial = client.get("/api/mission/status")
    assert initial.status_code == 200
    assert initial.json()["mission"] is None

    created = client.post(
        "/api/mission",
        json={"objective": "探索会场，找到门并在门前一米停下"},
    )

    assert created.status_code == 200
    assert created.json()["mission"]["state"] == "draft"
    assert client.get("/api/mission/status").json()["mission"]["objective"] == (
        "探索会场，找到门并在门前一米停下"
    )


def test_mission_start_requires_safety_gate_and_forwards_agent_prompt(
    tmp_path: Path,
    monkeypatch: MonkeyPatch,
) -> None:
    skill_path = tmp_path / "skills.py"
    skill_path.write_text(VALID_SKILL)
    app = create_app(tmp_path / "settings.json", skill_path)
    client = TestClient(app)
    service = app.state.studio_service
    client.post("/api/mission", json={"objective": "找到出口门"})

    locked = client.post(
        "/api/mission/start",
        json={"confirmation": "START MISSION"},
    )
    assert locked.status_code == 409
    assert "运动锁" in locked.json()["detail"]

    service.save_settings(service.load_settings().model_copy(update={"movement_locked": False}))
    monkeypatch.setattr(service, "runtime_status", lambda: {"running": True})
    sent: list[str] = []
    monkeypatch.setattr(service, "send_agent_message", lambda message: sent.append(message) or "ok")

    started = client.post(
        "/api/mission/start",
        json={"confirmation": "START MISSION"},
    )

    assert started.status_code == 200
    assert started.json()["mission"]["state"] == "running"
    assert sent
    assert "begin_exploration" in sent[0]
    assert "1.0 米" in sent[0]
    assert "不能修改或绕过安全参数" in sent[0]


def test_emergency_stop_is_recorded_when_mcp_is_unavailable(
    tmp_path: Path,
    monkeypatch: MonkeyPatch,
) -> None:
    skill_path = tmp_path / "skills.py"
    skill_path.write_text(VALID_SKILL)
    app = create_app(tmp_path / "settings.json", skill_path)
    client = TestClient(app)
    service = app.state.studio_service
    client.post("/api/mission", json={"objective": "找到门"})

    def unavailable(_message: str) -> str:
        raise ValueError("MCP unavailable")

    monkeypatch.setattr(service, "send_agent_message", unavailable)

    stopped = client.post(
        "/api/mission/estop",
        json={"reason": "operator test"},
    )

    assert stopped.status_code == 200
    assert stopped.json()["mission"]["state"] == "stopped"
    assert stopped.json()["mission"]["phase"] == "急停"
    assert stopped.json()["remote_stop_confirmed"] is False
    assert "MCP unavailable" in stopped.json()["remote_stop_error"]


def test_studio_has_unified_mission_control_page(tmp_path: Path) -> None:
    skill_path = tmp_path / "skills.py"
    skill_path.write_text(VALID_SKILL)
    client = TestClient(create_app(tmp_path / "settings.json", skill_path))

    html = client.get("/").text

    assert 'data-page="mission"' in html
    assert 'src="http://127.0.0.1:7779/"' in html
    assert 'id="mission-objective"' in html
    assert 'id="create-mission"' in html
    assert 'id="start-mission"' in html
    assert 'id="pause-mission"' in html
    assert 'id="resume-mission"' in html
    assert 'id="stop-mission"' in html
    assert 'id="mission-estop"' in html
    assert 'id="mission-state"' in html
    assert "人流礼让" in html
    assert "运动锁" in html
