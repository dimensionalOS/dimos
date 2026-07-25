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


def test_command_preview_uses_native_viewer_without_embedded_model(tmp_path: Path) -> None:
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
    assert not any(item.startswith("mcpclient.") for item in command)
    assert "--daemon" not in command
    assert command[command.index("--rerun-open") + 1] == "native"
    assert "--no-rerun-web" in command


def test_readonly_hardware_command_disables_motion(tmp_path: Path) -> None:
    service = StudioService(
        settings_path=tmp_path / "settings.json",
        skill_path=tmp_path / "skills.py",
    )

    command = service.command_preview("hardware_readonly")

    assert "--simulation" not in command
    assert "go2connection.movement_enabled=false" in command
    assert "go2connection.auto_stand=false" in command


def test_local_runtime_bypasses_proxy_for_robot_and_mcp(tmp_path: Path) -> None:
    service = StudioService(
        settings_path=tmp_path / "settings.json",
        skill_path=tmp_path / "skills.py",
    )
    settings = service.load_settings().model_copy(update={"robot_ip": "192.168.12.1"})

    environment = service._build_environment(settings, "hardware_readonly")

    assert {"localhost", "127.0.0.1", "192.168.12.1"} <= set(
        environment["NO_PROXY"].split(",")
    )
    assert environment["no_proxy"] == environment["NO_PROXY"]


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


def test_legacy_mission_api_is_not_mounted_or_loaded(tmp_path: Path) -> None:
    skill_path = tmp_path / "skills.py"
    skill_path.write_text(VALID_SKILL)
    app = create_app(tmp_path / "settings.json", skill_path)
    client = TestClient(app)

    assert client.get("/api/mission/status").status_code == 404
    assert client.post("/api/mission", json={"objective": "找到门"}).status_code == 404
    assert app.state.studio_service._legacy_mission_controller is None
    assert not (tmp_path / "dimos-studio-mission.json").exists()


def test_studio_has_thin_stage2_control_page(tmp_path: Path) -> None:
    skill_path = tmp_path / "skills.py"
    skill_path.write_text(VALID_SKILL)
    client = TestClient(create_app(tmp_path / "settings.json", skill_path))

    html = client.get("/").text

    assert 'data-page="mission"' in html
    assert 'src="http://127.0.0.1:7779/"' in html
    assert 'id="mission-estop"' in html
    assert 'id="stage2-map-id"' in html
    assert 'id="stage2-places"' in html
    assert 'id="stage2-place-name"' in html
    assert 'id="stage2-confirm-place"' in html
    assert 'id="stage2-task-id"' in html
    assert 'id="stage2-task-state"' in html
    assert 'id="stage2-destination"' in html
    assert 'id="stage2-recovery"' in html
    assert 'id="stage2-navigate"' in html
    assert 'id="stage2-cancel"' in html
    assert 'id="mission-objective"' not in html
    assert 'id="create-mission"' not in html
    assert 'id="stage2-trajectory"' not in html
    assert "Studio 只负责地点命名、路线选择和发送任务" in html


def test_stage2_api_delegates_to_the_single_supervision_adapter(
    tmp_path: Path,
    monkeypatch: MonkeyPatch,
) -> None:
    skill_path = tmp_path / "skills.py"
    skill_path.write_text(VALID_SKILL)
    app = create_app(tmp_path / "settings.json", skill_path)
    client = TestClient(app)
    control = app.state.studio_service.stage2_control
    calls: list[tuple[str, object]] = []

    monkeypatch.setattr(
        control,
        "status",
        lambda: {
            "connected": True,
            "semantic_world": {
                "map_id": "venue-a",
                "map_version": "v1",
                "places": [],
            },
            "task": {"state": "idle", "active": False, "task": None},
            "telemetry": {"planned_path": [], "actual_path": []},
            "errors": [],
        },
    )
    monkeypatch.setattr(
        control,
        "confirm_current_place",
        lambda name, aliases: calls.append(("confirm", (name, aliases)))
        or {"accepted": True},
    )
    monkeypatch.setattr(
        control,
        "navigate",
        lambda instruction_id, destination: calls.append(
            ("navigate", (instruction_id, destination))
        )
        or {"accepted": True},
    )
    monkeypatch.setattr(
        control,
        "cancel",
        lambda task_id: calls.append(("cancel", task_id))
        or {"state": "cancelled", "active": False},
    )
    monkeypatch.setattr(
        control,
        "stop_all",
        lambda: calls.append(("stop_all", None))
        or {"status": "stopped", "failed_components": []},
    )
    monkeypatch.setattr(
        control,
        "record_reply",
        lambda payload: calls.append(("reply", payload))
        or {"accepted": True, "duplicate": False},
    )

    assert client.get("/api/stage2/status").json()["connected"] is True
    assert (
        client.post(
            "/api/stage2/places/confirm-current",
            json={"name": "测试起点", "aliases": ["起点"]},
        ).status_code
        == 200
    )
    assert client.post("/api/stage2/stop-all").status_code == 200
    assert (
        client.post(
            "/api/stage2/navigate",
            json={
                "instruction_id": "studio-instruction-0001",
                "destination": "门口测试点",
            },
        ).status_code
        == 200
    )
    assert (
        client.post(
            "/api/stage2/cancel",
            json={"task_id": "task-stage2-0001"},
        ).status_code
        == 200
    )
    assert (
        client.post(
            "/api/stage2/reply",
            json={
                "event": "agent.reply.completed",
                "reply_id": "reply-stage2-0001",
                "instruction_id": "studio-instruction-0001",
                "text": "任务已完成",
                "completed_at": "2026-07-25T06:00:02Z",
            },
        ).status_code
        == 200
    )
    assert calls[0] == ("confirm", ("测试起点", ["起点"]))
    assert calls[1] == ("stop_all", None)
    assert calls[2] == (
        "navigate",
        ("studio-instruction-0001", "门口测试点"),
    )
    assert calls[3] == ("cancel", "task-stage2-0001")
    assert calls[4][0] == "reply"
