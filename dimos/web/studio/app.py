"""FastAPI application for the local DimOS Studio workbench."""

from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .models import (
    AgentMessageRequest,
    RobotCheckRequest,
    RuntimeStartRequest,
    SkillSourceRequest,
    StageTwoCancelRequest,
    StageTwoConfirmPlaceRequest,
    StageTwoNavigateRequest,
    StageTwoReplyRequest,
    StudioSettings,
    TeleopKeyRequest,
)
from .service import DEFAULT_SETTINGS_PATH, DEFAULT_SKILL_PATH, StudioService


def create_app(
    settings_path: Path = DEFAULT_SETTINGS_PATH,
    skill_path: Path = DEFAULT_SKILL_PATH,
) -> FastAPI:
    """Create an isolated Studio app, allowing temporary paths in tests."""

    app = FastAPI(title="DimOS Studio", version="0.1.0")
    service = StudioService(settings_path=settings_path, skill_path=skill_path)
    app.state.studio_service = service
    static_dir = Path(__file__).parent / "static"
    app.mount("/static", StaticFiles(directory=static_dir), name="static")

    @app.get("/", include_in_schema=False)
    def index() -> FileResponse:
        return FileResponse(static_dir / "index.html")

    @app.get("/api/health")
    def health() -> dict[str, Any]:
        return {
            "ok": True,
            "service": "DimOS Studio",
            "safety": "movement_locked_by_default",
        }

    @app.get("/api/settings")
    def get_settings() -> StudioSettings:
        return service.load_settings()

    @app.put("/api/settings")
    def put_settings(settings: StudioSettings) -> StudioSettings:
        return service.save_settings(settings)

    @app.post("/api/robot/check")
    def robot_check(payload: RobotCheckRequest) -> dict[str, Any]:
        return service.robot_check(payload.robot_ip, payload.signal_port)

    @app.post("/api/robot/discover")
    def robot_discover() -> dict[str, Any]:
        try:
            return service.discover_robots()
        except (ValueError, TimeoutError) as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @app.get("/api/teleop/key")
    def teleop_key_status() -> dict[str, Any]:
        return service.teleop_key_status()

    @app.put("/api/teleop/key")
    def put_teleop_key(payload: TeleopKeyRequest) -> dict[str, Any]:
        try:
            return service.save_teleop_key(payload.api_key)
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc

    @app.get("/api/runtime/status")
    def runtime_status() -> dict[str, Any]:
        return service.runtime_status()

    @app.get("/api/runtime/command")
    def runtime_command(mode: str = "simulation") -> dict[str, Any]:
        if mode not in {
            "simulation",
            "hardware_readonly",
            "hardware",
            "hosted_teleop",
        }:
            raise HTTPException(
                status_code=400,
                detail=(
                    "mode 必须是 simulation、hardware_readonly、hardware "
                    "或 hosted_teleop"
                ),
            )
        return {"mode": mode, "command": service.command_preview(mode)}

    @app.post("/api/runtime/start")
    def runtime_start(payload: RuntimeStartRequest) -> dict[str, Any]:
        try:
            return service.start_runtime(payload.mode, payload.confirmation)
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/runtime/stop")
    def runtime_stop() -> dict[str, Any]:
        try:
            return service.stop_runtime()
        except (ValueError, TimeoutError) as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.get("/api/mcp/tools")
    def mcp_tools() -> dict[str, Any]:
        try:
            return {"tools": service.list_mcp_tools()}
        except ValueError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @app.post("/api/agent/send")
    def agent_send(payload: AgentMessageRequest) -> dict[str, Any]:
        try:
            return {"result": service.send_agent_message(payload.message)}
        except ValueError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @app.get("/api/stage2/status")
    def stage2_status() -> dict[str, Any]:
        return service.stage2_status()

    @app.post("/api/stage2/navigate")
    def stage2_navigate(payload: StageTwoNavigateRequest) -> dict[str, Any]:
        try:
            return service.stage2_navigate(
                payload.instruction_id,
                payload.destination,
            )
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/stage2/places/confirm-current")
    def stage2_confirm_current_place(
        payload: StageTwoConfirmPlaceRequest,
    ) -> dict[str, Any]:
        try:
            return service.stage2_confirm_current_place(
                payload.name,
                payload.aliases,
            )
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/stage2/cancel")
    def stage2_cancel(payload: StageTwoCancelRequest) -> dict[str, Any]:
        try:
            return service.stage2_cancel(payload.task_id)
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/stage2/stop-all")
    def stage2_stop_all() -> dict[str, Any]:
        try:
            return service.stage2_stop_all()
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/stage2/reply")
    def stage2_reply(payload: StageTwoReplyRequest) -> dict[str, Any]:
        try:
            return service.stage2_reply(payload.model_dump(mode="json"))
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.get("/api/skill")
    def get_skill() -> dict[str, Any]:
        try:
            return service.read_skill_source()
        except OSError as exc:
            raise HTTPException(status_code=404, detail=f"Skill 文件不存在: {exc}") from exc

    @app.put("/api/skill")
    def put_skill(payload: SkillSourceRequest) -> dict[str, Any]:
        try:
            return service.save_skill_source(payload.source)
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc

    return app


app = create_app()
