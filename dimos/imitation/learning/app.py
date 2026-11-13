"""Standalone Learning HTTP API. Importing this module starts no robot graph."""

from __future__ import annotations

from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import Depends, FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import FileResponse, JSONResponse
from starlette.exceptions import HTTPException

from dimos.imitation.learning.protocol import LearningError, allowed, identity, source
from dimos.imitation.learning.service import PRESETS, Service
from dimos.imitation.learning.store import Store, default_store


def create_app(store: Store | None = None) -> FastAPI:
    service = Service(store or default_store())

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        # Migrations are explicit, never implicitly applied by web startup.
        with service.store.transaction() as db:
            db.execute("SELECT version FROM migrations")
        yield

    app = FastAPI(title="DimOS Learning", version="1", lifespan=lifespan)
    app.state.service = service

    @app.exception_handler(LearningError)
    async def learning_error(request: Request, error: LearningError):
        return JSONResponse(
            status_code=error.status,
            content={
                "error": {"code": error.code, "message": error.message, "details": error.details}
            },
        )

    @app.exception_handler(RequestValidationError)
    async def validation_error(request: Request, error: RequestValidationError):
        return JSONResponse(
            status_code=422,
            content={"error": {"code": "invalid_input", "message": "Invalid request payload"}},
        )

    @app.exception_handler(HTTPException)
    async def http_error(request: Request, error: HTTPException):
        return JSONResponse(
            status_code=error.status_code,
            content={"error": {"code": "http_error", "message": str(error.detail)}},
        )

    @app.get("/health")
    def health():
        return {"status": "ok", "service": "dimos-learning", "version": 1}

    @app.get("/v1/recordings")
    def recordings(who: dict = Depends(identity)):
        return {
            "items": [
                {
                    k: v
                    for k, v in item.items()
                    if k not in ("download_url", "schema_url", "manifest_url")
                }
                for item in who.get("sources", [])
                if item.get("can_read")
            ]
        }

    @app.get("/v1/recordings/{key}")
    def recording(key: str, who: dict = Depends(identity)):
        return {
            k: v for k, v in service.recording(who, key).items() if k not in ("schema", "origin_s")
        }

    @app.get("/v1/recordings/{key}/annotations")
    def annotations(key: str, who: dict = Depends(identity)):
        return service.annotations(who, key)

    @app.put("/v1/recordings/{key}/annotations")
    def save_annotations(key: str, body: dict, who: dict = Depends(identity)):
        return service.save_annotations(who, key, body)

    @app.get("/v1/recordings/{key}/media")
    def media_status(key: str, who: dict = Depends(identity)):
        return service.media_status(who, key)

    @app.post("/v1/recordings/{key}/media")
    def prepare_media(key: str, who: dict = Depends(identity)):
        return service.queue_media(who, key)

    @app.get("/v1/recordings/{key}/media/{camera}")
    def media_file(key: str, camera: str, who: dict = Depends(identity)):
        source(who, key)
        media = service.media_status(who, key)
        entry = next((c for c in media.get("cameras", []) if c["id"] == camera), None)
        if media.get("status") != "ready" or not entry:
            raise LearningError(404, "media_not_ready", "Prepared camera media not available")
        # Worker result paths cannot escape the configured state directory.
        path = (service.state / entry["path"]).resolve()
        if not path.is_relative_to(service.state.resolve()) or not path.is_file():
            raise LearningError(404, "media_missing", "Prepared media missing")
        return FileResponse(path, media_type="video/mp4")

    @app.get("/v1/profiles")
    def profiles(who: dict = Depends(identity)):
        items = {}
        for item in who.get("sources", []):
            if not item.get("can_read"):
                continue
            info = service.recording(who, item["id"])
            schema = info["schema"]
            semantics = info["capture_truth"]["gripper_semantics"] or {}
            items[info["profile_id"]] = {
                "id": info["profile_id"],
                "name": info["profile_id"],
                "cameras": info["cameras"],
                "action": schema.get("action"),
                "observation": schema.get("observation"),
                "compatible": semantics.get("gripperCommandRecorded") is not False,
                "block_reason": "gripper_command_missing"
                if semantics.get("gripperCommandRecorded") is False
                else None,
            }
        return {"items": list(items.values())}

    @app.get("/v1/presets")
    def presets(who: dict = Depends(identity)):
        return {"items": PRESETS}

    @app.post("/v1/datasets")
    def create_dataset(body: dict, who: dict = Depends(identity)):
        return service.dataset(who, body)

    @app.post("/v1/runs")
    def create_run(body: dict, who: dict = Depends(identity)):
        return service.run(who, body)

    @app.get("/v1/jobs/{key}/logs")
    def logs(key: str, after: int = 0, who: dict = Depends(identity)):
        job = service.detail(who, "job", key)
        return {"items": [x for x in job["logs"] if x["sequence"] > after]}

    @app.post("/v1/jobs/{key}/cancel")
    def cancel(key: str, who: dict = Depends(identity)):
        return service.cancel(who, key)

    @app.post("/v1/runs/{key}/archive")
    def archive(key: str, who: dict = Depends(identity)):
        with service.store.transaction() as db:
            item = allowed(service.store.get(db, "run", key), who, True)
            item["archived"] = True
            service.store.put(db, "run", key, item)
            return item

    @app.post("/v1/policies/{key}/load-check")
    def load_check(key: str, who: dict = Depends(identity)):
        policy = service.detail(who, "policy", key)
        allowed(policy, who, True)
        with service.store.transaction() as db:
            return service.public_job(service.new_job(db, who, "load-check", {"policy": policy}))

    @app.get("/v1/policies/{key}/download")
    def download(key: str, who: dict = Depends(identity)):
        policy = service.detail(who, "policy", key)
        path = Path(policy.get("package_path", "")).resolve()
        if not path.is_relative_to(service.state.resolve()) or not path.is_file():
            raise LearningError(
                404, "package_missing", "Verified package unavailable on this API host"
            )
        from dimos.imitation.learning.media import sha256

        if sha256(path) != policy.get("package_sha256"):
            raise LearningError(409, "checksum_mismatch", "Policy package failed verification")
        return FileResponse(path, filename=f"policy-{key}.tar.gz", media_type="application/gzip")

    @app.post("/v1/workers/pairing")
    def pairing(body: dict, who: dict = Depends(identity)):
        return service.pairing(who, body)

    @app.post("/v1/workers/pair")
    def pair(body: dict):
        return service.pair(body)

    @app.post("/v1/workers/{key}/revoke")
    def revoke(key: str, who: dict = Depends(identity)):
        with service.store.transaction() as db:
            item = allowed(service.store.get(db, "worker", key), who, True)
            item["revoked"] = True
            service.store.put(db, "worker", key, item)
            return {"id": key, "revoked": True}

    def worker_token(request: Request) -> str:
        header = request.headers.get("authorization", "")
        if not header.startswith("Bearer "):
            raise LearningError(401, "worker_unauthorized", "Worker bearer token required")
        return header[7:]

    @app.post("/v1/worker/heartbeat")
    def heartbeat(body: dict, token: str = Depends(worker_token)):
        return service.heartbeat(token, body)

    @app.post("/v1/worker/claim")
    def claim(body: dict, token: str = Depends(worker_token)):
        return service.claim(token, body)

    @app.post("/v1/worker/jobs/{key}/report")
    def report(key: str, body: dict, token: str = Depends(worker_token)):
        return service.report(token, key, body)

    def register_resources(plural: str, kind: str) -> None:
        def listing(who: dict = Depends(identity)):
            return {"items": service.list(who, kind)}

        def detail(key: str, who: dict = Depends(identity)):
            return service.detail(who, kind, key)

        app.add_api_route(f"/v1/{plural}", listing, methods=["GET"])
        if kind != "worker":
            app.add_api_route(f"/v1/{plural}/{{key}}", detail, methods=["GET"])

    for plural, kind in (
        ("datasets", "dataset"),
        ("runs", "run"),
        ("policies", "policy"),
        ("jobs", "job"),
        ("workers", "worker"),
    ):
        register_resources(plural, kind)
    return app


app = create_app()
