"""dimos-teleop: Session microservice for hosted teleoperation."""

import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

# Configure the root logger before any module-level loggers get created so
# log.info / log.error calls in services/* and routers/* actually reach
# uvicorn's stderr (and therefore journald). Without this the root logger
# stays at WARNING and every INFO log below is silently dropped — which is
# why CF SDP exchange / track-add / datachannel-bridge detail wasn't
# showing up in journal even though the log calls exist.
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s %(message)s",
)

from config import settings
from models.database import init_db
from routers import auth, keys, sessions
from services.auth import register_robot_key


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: init DB (creates tables if missing)
    await init_db()

    if settings.environment == "dev":
        dev_key = "dev-robot-key-change-me"
        register_robot_key(dev_key, "dev-robot")
        print(f"[dev] robot key registered: {dev_key} → dev-robot")

    yield
    # Shutdown


app = FastAPI(
    title="dimos-teleop",
    description="Session microservice for hosted teleoperation",
    version="0.1.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=[settings.public_origin],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(auth.router, prefix="/api/v1")
app.include_router(keys.router, prefix="/api/v1")
app.include_router(sessions.router, prefix="/api/v1")


@app.get("/health")
async def health():
    return {"status": "ok", "service": "dimos-teleop", "environment": settings.environment}
