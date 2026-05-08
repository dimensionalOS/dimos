"""dimos-teleop: Session microservice for hosted teleoperation."""

from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

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
