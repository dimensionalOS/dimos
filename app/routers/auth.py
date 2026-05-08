"""Auth endpoints for operator login and robot key management."""

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel

from services.auth import (
    create_token,
    get_current_user,
    hash_password,
    register_robot_key,
    verify_password,
)

router = APIRouter(prefix="/auth", tags=["auth"])

# In-memory user store (move to DB for production)
USERS: dict[str, dict] = {}


class RegisterRequest(BaseModel):
    email: str
    password: str


class LoginRequest(BaseModel):
    email: str
    password: str


class TokenResponse(BaseModel):
    token: str
    user_id: str
    role: str


class RegisterRobotRequest(BaseModel):
    robot_id: str
    api_key: str


@router.post("/register", response_model=TokenResponse)
async def register(body: RegisterRequest):
    """Register a new operator account."""
    if body.email in USERS:
        raise HTTPException(status_code=409, detail="User already exists")

    USERS[body.email] = {
        "email": body.email,
        "password_hash": hash_password(body.password),
        "role": "operator",
    }

    token = create_token(body.email, "operator")
    return TokenResponse(token=token, user_id=body.email, role="operator")


@router.post("/login", response_model=TokenResponse)
async def login(body: LoginRequest):
    """Operator login. Returns JWT."""
    user = USERS.get(body.email)
    if not user or not verify_password(body.password, user["password_hash"]):
        raise HTTPException(status_code=401, detail="Invalid credentials")

    token = create_token(body.email, user["role"])
    return TokenResponse(token=token, user_id=body.email, role=user["role"])


@router.post("/robots")
async def register_robot(body: RegisterRobotRequest, user: dict = Depends(get_current_user)):
    """Admin: register a robot API key."""
    if user.get("role") != "admin":
        raise HTTPException(status_code=403, detail="Admin only")

    register_robot_key(body.api_key, body.robot_id)
    return {"registered": body.robot_id}


@router.get("/me")
async def me(user: dict = Depends(get_current_user)):
    """Get current user info from token."""
    return user
