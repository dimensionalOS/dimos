"""Auth endpoints for operator login and robot key management."""

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, EmailStr
from sqlalchemy.ext.asyncio import AsyncSession

from models.database import get_db
from services.auth import (
    create_token,
    create_user,
    get_current_user,
    get_user_by_email,
    register_robot_key,
    verify_password,
)

router = APIRouter(prefix="/auth", tags=["auth"])


class RegisterRequest(BaseModel):
    email: EmailStr
    password: str


class LoginRequest(BaseModel):
    email: EmailStr
    password: str


class TokenResponse(BaseModel):
    token: str
    user_id: str
    role: str


class RegisterRobotRequest(BaseModel):
    robot_id: str
    api_key: str


@router.post("/register", response_model=TokenResponse)
async def register(body: RegisterRequest, db: AsyncSession = Depends(get_db)):
    """Register a new operator account."""
    existing = await get_user_by_email(db, body.email)
    if existing:
        raise HTTPException(status_code=409, detail="User already exists")

    user = await create_user(db, body.email, body.password)
    token = create_token(user.email, user.role)
    return TokenResponse(token=token, user_id=user.email, role=user.role)


@router.post("/login", response_model=TokenResponse)
async def login(body: LoginRequest, db: AsyncSession = Depends(get_db)):
    """Operator login. Returns JWT."""
    user = await get_user_by_email(db, body.email)
    if not user or not verify_password(body.password, user.password_hash):
        raise HTTPException(status_code=401, detail="Invalid credentials")

    token = create_token(user.email, user.role)
    return TokenResponse(token=token, user_id=user.email, role=user.role)


@router.post("/robots")
async def register_robot(body: RegisterRobotRequest, user: dict = Depends(get_current_user)):
    """Admin: register a legacy robot API key."""
    if user.get("role") != "admin":
        raise HTTPException(status_code=403, detail="Admin only")

    register_robot_key(body.api_key, body.robot_id)
    return {"registered": body.robot_id}


@router.get("/me")
async def me(user: dict = Depends(get_current_user)):
    """Get current user info from token."""
    return user
