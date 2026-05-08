"""API key management endpoints for developers."""

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from sqlalchemy.ext.asyncio import AsyncSession

from models.database import get_db
from services.auth import get_current_user
from services.keys import create_api_key, list_api_keys, revoke_api_key

router = APIRouter(prefix="/keys", tags=["keys"])


class CreateKeyRequest(BaseModel):
    name: str  # Human-readable label, e.g. "Lab Robot 01"
    robot_id: str | None = None  # Optional robot association


class CreateKeyResponse(BaseModel):
    id: str
    name: str
    key_prefix: str
    robot_id: str | None
    api_key: str  # Full plaintext key — shown ONCE
    created_at: str


class KeyInfo(BaseModel):
    id: str
    name: str
    key_prefix: str
    robot_id: str | None
    last_used_at: str | None
    created_at: str


class KeyListResponse(BaseModel):
    keys: list[KeyInfo]


@router.post("", response_model=CreateKeyResponse)
async def create_key(
    body: CreateKeyRequest,
    user: dict = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """Generate a new API key for robot authentication.

    The full key is returned ONCE in this response. Store it securely —
    it cannot be retrieved again.

    Usage in DimOS:
        RemoteTeleopModule(api_key="dtk_live_...", robot_name="My Robot")
    """
    key_record, plaintext = await create_api_key(
        db=db,
        owner_id=user["sub"],
        name=body.name,
        robot_id=body.robot_id,
    )
    return CreateKeyResponse(
        id=key_record.id,
        name=key_record.name,
        key_prefix=key_record.key_prefix,
        robot_id=key_record.robot_id,
        api_key=plaintext,
        created_at=key_record.created_at.isoformat(),
    )


@router.get("", response_model=KeyListResponse)
async def list_keys(
    user: dict = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """List all active API keys for the authenticated user."""
    keys = await list_api_keys(db=db, owner_id=user["sub"])
    return KeyListResponse(
        keys=[
            KeyInfo(
                id=k.id,
                name=k.name,
                key_prefix=k.key_prefix,
                robot_id=k.robot_id,
                last_used_at=k.last_used_at.isoformat() if k.last_used_at else None,
                created_at=k.created_at.isoformat(),
            )
            for k in keys
        ]
    )


@router.delete("/{key_id}")
async def delete_key(
    key_id: str,
    user: dict = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """Revoke an API key. The key will immediately stop working."""
    success = await revoke_api_key(db=db, key_id=key_id, owner_id=user["sub"])
    if not success:
        raise HTTPException(status_code=404, detail="Key not found or already revoked")
    return {"revoked": True, "id": key_id}
