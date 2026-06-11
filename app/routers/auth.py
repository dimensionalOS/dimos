"""Auth endpoints.

Login/signup happen directly between the SPA and Cognito — the broker only
verifies tokens. The SPA reads pool/client IDs from /auth/config at boot.
"""

from fastapi import APIRouter, Depends

from config import settings
from services.auth import get_current_user

router = APIRouter(prefix="/auth", tags=["auth"])


@router.get("/config")
async def auth_config():
    """Public Cognito client config for the SPA (not secret)."""
    return {
        "region": settings.cognito_region,
        "user_pool_id": settings.cognito_user_pool_id,
        "client_id": settings.cognito_client_id,
    }


@router.get("/me")
async def me(user: dict = Depends(get_current_user)):
    """Get current user info from token."""
    return user
