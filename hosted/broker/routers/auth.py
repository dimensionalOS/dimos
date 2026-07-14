# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Auth endpoints.

Login/signup happen directly between the SPA and Cognito — the broker only
verifies tokens. The SPA reads pool/client IDs from /auth/config at boot.
"""

from config import settings
from fastapi import APIRouter, Depends
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
