"""Request-bound platform delegation and independent opaque worker credentials."""

from __future__ import annotations

import hashlib
import os
import time
from typing import Any

from fastapi import Request
import jwt


class LearningError(Exception):
    def __init__(self, status: int, code: str, message: str, **details: Any):
        self.status, self.code, self.message, self.details = status, code, message, details


def digest(token: str) -> str:
    return hashlib.sha256(token.encode()).hexdigest()


def identity(request: Request) -> dict[str, Any]:
    secret = os.environ.get("DIMOS_LEARNING_DELEGATION_SECRET", "")
    if len(secret.encode()) < 32:
        raise LearningError(
            503,
            "delegation_unconfigured",
            "A dedicated delegation secret of at least 32 bytes is required",
        )
    header = request.headers.get("authorization", "")
    if not header.startswith("Bearer "):
        raise LearningError(401, "unauthorized", "Platform delegation required")
    try:
        claims = jwt.decode(
            header[7:],
            secret,
            algorithms=["HS256"],
            issuer="dimos-cloud",
            audience="dimos-learning",
            options={
                "require": [
                    "exp",
                    "iat",
                    "sub",
                    "org_id",
                    "role",
                    "platform_admin",
                    "method",
                    "path",
                ]
            },
        )
        if (
            claims["exp"] - claims["iat"] > 60
            or claims["iat"] > time.time()
            or claims["exp"] <= claims["iat"]
        ):
            raise ValueError("invalid delegation lifetime")
        if claims["method"] != request.method or claims["path"] != request.url.path:
            raise ValueError("delegation is bound to a different request")
        if (
            claims["role"] not in ("member", "admin")
            or type(claims["platform_admin"]) is not bool
            or not claims["sub"]
            or not claims["org_id"]
        ):
            raise ValueError("invalid identity claims")
        return claims
    except (jwt.PyJWTError, ValueError, TypeError) as error:
        raise LearningError(
            401, "unauthorized", "Invalid or expired platform delegation"
        ) from error


def allowed(
    resource: dict[str, Any] | None, who: dict[str, Any], write: bool = False
) -> dict[str, Any]:
    if resource is None:
        raise LearningError(404, "not_found", "Resource not found")
    if resource.get("org_id") != who["org_id"]:
        raise LearningError(403, "forbidden", "Organization mismatch")
    if (
        write
        and resource.get("owner_id") != who["sub"]
        and who["role"] != "admin"
        and not who["platform_admin"]
    ):
        raise LearningError(403, "forbidden", "Owner or administrator required")
    return resource


def source(who: dict[str, Any], key: str, write: bool = False) -> dict[str, Any]:
    item = next((x for x in who.get("sources", []) if x.get("id") == key), None)
    if item is None or not item.get("can_read") or (write and not item.get("can_write")):
        raise LearningError(403, "source_denied", "Request-scoped source authorization required")
    if not item.get("complete") or item.get("quarantined") or not item.get("verified"):
        raise LearningError(
            422, "source_unavailable", "Source must be complete, verified and not quarantined"
        )
    return item
