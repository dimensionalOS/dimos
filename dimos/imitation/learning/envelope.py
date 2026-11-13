"""Internal source envelope: signed digest, bounded body, no large JWT headers.

Browser credentials never reach this boundary. Only the platform has the signing
key; sources are authenticated before the original request is passed to FastAPI.
"""

from __future__ import annotations

import base64
import hashlib
import json
from typing import Any

from fastapi import Request
from fastapi.responses import JSONResponse

from dimos.imitation.learning.protocol import LearningError, identity

MAX_ENVELOPE = 8 * 1024 * 1024


def sources_digest(sources: list[dict[str, Any]]) -> str:
    return hashlib.sha256(
        json.dumps(sources, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


class DelegationEnvelope:
    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if (
            scope["type"] != "http"
            or dict(scope.get("headers", [])).get(b"x-learning-envelope") != b"1"
        ):
            return await self.app(scope, receive, send)
        try:
            body = bytearray()
            while True:
                event = await receive()
                if event["type"] == "http.disconnect":
                    return
                body.extend(event.get("body", b""))
                if len(body) > MAX_ENVELOPE:
                    raise LearningError(
                        413, "envelope_too_large", "Internal source envelope exceeds limit"
                    )
                if not event.get("more_body", False):
                    break
            value = json.loads(body)
            sources = value["sources"]
            if (
                not isinstance(sources, list)
                or len(sources) > 100
                or any(not isinstance(s, dict) for s in sources)
            ):
                raise ValueError("Invalid source list")
            raw = base64.b64decode(value["body_base64"], validate=True)
            if len(raw) > 2 * 1024 * 1024:
                raise ValueError("Original body exceeds limit")
            scope.setdefault("state", {})["learning_sources"] = sources
            scope["state"]["learning_source_digest"] = sources_digest(sources)
            if "source_digest" not in identity(Request(scope)):
                raise LearningError(401, "unsigned_envelope", "Signed source digest required")
            scope["headers"] = [
                (k, v)
                for k, v in scope["headers"]
                if k.lower() not in (b"content-length", b"content-type", b"x-learning-envelope")
            ]
            scope["headers"] += [
                (b"content-length", str(len(raw)).encode()),
                (b"content-type", b"application/json"),
            ]
            consumed = False

            async def original():
                nonlocal consumed
                if not consumed:
                    consumed = True
                    return {"type": "http.request", "body": raw, "more_body": False}
                return await receive()

            return await self.app(scope, original, send)
        except LearningError as error:
            response = JSONResponse(
                {"error": {"code": error.code, "message": error.message}}, status_code=error.status
            )
        except (ValueError, TypeError, KeyError, AttributeError):
            response = JSONResponse(
                {
                    "error": {
                        "code": "invalid_envelope",
                        "message": "Malformed internal source envelope",
                    }
                },
                status_code=422,
            )
        await response(scope, receive, send)
