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

"""HTTP transfer API for a hosted replay repository."""

from __future__ import annotations

import argparse
from collections.abc import Iterator
from pathlib import Path
import re
from typing import Annotated, Any
from urllib.parse import quote

from fastapi import FastAPI, Header, HTTPException, Request
from fastapi.responses import JSONResponse, StreamingResponse
from starlette.responses import Response
import uvicorn

from dimos.hosted_data.repository import (
    ClosableBinaryReader,
    ReplayObject,
    ReplayRepository,
    ReplayRepositoryBackend,
    RepositoryError,
)

_STREAM_CHUNK_SIZE = 1024 * 1024
_RANGE_RE = re.compile(r"^bytes=(\d*)-(\d*)$")


def _object_headers(item: ReplayObject) -> dict[str, str]:
    return {
        "Accept-Ranges": "bytes",
        "Content-Disposition": f"attachment; filename*=UTF-8''{quote(item.filename, safe='')}",
        "ETag": f'"{item.sha256}"',
        "X-Object-ID": item.object_id,
    }


def _stream(body: ClosableBinaryReader, *, remaining: int) -> Iterator[bytes]:
    try:
        while remaining:
            chunk = body.read(min(_STREAM_CHUNK_SIZE, remaining))
            if not chunk:
                raise RepositoryError("stored object ended before its declared size")
            remaining -= len(chunk)
            yield chunk
    finally:
        body.close()


def _parse_range(value: str, size: int) -> tuple[int, int]:
    match = _RANGE_RE.fullmatch(value.strip())
    if match is None or not any(match.groups()) or size == 0:
        raise ValueError("invalid byte range")

    start_text, end_text = match.groups()
    if start_text:
        start = int(start_text)
        if start >= size:
            raise ValueError("byte range starts beyond the object")
        end = min(int(end_text), size - 1) if end_text else size - 1
        if end < start:
            raise ValueError("byte range ends before it starts")
        return start, end

    suffix_length = int(end_text)
    if suffix_length <= 0:
        raise ValueError("byte suffix length must be positive")
    return max(size - suffix_length, 0), size - 1


def create_app(repository: ReplayRepositoryBackend) -> FastAPI:
    """Create an upload, list, and download API over a repository backend."""
    app = FastAPI(title="DimOS hosted replay repository", version="1")

    @app.exception_handler(RepositoryError)
    async def repository_error_handler(
        _request: Request,
        exc: RepositoryError,
    ) -> JSONResponse:
        return JSONResponse(status_code=400, content={"detail": str(exc)})

    @app.get("/healthz")
    def health() -> dict[str, str]:
        return {"status": "ok"}

    @app.put(
        "/v1/repositories/{owner}/{repository_name}/objects",
        status_code=201,
    )
    async def upload_object(
        owner: str,
        repository_name: str,
        request: Request,
        filename: str,
        content_length: Annotated[int | None, Header(alias="Content-Length")] = None,
        expected_sha256: Annotated[str | None, Header(alias="X-Content-SHA256")] = None,
    ) -> dict[str, Any]:
        if content_length is None:
            raise HTTPException(status_code=411, detail="Content-Length is required")
        if content_length < 0:
            raise HTTPException(status_code=400, detail="Content-Length cannot be negative")

        received = 0
        with tempfile.SpooledTemporaryFile(max_size=8 * _STREAM_CHUNK_SIZE, mode="w+b") as source:
            async for chunk in request.stream():
                received += len(chunk)
                if received > content_length:
                    raise HTTPException(
                        status_code=400, detail="request body exceeds Content-Length"
                    )
                source.write(chunk)
            if received != content_length:
                raise HTTPException(
                    status_code=400,
                    detail=f"received {received} bytes, expected {content_length}",
                )
            source.seek(0)
            item = repository.put_stream(
                owner=owner,
                repository=repository_name,
                filename=filename,
                source=source,
                size_bytes=content_length,
                content_type=request.headers.get("content-type", "application/octet-stream"),
                expected_sha256=expected_sha256,
            )
        return item.to_dict()

    @app.get("/v1/repositories/{owner}/{repository_name}/objects")
    def list_objects(owner: str, repository_name: str) -> list[dict[str, Any]]:
        return [item.to_dict() for item in repository.list(owner, repository_name)]

    @app.head("/v1/repositories/{owner}/{repository_name}/objects/{object_id}")
    def head_object(owner: str, repository_name: str, object_id: str) -> Response:
        try:
            item, body = repository.open(owner, repository_name, object_id)
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail="object not found") from exc
        body.close()
        headers = _object_headers(item)
        headers["Content-Length"] = str(item.size_bytes)
        return Response(headers=headers, media_type=item.content_type)

    @app.get("/v1/repositories/{owner}/{repository_name}/objects/{object_id}")
    def download_object(
        owner: str,
        repository_name: str,
        object_id: str,
        range_header: Annotated[str | None, Header(alias="Range")] = None,
    ) -> StreamingResponse:
        try:
            item, body = repository.open(owner, repository_name, object_id)
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail="object not found") from exc

        headers = _object_headers(item)
        start = 0
        end = item.size_bytes - 1
        status_code = 200
        if range_header is not None:
            try:
                start, end = _parse_range(range_header, item.size_bytes)
            except ValueError as exc:
                body.close()
                raise HTTPException(
                    status_code=416,
                    detail=str(exc),
                    headers={"Content-Range": f"bytes */{item.size_bytes}"},
                ) from exc
            body.seek(start)
            status_code = 206
            headers["Content-Range"] = f"bytes {start}-{end}/{item.size_bytes}"

        content_length = max(end - start + 1, 0)
        headers["Content-Length"] = str(content_length)
        return StreamingResponse(
            _stream(body, remaining=content_length),
            status_code=status_code,
            media_type=item.content_type,
            headers=headers,
        )

    return app


def main() -> None:
    """Run a standalone repository server."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    uvicorn.run(create_app(ReplayRepository(args.root)), host=args.host, port=args.port)


if __name__ == "__main__":
    main()
