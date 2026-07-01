#!/usr/bin/env python3
# `python -m gateway` — boot the dimoscope backend: uvicorn hosting the FastAPI app from app.py.
from __future__ import annotations

import uvicorn

from .app import HOST, PORT, build_app


def main() -> None:
    uvicorn.run(build_app(), host=HOST, port=PORT, log_level="info")


if __name__ == "__main__":
    main()
