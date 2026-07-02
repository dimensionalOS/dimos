#!/usr/bin/env python3
# `python -m gateway` — boot the dimoscope backend: uvicorn hosting the FastAPI app from app.py.
#
# The listen socket is created here so SO_SNDBUF can be bounded before accept (accepted sockets
# inherit it). A bounded send buffer is what makes egress priority actually bite on /ws over WAN:
# kernel autotuning otherwise grows the buffer to megabytes and drains the priority outbox into a
# FIFO. 256 KB caps nothing at loopback RTTs but surfaces WAN backlog in the outbox within ~256 KB
# in-flight (~40 Mbps at 50 ms RTT). WS_SNDBUF=0 restores OS autotuning; EGRESS_KBPS stays the
# explicit hard-cap tool.
from __future__ import annotations

import os
import socket

import uvicorn

from dimos.utils.logging_config import setup_logger

from .app import HOST, PORT, build_app

logger = setup_logger()

WS_SNDBUF = int(os.environ.get("WS_SNDBUF", "262144"))


def main() -> None:
    family = socket.AF_INET6 if ":" in HOST else socket.AF_INET
    sock = socket.socket(family, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if WS_SNDBUF > 0:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, WS_SNDBUF)
    sock.bind((HOST, PORT))
    effective = sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
    logger.info("egress send buffer", ws_sndbuf=WS_SNDBUF or "os-autotuned", effective=effective)
    config = uvicorn.Config(build_app(), log_level="info")
    uvicorn.Server(config).run(sockets=[sock])


if __name__ == "__main__":
    main()
