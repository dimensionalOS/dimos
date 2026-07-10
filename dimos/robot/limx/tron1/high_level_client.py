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

from __future__ import annotations

import json
import threading
from typing import Any, Callable
import uuid

import websocket

from dimos.robot.limx.tron1 import protocol
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TRON1ProtocolError(RuntimeError):
    pass


class TRON1HighLevelClient:
    def __init__(
        self,
        ws_url: str,
        accid: str | None = None,
        timeout: float = 10.0,
    ) -> None:
        self.ws_url = self._normalize_ws_url(ws_url)
        self.accid = accid
        self.timeout = timeout

        self._ws: websocket.WebSocket | None = None
        self._recv_thread: threading.Thread | None = None
        self._closed = False

        self._pending: dict[str, threading.Event] = {}
        self._responses: dict[str, dict[str, Any]] = {}
        self._notify_handlers: dict[str, list[Callable[[dict[str, Any]], None]]] = {}
        self._lock = threading.Lock()

    def start(self) -> None:
        self._closed = False
        self._ws = websocket.WebSocket()
        self._ws.connect(self.ws_url)  # type: ignore[no-untyped-call]
        self._ws.settimeout(1.0)
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

    def stop(self) -> None:
        self._closed = True
        if self._ws:
            try:
                self._ws.close()
            except Exception:
                pass
        self._ws = None

    def register_notify_handler(self, name: str, handler: Callable[[dict[str, Any]], None]) -> None:
        with self._lock:
            self._notify_handlers.setdefault(name, []).append(handler)

    def send(self, name: str, arguments: dict[str, Any] | None = None) -> str:
        if self._ws is None:
            raise RuntimeError("TRON1HighLevelClient is not started")

        request_id = uuid.uuid4().hex
        payload = protocol.encode_call(request_id=request_id, name=name, arguments=arguments or {})
        if self.accid:
            payload["accid"] = self.accid
        self._ws.send(json.dumps(payload))
        return request_id

    def call(self, name: str, arguments: dict[str, Any] | None = None) -> dict[str, Any]:
        if self._ws is None:
            raise RuntimeError("TRON1HighLevelClient is not started")

        request_id = uuid.uuid4().hex
        event = threading.Event()
        with self._lock:
            self._pending[request_id] = event
        payload = protocol.encode_call(request_id=request_id, name=name, arguments=arguments or {})
        if self.accid:
            payload["accid"] = self.accid
        self._ws.send(json.dumps(payload))

        if not event.wait(timeout=self.timeout):
            with self._lock:
                self._pending.pop(request_id, None)
                self._responses.pop(request_id, None)
            raise TimeoutError(f"Timed out waiting for response: {name}")

        with self._lock:
            resp = self._responses.pop(request_id, None)
            self._pending.pop(request_id, None)

        if resp is None:
            raise TRON1ProtocolError(f"Missing response for request id: {request_id}")

        if resp.get("type") == "error":
            err = resp.get("error") or {}
            raise TRON1ProtocolError(f"TRON1 error: {err}")

        result = resp.get("result") if "result" in resp else resp.get("data")
        return result if isinstance(result, dict) else {"result": result}

    def _recv_loop(self) -> None:
        while not self._closed and self._ws:
            try:
                raw = self._ws.recv()
            except websocket.WebSocketTimeoutException:
                continue
            except (websocket.WebSocketConnectionClosedException, OSError):
                break

            if isinstance(raw, bytes):
                continue

            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue

            if not isinstance(msg, dict):
                continue

            if protocol.is_notify(msg):
                self._dispatch_notify(msg)
                continue

            if protocol.is_response(msg):
                req_id = str(msg.get("guid") or msg.get("id") or "")
                if not req_id:
                    continue
                with self._lock:
                    self._responses[req_id] = msg
                    pending = self._pending.get(req_id)
                if pending:
                    pending.set()
                continue

    def _dispatch_notify(self, msg: dict[str, Any]) -> None:
        name = protocol.notify_name(msg)
        payload = protocol.notify_payload(msg)
        with self._lock:
            handlers = list(self._notify_handlers.get(name, []))
        for h in handlers:
            try:
                h(payload)
            except Exception:
                logger.exception("TRON1 notify handler failed", name=name)

    @staticmethod
    def _normalize_ws_url(ws_url: str) -> str:
        s = ws_url.strip()
        if s.startswith("ws://") or s.startswith("wss://"):
            return s
        if s.startswith("http://"):
            return "ws://" + s.removeprefix("http://")
        if s.startswith("https://"):
            return "wss://" + s.removeprefix("https://")
        return "ws://" + s
