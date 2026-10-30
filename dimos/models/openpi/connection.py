# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Bounded, closeable connection using the OpenPI wire protocol."""

from typing import Any

from websockets.sync.client import connect

try:
    from openpi_client import msgpack_numpy  # type: ignore[import-untyped]
except ImportError:
    msgpack_numpy = None


class OpenPIConnection:
    def __init__(self, host: str, port: int, timeout_s: float) -> None:
        if msgpack_numpy is None:
            raise RuntimeError("Install openpi-client in the DimOS environment")
        self._timeout_s = timeout_s
        self._packer = msgpack_numpy.Packer()
        uri = host if host.startswith(("ws://", "wss://")) else f"ws://{host}"
        self._socket = connect(
            f"{uri}:{port}",
            compression=None,
            max_size=None,
            open_timeout=timeout_s,
            close_timeout=1.0,
        )
        try:
            self.metadata = self._decode(self._socket.recv(timeout=timeout_s))
        except Exception:
            self.close()
            raise

    def _decode(self, response: str | bytes) -> dict[str, Any]:
        if isinstance(response, str):
            raise RuntimeError(f"OpenPI server error: {response}")
        if msgpack_numpy is None:
            raise RuntimeError("openpi-client is unavailable")
        result = msgpack_numpy.unpackb(response)
        if not isinstance(result, dict):
            raise ValueError("OpenPI response must be a mapping")
        return result

    def infer(self, observation: dict[str, Any]) -> dict[str, Any]:
        try:
            self._socket.send(self._packer.pack(observation))
            return self._decode(self._socket.recv(timeout=self._timeout_s))
        except Exception:
            # A late reply must never be interpreted as the next request's action.
            self.close()
            raise

    def close(self) -> None:
        self._socket.close()
