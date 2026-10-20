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

"""ZeroMQ session: a shared context plus the XSUB/XPUB proxy every socket meets at.

ZeroMQ pub/sub is point-to-point: one side of every PUB/SUB pair has to bind.
To get the many-to-many bus the other dimos transports offer, every publisher
connects to the XSUB side of one proxy and every subscriber to its XPUB side.
The proxy is hosted by whichever process binds its endpoints first; later
processes find the ports taken and connect instead.
"""

from __future__ import annotations

import atexit
import os
import threading
from typing import Any, Literal
import uuid

from pydantic import Field

try:
    import zmq

    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False
    zmq = None  # type: ignore[assignment]

from dimos.core.global_config import global_config
from dimos.protocol.service.spec import BaseConfig, Service
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# How a service relates to the proxy on its endpoints.
#   auto:    host it when the endpoints are free, connect otherwise
#   host:    bind the endpoints, fail if another process holds them
#   connect: never bind, some other process runs the proxy
ProxyMode = Literal["auto", "host", "connect"]

# Seconds to wait for a proxy thread to acknowledge TERMINATE.
_PROXY_STOP_TIMEOUT = 2.0


def _default_pub_endpoint() -> str:
    return global_config.zmq_pub_endpoint


def _default_sub_endpoint() -> str:
    return global_config.zmq_sub_endpoint


class ZMQConfig(BaseConfig):
    """Settings for one ZeroMQ bus.

    Publishers connect to `pub_endpoint` (the proxy's XSUB), subscribers to
    `sub_endpoint` (the proxy's XPUB). Any zmq endpoint works, but automatic
    proxy election relies on a second bind failing, which libzmq only
    guarantees for `tcp://`; with `ipc://` pin one process to `proxy="host"`.
    """

    pub_endpoint: str = Field(default_factory=_default_pub_endpoint)
    sub_endpoint: str = Field(default_factory=_default_sub_endpoint)
    proxy: ProxyMode = "auto"
    # Messages zmq queues per peer before dropping (its high-water mark), on
    # every socket this config opens. 0 is unbounded.
    hwm: int = Field(default=1000, ge=0)
    # Seconds subscribe() waits for the subscription to reach the publishers.
    # 0 returns at once and the first messages after subscribing may be missed.
    settle_timeout: float = Field(default=1.0, ge=0)

    @property
    def endpoints(self) -> tuple[str, str]:
        return (self.pub_endpoint, self.sub_endpoint)


class _Proxy:
    """One XSUB/XPUB proxy, forwarded by libzmq on its own thread."""

    def __init__(self, context: zmq.Context[zmq.Socket[bytes]], config: ZMQConfig) -> None:
        self.config = config
        # Bind the subscriber side first, so a process that loses the race
        # on it never half-binds the publisher side.
        self._xpub = context.socket(zmq.XPUB)
        self._xpub.set_hwm(config.hwm)
        self._xpub.linger = 0
        try:
            self._xpub.bind(config.sub_endpoint)
        except Exception:
            self._xpub.close()
            raise
        self._xsub = context.socket(zmq.XSUB)
        self._xsub.set_hwm(config.hwm)
        self._xsub.linger = 0
        try:
            self._xsub.bind(config.pub_endpoint)
        except zmq.ZMQError as e:
            self._xpub.close()
            self._xsub.close()
            if e.errno == zmq.EADDRINUSE:
                # The subscriber endpoint was free, so whoever holds the
                # publisher endpoint is not a proxy we could join.
                raise RuntimeError(
                    f"ZeroMQ publisher endpoint {config.pub_endpoint} is taken while "
                    f"subscriber endpoint {config.sub_endpoint} was free: no proxy to join"
                ) from e
            raise
        control_endpoint = f"inproc://dimos-zmq-proxy-{uuid.uuid4().hex}"
        self._control_rx = context.socket(zmq.PAIR)
        self._control_rx.linger = 0
        self._control_rx.bind(control_endpoint)
        self._control_tx = context.socket(zmq.PAIR)
        self._control_tx.linger = 0
        self._control_tx.connect(control_endpoint)
        self._thread = threading.Thread(
            target=self._run, name=f"zmq-proxy {config.pub_endpoint}", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        try:
            zmq.proxy_steerable(self._xsub, self._xpub, None, self._control_rx)
        except zmq.ZMQError as e:
            if e.errno != zmq.ETERM:
                logger.error("ZeroMQ proxy stopped on error", error=str(e))
        finally:
            self._xsub.close()
            self._xpub.close()
            self._control_rx.close()

    def close(self) -> None:
        try:
            self._control_tx.send(b"TERMINATE", zmq.NOBLOCK)
        except zmq.ZMQError:
            pass
        self._thread.join(timeout=_PROXY_STOP_TIMEOUT)
        if self._thread.is_alive():
            logger.warning("ZeroMQ proxy did not stop in time", pub=self.config.pub_endpoint)
        self._control_tx.close()


# Proxies this process hosts, by endpoints. They outlive the services that
# started them: another process may still depend on them, and tearing one down
# on the last local stop() would cut it off. Closed at interpreter exit.
_proxies: dict[tuple[str, str], _Proxy] = {}
_proxies_lock = threading.Lock()
_proxies_pid: int | None = None


def _forget_inherited_proxies() -> None:
    """A fork child inherits the registry but not the threads behind it."""
    global _proxies_pid
    if _proxies_pid != os.getpid():
        _proxies.clear()
        _proxies_pid = os.getpid()


def ensure_proxy(context: zmq.Context[zmq.Socket[bytes]], config: ZMQConfig) -> bool:
    """Host the proxy for these endpoints if the config allows and nobody does.

    Returns whether this process hosts it. In `auto` mode an endpoint already
    bound means another process does; `host` mode raises in that case, and
    `connect` mode never binds.
    """
    key = config.endpoints
    with _proxies_lock:
        _forget_inherited_proxies()
        if key in _proxies:
            return True
        if config.proxy == "connect":
            return False
        try:
            proxy = _Proxy(context, config)
        except zmq.ZMQError as e:
            if config.proxy == "host" or e.errno != zmq.EADDRINUSE:
                raise RuntimeError(
                    f"cannot host the ZeroMQ proxy on {config.pub_endpoint} / "
                    f"{config.sub_endpoint}: {e}"
                ) from e
            logger.debug(
                "ZeroMQ proxy endpoints taken, connecting to the existing proxy",
                pub=config.pub_endpoint,
                sub=config.sub_endpoint,
            )
            return False
        _proxies[key] = proxy
        logger.info(
            "ZeroMQ proxy hosted", pub=config.pub_endpoint, sub=config.sub_endpoint, hwm=config.hwm
        )
        return True


def hosted_proxies() -> list[tuple[str, str]]:
    """Endpoints of every proxy this process hosts."""
    with _proxies_lock:
        _forget_inherited_proxies()
        return list(_proxies)


def close_proxies() -> None:
    """Stop every proxy this process hosts, freeing its endpoints."""
    with _proxies_lock:
        _forget_inherited_proxies()
        proxies = list(_proxies.values())
        _proxies.clear()
    for proxy in proxies:
        proxy.close()


atexit.register(close_proxies)


class ZMQService(Service):
    config: ZMQConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._context: zmq.Context[zmq.Socket[bytes]] | None = None
        self._hosts_proxy = False

    def __getstate__(self):  # type: ignore[no-untyped-def]
        """Drop the live context; a module carries its service to a worker by pickle."""
        state = self.__dict__.copy()
        state.pop("_context", None)
        state["_hosts_proxy"] = False
        return state

    def __setstate__(self, state) -> None:  # type: ignore[no-untyped-def]
        self.__dict__.update(state)
        self._context = None

    def start(self) -> None:
        if not ZMQ_AVAILABLE:
            raise RuntimeError(
                "pyzmq is not installed; install the zmq extra: uv sync --extra zmq --inexact"
            )
        # The process-wide context: pyzmq replaces it after a fork.
        self._context = zmq.Context.instance()
        self._hosts_proxy = ensure_proxy(self._context, self.config)
        super().start()

    def stop(self) -> None:
        super().stop()

    @property
    def context(self) -> zmq.Context[zmq.Socket[bytes]]:
        if self._context is None:
            raise RuntimeError("ZeroMQ context not initialized. Call start() first.")
        return self._context

    @property
    def hosts_proxy(self) -> bool:
        """Whether this process runs the proxy behind this service's endpoints."""
        return self._hosts_proxy
