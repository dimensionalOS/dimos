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
import os
import platform
import socket
import threading
import time
from typing import TYPE_CHECKING, Any, ClassVar

from pydantic import Field, model_validator
import zenoh

if TYPE_CHECKING:
    from collections.abc import Sequence

from dimos.core.global_config import TransportBackend, ZenohMode, global_config
from dimos.protocol.service.spec import Service, SessionConfig
from dimos.utils.logging_config import setup_logger

zenoh.init_log_from_env_or("warn")

logger = setup_logger()

# Zenoh's own default port, which robot-side bridges listen on.
ROBOT_ZENOH_PORT = 7447

# Poll interval while waiting for connect endpoints to link.
_CONNECT_POLL_INTERVAL = 0.05

# Interface scouting falls back to when network discovery is off.
# Darwin spells loopback differently.
LOOPBACK_INTERFACE = "lo0" if platform.system() == "Darwin" else "lo"

# Zenoh's own name for "every multicast-capable interface".
ALL_INTERFACES = "auto"

# Zenoh's own default listener binds every interface, so an unconfigured session
# is reachable from the LAN. A session whose discovery never leaves loopback
# listens here instead: loopback only, on a port the kernel picks.
LOOPBACK_LISTEN = "tcp/127.0.0.1:0"


def _locators(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


# Explicit loopback endpoints wiring the coordinator and its workers together.
# Sibling discovery cannot rely on multicast scouting: macOS never delivers
# multicast pinned to lo0, so with scouting off the processes would never find
# each other. Instead each worker listens on a coordinator-allocated port and
# dials the workers spawned before it, and coordinator-side sessions dial every
# live worker. Process-wide; consumed by the ZenohConfig default factories.
_mesh_listen: str | None = None
_mesh_connect: tuple[str, ...] = ()


def allocate_mesh_endpoint() -> str:
    """Reserve a free loopback port and return it as a ``tcp/`` locator.

    The probe socket closes before zenoh binds the port at session open; a
    collision in that window fails the session open loudly rather than
    silently dropping traffic.
    """
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port: int = sock.getsockname()[1]
    return f"tcp/127.0.0.1:{port}"


def configure_zenoh_mesh(listen: str | None, connect: Sequence[str]) -> None:
    """Set the loopback endpoints this process's future zenoh sessions mesh over.

    Workers pass their own ``listen`` endpoint plus the endpoints of the
    workers spawned before them; the coordinator passes ``None`` and dials all
    live workers. Already-open sessions are not reconfigured: the mesh stays
    complete because every session dials whatever existed when it opened and
    is dialled by everything newer.

    An emptied roster (every worker shut down) also closes the pooled sessions
    that dialed the departed endpoints. Zenoh retries refused dials forever,
    so without this every coordinator lifecycle in a long-lived process (a
    pytest worker, a daemon restarting blueprints) leaks a session whose
    runtime threads and sockets keep hammering dead ports.
    """
    global _mesh_listen, _mesh_connect
    previous = _mesh_connect
    _mesh_listen = listen
    _mesh_connect = tuple(connect)
    if previous and not _mesh_connect:
        default_session_pool.close_dialing(previous)


def _default_connect_endpoints() -> list[str]:
    """Dial known robots and mesh siblings instead of trusting multicast scouting.

    Many APs filter multicast between WiFi clients, so a robot reachable over
    TCP never answers a scout. An IP carrying its own port is used as given.
    """
    out: list[str] = []
    if global_config.transport == "zenoh":
        ips = _locators(f"{global_config.robot_ip or ''},{global_config.robot_ips or ''}")
        robots = [f"tcp/{ip}" if ":" in ip else f"tcp/{ip}:{ROBOT_ZENOH_PORT}" for ip in ips]
        out = list(dict.fromkeys(robots + _locators(global_config.zenoh_connect)))
    out.extend(endpoint for endpoint in _mesh_connect if endpoint not in out)
    return out


def _default_listen_endpoints() -> list[str]:
    return [_mesh_listen] if _mesh_listen else []


def _default_scouting() -> bool:
    return global_config.zenoh_scouting


def _default_scouting_interface() -> str:
    return global_config.zenoh_interface.strip()


def _default_multicast() -> bool:
    return global_config.zenoh_multicast


def _default_scout_addr() -> str:
    return global_config.zenoh_scout_addr.strip()


def _default_gossip() -> bool | None:
    return global_config.zenoh_gossip


def _default_mode() -> ZenohMode:
    return global_config.zenoh_mode


def _default_connect_timeout() -> float:
    return global_config.zenoh_connect_timeout


def endpoint_addresses(endpoint: str) -> set[str]:
    """Resolve a locator to the host:port forms a live link may report.

    A locator dialed by name never matches the address the link reports.
    """
    _, _, address = endpoint.rpartition("/")
    host, _, port = address.rpartition(":")
    if not host:
        return {address}
    out = {f"{host}:{port}"}
    try:
        for info in socket.getaddrinfo(host, None):
            out.add(f"{info[4][0]}:{port}")
    except OSError:
        pass
    return out


class ZenohConfig(SessionConfig):
    transport: ClassVar[TransportBackend] = "zenoh"

    mode: ZenohMode = Field(default_factory=_default_mode)
    connect: list[str] = Field(default_factory=_default_connect_endpoints)
    # Pinned per session or per process (a worker's mesh port), never network
    # global. Only one process can hold a listen port.
    listen: list[str] = Field(default_factory=_default_listen_endpoints)
    # Discover peers across the network. Off keeps discovery on loopback.
    scouting: bool = Field(default_factory=_default_scouting)
    # Named interface to scout on, overriding scouting. Empty derives it.
    scouting_interface: str = Field(default_factory=_default_scouting_interface)
    # Whether multicast scouting runs at all, as opposed to how far it reaches.
    multicast: bool = Field(default_factory=_default_multicast)
    # Multicast group scouting joins. Empty takes zenoh's own. Sessions on
    # different groups never discover each other, even on the same loopback.
    scout_addr: str = Field(default_factory=_default_scout_addr)
    # Learn peers from the peers already linked, not only from the dialed ones.
    # None follows scouting.
    gossip: bool | None = Field(default_factory=_default_gossip)
    # Seconds to block in start() waiting for the connect endpoints to link.
    # Also bounds zenoh's dial retries at open. 0 skips both.
    connect_timeout: float = Field(default_factory=_default_connect_timeout, ge=0, le=86400)

    @model_validator(mode="after")
    def _router_needs_a_listen_endpoint(self) -> ZenohConfig:
        """Reject a router that would fall back to 7447, the robot bridge's port."""
        if self.mode == "router" and not self.listen:
            raise ValueError(
                "zenoh router mode needs an explicit listen endpoint, e.g. "
                f"listen=['tcp/127.0.0.1:{ROBOT_ZENOH_PORT}']"
            )
        return self

    @property
    def multicast_interface(self) -> str:
        return self.scouting_interface or (ALL_INTERFACES if self.scouting else LOOPBACK_INTERFACE)

    @property
    def listen_endpoints(self) -> list[str]:
        """Where this session accepts links, pinned to loopback while discovery is.

        A session that only ever scouts loopback has no reason to be reachable
        from the LAN, so it does not offer zenoh's all-interfaces default there.
        Dialing out unpins: gossip advertises this session's locator to the
        peers behind the dialed endpoint, and a loopback locator would leave
        them unable to link back -- a router never forwards between peers.
        """
        if self.listen or self.mode == "client":
            # A client dials its router and accepts no links, so it has no
            # listener to pin in the first place.
            return self.listen
        if self.connect or self.multicast_interface != LOOPBACK_INTERFACE:
            return []
        return [LOOPBACK_LISTEN]

    @property
    def gossip_enabled(self) -> bool:
        """Gossip discovery, on unless a caller turns it off.

        Off, a peer cannot resolve the key expressions its links send it and
        drops their data with `Route data with unknown scope`.
        """
        return self.gossip if self.gossip is not None else True

    @property
    def session_key(self) -> str:
        """Every setting zenoh sees."""
        return (
            f"{self.mode}|{json.dumps(sorted(self.connect))}"
            f"|{json.dumps(sorted(self.listen_endpoints))}|{self.multicast_interface}"
            f"|{self.multicast}|{self.scout_addr}|{self.gossip_enabled}"
            f"|{self.connect_timeout}"
        )

    def to_wire(self) -> dict[str, Any]:
        """This session as the JSON object a native module reads on stdin."""
        return {
            "mode": self.mode,
            "connect": self.connect,
            "listen": self.listen_endpoints,
            "multicast": self.multicast,
            "scout_addr": self.scout_addr,
            "gossip": self.gossip_enabled,
            "interface": self.multicast_interface,
            "connect_timeout_ms": int(self.connect_timeout * 1000),
        }


def _warn_client_single_link(config: ZenohConfig) -> None:
    """Warn when a client config lists several endpoints. Zenoh keeps one link."""
    if config.mode == "client" and len(config.connect) > 1:
        logger.warning(
            "Zenoh client mode holds a single link, traffic flows only through "
            "the first endpoint that connects",
            connect=sorted(config.connect),
        )


# Wire setting to the zenoh config key that carries it. A python session inserts
# these into a zenoh.Config, a native module reads the same values off stdin.
_ZENOH_KEYS = {
    "mode": "mode",
    "connect": "connect/endpoints",
    "listen": "listen/endpoints",
    "multicast": "scouting/multicast/enabled",
    "scout_addr": "scouting/multicast/address",
    "interface": "scouting/multicast/interface",
    "gossip": "scouting/gossip/enabled",
    "connect_timeout_ms": "connect/timeout_ms",
}

# Settings zenoh decides for itself when we send nothing. An empty listen list
# means its own default port, and a zero timeout means its own retry policy.
_ZENOH_DEFAULTED_WHEN_EMPTY = ("connect", "listen", "scout_addr", "connect_timeout_ms")


def _zenoh_config(config: ZenohConfig) -> zenoh.Config:
    """The zenoh session config these settings open."""
    zconfig = zenoh.Config()
    for name, value in config.to_wire().items():
        if not value and name in _ZENOH_DEFAULTED_WHEN_EMPTY:
            continue
        zconfig.insert_json5(_ZENOH_KEYS[name], json.dumps(value))
    if config.connect:
        # A dial can race a mesh sibling's listener still coming up; zenoh's
        # default 1s initial retry would hold back the first messages. Retry
        # fast, keep the default backoff cap. (Whole object: zenoh rejects
        # inserts at the leaf keys.)
        zconfig.insert_json5(
            "connect/retry",
            json.dumps({"period_init_ms": 100, "period_max_ms": 4000, "period_increase_factor": 2}),
        )
    return zconfig


class ZenohSessionPool:
    def __init__(self) -> None:
        self._sessions: dict[str, zenoh.Session] = {}
        # Endpoints each session dialed at open, for close_dialing().
        self._dialed: dict[str, tuple[str, ...]] = {}
        # Key of the session holding this process's mesh listen endpoint.
        self._mesh_key: str | None = None
        self._lock = threading.Lock()
        self._opened_in_pid: int | None = None

    def acquire(self, config: ZenohConfig) -> zenoh.Session:
        """Open a session for this config, or return the existing shared one."""
        key = config.session_key
        with self._lock:
            if self._opened_in_pid not in (None, os.getpid()):
                # Fail fast: both inherited sessions and new zenoh.open() calls
                # deadlock in a fork child because zenoh's process-global
                # runtime loses its threads at fork.
                raise RuntimeError(
                    f"zenoh sessions were opened in pid {self._opened_in_pid} before this "
                    "process forked; zenoh's runtime does not survive fork. "
                    "Fork or daemonize before any zenoh use."
                )
            if key not in self._sessions:
                if self._mesh_key is not None and _mesh_listen in config.listen_endpoints:
                    # The mesh listen port binds once per process. When session
                    # settings shift after the mesh session opened (a deploy's
                    # host-config sync bringing a runtime robot_ip), the new
                    # config rides the session already holding the port: a
                    # second bind can only fail, and the coordinator reaches
                    # this worker only through that session anyway.
                    logger.warning(
                        "Zenoh config diverged after the mesh session opened, reusing it",
                        held=self._mesh_key,
                        requested=key,
                    )
                    return self._sessions[self._mesh_key]
                _warn_client_single_link(config)
                self._sessions[key] = zenoh.open(_zenoh_config(config))
                self._opened_in_pid = os.getpid()
                self._dialed[key] = tuple(config.connect)
                if _mesh_listen is not None and _mesh_listen in config.listen_endpoints:
                    self._mesh_key = key
                logger.info(
                    "Zenoh session opened",
                    mode=config.mode,
                    connect=config.connect,
                    listen=config.listen_endpoints,
                    multicast_interface=config.multicast_interface,
                    gossip=config.gossip_enabled,
                )
            return self._sessions[key]

    def _close_at_exit(self) -> None:
        """Close pooled sessions at interpreter exit, in the opening process only."""
        if self._opened_in_pid == os.getpid():
            self.close_all()

    def close_dialing(self, endpoints: Sequence[str]) -> None:
        """Close every pooled session that dialed any of these endpoints.

        Runs when the mesh roster empties: the departed workers' ports are
        dead (and the kernel may hand them to an unrelated process later), so
        the sessions dialing them can only burn retries.
        """
        doomed = set(endpoints)
        with self._lock:
            for key in [k for k, dialed in self._dialed.items() if doomed & set(dialed)]:
                try:
                    self._sessions[key].close()
                except zenoh.ZError as e:
                    logger.warning("Zenoh session close failed", session_key=key, error=str(e))
                del self._sessions[key]
                del self._dialed[key]
                if self._mesh_key == key:
                    self._mesh_key = None

    def close_all(self) -> None:
        """Close every pooled session and empty the pool."""
        with self._lock:
            for key, session in self._sessions.items():
                # A close can time out while the session still holds links to
                # unreachable peers. The pool is torn down either way.
                try:
                    session.close()
                except zenoh.ZError as e:
                    logger.warning("Zenoh session close failed", session_key=key, error=str(e))
            self._sessions.clear()
            self._dialed.clear()
            self._mesh_key = None


# Process-default pool used by production code. Constructing it opens no sessions.
default_session_pool = ZenohSessionPool()

# A session's callback threads are non-daemon, and interpreter shutdown joins
# those before atexit callbacks run -- a bare script that touched zenoh would
# hang past its last line. threading's own shutdown hook runs before the join,
# so the pool close rides that instead of atexit. Only the default pool: an
# explicitly constructed pool has an owner who closes it.
threading._register_atexit(  # type: ignore[attr-defined]
    default_session_pool._close_at_exit
)


class ZenohService(Service):
    config: ZenohConfig

    def __init__(self, *, session_pool: ZenohSessionPool | None = None, **kwargs: Any) -> None:
        # session_pool is keyword-only so it never reaches the pydantic config
        # (which is extra="forbid"). It rides the same **kwargs path as mode/connect/listen.
        super().__init__(**kwargs)
        self._session_pool = session_pool or default_session_pool
        self._session: zenoh.Session | None = None

    def __getstate__(self):  # type: ignore[no-untyped-def]
        """Drop the live session, which pyo3 cannot pickle.

        A module travels to its worker by pickle, so anything holding a session
        has to shed it and re-acquire from the pool on the far side -- the same
        move LCMService makes with its own runtime handles.
        """
        state = self.__dict__.copy()
        state.pop("_session", None)
        state.pop("_session_pool", None)
        return state

    def __setstate__(self, state) -> None:  # type: ignore[no-untyped-def]
        self.__dict__.update(state)
        self._session_pool = default_session_pool
        self._session = None

    def start(self) -> None:
        try:
            self._session = self._session_pool.acquire(self.config)
        except zenoh.ZError as e:
            if self.config.mode == "client":
                raise RuntimeError(
                    "zenoh client mode needs a reachable router, none at "
                    f"{self.config.connect or 'any scouted locator'}"
                ) from e
            raise
        self._await_connect(self._session)
        super().start()

    def _await_connect(self, session: zenoh.Session) -> None:
        """Block until the dialed endpoints have links.

        A session opens before its endpoints are dialed, so without this the
        first published messages have nowhere to go.
        """
        # Mesh endpoints are excluded: a sibling's listener only comes up once
        # its own first module deploys, zenoh keeps dialling in the background,
        # and the RPC retry loop rides out the gap. Blocking every session
        # start on them would stall deploys instead.
        pending = {
            ep: endpoint_addresses(ep) for ep in self.config.connect if ep not in _mesh_connect
        }
        if not pending or self.config.connect_timeout <= 0:
            return
        # A client session holds one link. Zenoh dials the endpoints as
        # alternatives and keeps the first that connects.
        needed = 1 if self.config.mode == "client" else len(pending)
        total = len(pending)
        deadline = time.monotonic() + self.config.connect_timeout
        while True:
            linked = {str(link.dst).rpartition("/")[2] for link in session.info.links()}
            for endpoint in [e for e, addrs in pending.items() if addrs & linked]:
                logger.debug(f"Zenoh linked {endpoint}")
                del pending[endpoint]
            if total - len(pending) >= needed:
                return
            if time.monotonic() >= deadline:
                logger.warning(
                    f"Zenoh endpoints not linked after {self.config.connect_timeout}s: "
                    f"{sorted(pending)} - continuing, published messages may be dropped"
                )
                return
            time.sleep(_CONNECT_POLL_INTERVAL)

    @property
    def session(self) -> zenoh.Session:
        if self._session is None:
            raise RuntimeError("Zenoh session not initialized. Call start() first.")
        return self._session
