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

"""Blueprint root: place a hosted Blueprint on live HostDaemons and run it until exit."""

from __future__ import annotations

from collections.abc import Callable, Iterable, Iterator, Mapping
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack, contextmanager
import sys
import threading
import time
from typing import Any

from dimos.core.coordination.blueprints import Blueprint
from dimos.hosted.client import HostClient, describe_all
from dimos.hosted.fragment import HostFragment
from dimos.hosted.fragment_compiler import compile_fragments, resolve_hosted_assignments
from dimos.hosted.machine import code_revision
from dimos.hosted.protocol import RUN_LIVELINESS_KEY, RUN_LOG_KEY, HostDescriptor
from dimos.hosted.tags import format_tags, missing
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.utils.logging_config import setup_logger
from dimos.utils.safe_thread_map import safe_thread_map

logger = setup_logger()

DEFAULT_RPC_TIMEOUT = 120.0
DEFAULT_HOST_TIMEOUT = 30.0
# A describe over wifi takes longer than a liveliness query on the fabric.
DESCRIBE_TIMEOUT = 5.0
HOST_COLOURS = (36, 33, 35, 32, 34, 31)


def wait_for_hosts(
    rpc: ZenohRPC,
    names: set[str],
    timeout: float = DEFAULT_HOST_TIMEOUT,
    tag_sets: Iterable[Mapping[str, str | bool]] = (),
    poll: float = 0.2,
) -> tuple[HostDescriptor, ...]:
    """Discover until every named Host and some Host per tag set is live, or raise."""
    tag_sets = tuple(tag_sets)
    deadline = time.monotonic() + timeout
    descriptors: tuple[HostDescriptor, ...] = ()
    while time.monotonic() < deadline:
        try:
            descriptors = describe_all(rpc, timeout=1.0, rpc_timeout=DESCRIBE_TIMEOUT)
        except TimeoutError:
            descriptors = ()
        named = names <= {d.name for d in descriptors} | {d.host_id for d in descriptors}
        tagged = all(any(not missing(d.tags, tags) for d in descriptors) for tags in tag_sets)
        if named and tagged:
            return descriptors
        time.sleep(poll)
    found = ", ".join(sorted(f"{d.name}[{format_tags(d.tags)}]" for d in descriptors)) or "none"
    wanted = [*sorted(names), *(missing({}, tags) for tags in tag_sets)]
    raise TimeoutError(f"Hosts {wanted} not all live; found: {found}")


def log_printer() -> Callable[[str, str], None]:
    """Print each fragment line behind its Host's name, a colour per Host on a tty."""
    colours: dict[str, int] = {}
    lock = threading.Lock()

    def emit(host: str, line: str) -> None:
        tag = f"{host:>12} |"
        if sys.stdout.isatty():
            colour = colours.setdefault(host, HOST_COLOURS[len(colours) % len(HOST_COLOURS)])
            tag = f"\x1b[{colour}m{tag}\x1b[0m"
        with lock:
            print(f"{tag} {line}", flush=True)

    return emit


@contextmanager
def deployed(
    blueprint: Blueprint,
    rpc: ZenohRPC,
    *,
    run_id: str,
    local_host_id: str | None,
    descriptors: tuple[HostDescriptor, ...],
    overrides: Mapping[str, Any] | None = None,
    global_overrides: Mapping[str, Any] | None = None,
    rpc_timeout: float = DEFAULT_RPC_TIMEOUT,
    on_log: Callable[[str, str], None] | None = None,
    start_poll: float = 1.0,
) -> Iterator[Mapping[str, str]]:
    """Start one fragment per placed Host; yield module -> Host name; stop them on exit.

    Unplaced modules go to ``local_host_id``, the daemon serving this machine. ``overrides``
    and ``global_overrides`` are the blueprint root's explicit configuration (see
    ``ParsedBlueprintConfig.file_and_cli``); each Host parses them over its own
    environment. ``on_log`` gets (host name, line) for every output line of this run's
    fragments. The run's liveliness token is held for the block: a Host stops the run
    once it is gone.
    """
    names = {d.host_id: d.name for d in descriptors}
    revision = code_revision()
    assignments = resolve_hosted_assignments(
        blueprint, descriptors, local_host_id=local_host_id, revision=revision
    )
    fragments = compile_fragments(
        blueprint,
        assignments,
        run_id=run_id,
        revision=revision,
        overrides=overrides,
        global_overrides=global_overrides,
    )
    if not fragments:
        raise ValueError("Nothing to place: every module of the Blueprint is disabled")
    placement = {module: names[host_id] for module, host_id in sorted(assignments.items())}
    logger.info("Placement", run_id=run_id, placement=placement)
    # A Host answers a start within its own limit, which may exceed the RPC default.
    start_timeouts = {d.host_id: max(rpc_timeout, d.start_timeout) for d in descriptors}

    def deliver(sample: Any) -> None:
        host_id = str(sample.key_expr).rsplit("/", 1)[1]
        line = sample.payload.to_bytes().decode("utf-8", "replace").rstrip("\n")
        on_log(names.get(host_id, host_id), line)  # type: ignore[misc]

    def start(item: tuple[HostClient, HostFragment]) -> None:
        client, fragment = item
        status = client.start(fragment)
        # A repeated start (zenoh re-sends a query its link dropped) answers with the state so far.
        while status.state == "starting":
            time.sleep(start_poll)
            status = client.status(run_id)
        if status.state != "running":
            raise RuntimeError(f"Host {names[client.host_id]} failed to start {run_id}: {status}")

    def stop(item: tuple[HostClient, HostFragment]) -> None:
        client, _ = item
        try:
            client.stop(run_id)
        except Exception:
            logger.error("Failed to stop fragment", host=names[client.host_id], exc_info=True)

    with ExitStack() as cleanup:
        token = rpc.session.liveliness().declare_token(RUN_LIVELINESS_KEY.format(run_id=run_id))
        cleanup.callback(token.undeclare)
        if on_log is not None:
            subscriber = rpc.session.declare_subscriber(
                RUN_LOG_KEY.format(run_id=run_id, host="*"), deliver
            )
            cleanup.callback(subscriber.undeclare)
        # Registered before any start: a Host keeps an accepted run even if its reply never arrives.
        started = [
            (HostClient(rpc, host_id, timeout=start_timeouts[host_id]), fragment)
            for host_id, fragment in fragments.items()
        ]
        cleanup.callback(safe_thread_map, started, stop)
        # Side by side: a module may call a remote ref on another Host while starting.
        pool = ThreadPoolExecutor(max_workers=len(started))
        try:
            list(pool.map(start, started))
        finally:
            # A failure or Ctrl-C stops the run at once; the stop cuts the starts still in flight.
            pool.shutdown(wait=False)
        yield placement
