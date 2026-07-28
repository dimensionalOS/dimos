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

"""Shared helpers for relay bridge tests."""

import asyncio
from collections.abc import Callable, Sequence
import time

from dimos.web.relay_bridge.protocol import (
    DataFrame,
    Manifest,
    Msg,
    Sub,
    Subs,
    Watch,
)
from dimos.web.relay_bridge.relay_bridge_module import RelayBridgeModule
from dimos.web.relay_bridge.wt_client import RelayClient


def stop_module(module: RelayBridgeModule) -> None:
    """module.stop(), then reap the loop's to_thread executor threads.

    The framework never shuts down the loop's default executor, so tests that
    drive a to_thread path (spawn/stop of a relay child) would trip the
    conftest thread-leak check on the idle asyncio_* workers.
    """
    loop = module._loop
    module.stop()
    if loop is not None and not loop.is_closed():
        loop.run_until_complete(loop.shutdown_default_executor())


async def next_control(client: RelayClient, timeout: float) -> Msg | None:
    try:
        return await asyncio.wait_for(client._session.control_msgs.get(), timeout)
    except asyncio.TimeoutError:
        return None


async def attach_viewer(
    viewer: RelayClient, robot_id: str, chs: Sequence[str], timeout: float = 10.0
) -> None:
    """Watch a robot and subscribe channels after the manifest confirms the watch."""
    deadline = time.monotonic() + timeout
    while True:
        viewer.send_control(Watch(robotId=robot_id))
        msg = await next_control(viewer, 0.5)
        while msg is not None and not isinstance(msg, Manifest):
            msg = await next_control(viewer, 0.5)
        if isinstance(msg, Manifest):
            break
        if time.monotonic() >= deadline:
            raise TimeoutError(f"no manifest for {robot_id} within {timeout} s")
    for ch in chs:
        viewer.send_control(Sub(ch=ch))


async def wait_subs(
    robot: RelayClient, chs: set[str], timeout: float = 10.0, *, exact: bool = False
) -> None:
    """Wait for a subscription snapshot covering, or exactly matching, ``chs``."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = await next_control(robot, deadline - time.monotonic())
        if not isinstance(msg, Subs):
            continue
        if set(msg.chs) == chs if exact else set(msg.chs) >= chs:
            return
    raise TimeoutError(f"no subs snapshot covering {chs} within {timeout} s")


async def collect_until(
    viewer: RelayClient,
    done: Callable[[list[DataFrame]], bool],
    timeout: float = 10.0,
) -> list[DataFrame]:
    """Consume viewer frames until ``done`` succeeds or the timeout expires."""
    frames: list[DataFrame] = []

    async def consume() -> None:
        async for frame in viewer.frames():
            frames.append(frame)
            if done(frames):
                return

    try:
        await asyncio.wait_for(consume(), timeout)
    except asyncio.TimeoutError:
        pass
    return frames
