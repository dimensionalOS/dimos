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

"""Bounded protocol load against the isolated localhost relay, without browser rendering."""

import asyncio
import json
import time
from collections import Counter
from pathlib import Path

import requests
from dimos.web.relay_bridge.protocol import Sub, Watch
from dimos.web.relay_bridge.wt_client import RelayClient

BASE = "http://127.0.0.1:7782"
ROOT = Path(__file__).resolve().parents[1]


async def main() -> None:
    tickets = []
    tasks = []
    measured = False
    stats = []
    http = requests.Session()
    http.trust_env = False

    def lobby(**body):
        response = http.post(BASE + "/api/lobby", json=body, timeout=10)
        response.raise_for_status()
        return response.json()

    async def observe(index, entry, info, ready):
        channels = {"world_state", f"duck{index % 6 + 1}_ball_camera"}
        counts = Counter()
        sample = {
            "client": index,
            "role": entry["role"],
            "frames": counts,
            "bytes": 0,
            "ready": False,
            "cameraReady": 0,
            "cameraUnavailable": 0,
            "cameraAgeMax": 0.0,
        }
        stats.append(sample)
        async with await RelayClient.connect(info["wtUrl"], "viewer") as client:
            await client.hello()

            async def subscribe():
                while True:
                    client.send_control(Watch(robotId=entry["runtime"]))
                    await client.ping()
                    for channel in channels:
                        client.send_control(Sub(ch=channel))
                    await asyncio.sleep(1)

            subscribing = asyncio.create_task(subscribe())
            try:
                async for frame in client.frames():
                    if frame.header.ch not in channels:
                        continue
                    message = json.loads(frame.payload)
                    if frame.header.ch == "world_state":
                        sample["ready"] = True
                        ready.set()
                    if not measured:
                        continue
                    counts[frame.header.ch] += 1
                    sample["bytes"] += len(frame.payload)
                    if frame.header.ch == "world_state":
                        sample.setdefault("firstSimTime", message["simTime"])
                        sample.setdefault("firstWallTime", time.monotonic())
                        sample["lastSimTime"] = message["simTime"]
                        sample["lastWallTime"] = time.monotonic()
                    else:
                        sample[
                            "cameraReady" if message["status"] == "ready" else "cameraUnavailable"
                        ] += 1
                        sample["cameraAgeMax"] = max(
                            sample["cameraAgeMax"], time.time() - message["ts"]
                        )
            finally:
                subscribing.cancel()
                await asyncio.gather(subscribing, return_exceptions=True)

    try:
        initial = http.get(BASE + "/api/lobby", timeout=10).json()
        if any(slot["occupied"] for slot in initial["slots"]):
            raise RuntimeError("The isolated test world must have all six slots free")
        readiness = []
        for index in range(26):
            entry = lobby(action="create")
            tickets.append(entry["token"])
            if index < 6:
                entry = lobby(
                    action="join",
                    token=entry["token"],
                    robot=f"duck{index + 1}",
                    displayName=f"Load test {index + 1}",
                )
            info = http.get(BASE + "/sessions/" + entry["token"] + "/api/info", timeout=10).json()
            ready = asyncio.Event()
            readiness.append(ready)
            tasks.append(asyncio.create_task(observe(index, entry, info, ready)))
        await asyncio.wait_for(asyncio.gather(*(r.wait() for r in readiness)), 90)
        await asyncio.sleep(15)
        measured = True
        started = time.monotonic()
        await asyncio.sleep(45)
        duration = time.monotonic() - started
        for sample in stats:
            wall = sample.get("lastWallTime", 0) - sample.get("firstWallTime", 0)
            sample["realTimeFactor"] = (
                (sample.get("lastSimTime", 0) - sample.get("firstSimTime", 0)) / wall
                if wall > 0
                else 0
            )
            sample["worldHz"] = sample["frames"]["world_state"] / duration
            sample["cameraHz"] = sample["cameraReady"] / duration
            for key in ["firstWallTime", "lastWallTime", "firstSimTime", "lastSimTime"]:
                sample.pop(key, None)
        failures = [
            str(t.exception()) for t in tasks if t.done() and not t.cancelled() and t.exception()
        ]
        report = {
            "players": 6,
            "spectators": 20,
            "durationSeconds": duration,
            "transport": "local DimOS WebTransport",
            "cameraSubscriptions": 26,
            "failures": failures,
            "clients": sorted(stats, key=lambda s: s["client"]),
        }
        path = ROOT / "state/validation/public-capacity.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
        if failures or any(s["cameraReady"] == 0 for s in stats):
            raise RuntimeError("Some clients failed to receive camera data")
    finally:
        measured = False
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        for ticket in tickets:
            try:
                lobby(action="observe", token=ticket)
            except requests.RequestException:
                pass
        http.close()


if __name__ == "__main__":
    asyncio.run(main())
