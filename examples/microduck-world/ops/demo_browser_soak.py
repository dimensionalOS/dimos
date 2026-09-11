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

"""Bounded, read-only browser soak; writes evidence under project logs."""

from __future__ import annotations

import argparse
import base64
import json
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from uuid import uuid4

import requests
from websockets.sync.client import connect

ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT = """
(() => {
 const channels = {};
 for (const name of ['chase_image','color_image','odom','policy_state']) {
  const row = [...document.querySelectorAll('tr')].find(r => r.cells[0]?.textContent === name);
  channels[name] = row ? {
   seq: Number(row.cells[4].textContent), age: row.cells[3].textContent
  } : null;
 }
 const canvas = document.querySelector('[data-testid=video-chase_image-canvas]');
 return {channels, cameraWidth: canvas?.width || 0, visibility: document.visibilityState,
         agentReadOnly: document.querySelector('[data-testid=chat-agent-input]')?.disabled};
})()
"""


class Browser:
    def __init__(self, url: str) -> None:
        self.socket = connect(url, open_timeout=10, close_timeout=5, max_size=16 * 1024 * 1024)
        self.sequence = 0

    def command(self, method: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        self.sequence += 1
        self.socket.send(
            json.dumps({"id": self.sequence, "method": method, "params": params or {}})
        )
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            message = json.loads(self.socket.recv(timeout=max(0.1, deadline - time.monotonic())))
            if message.get("id") == self.sequence:
                if "error" in message:
                    raise RuntimeError(str(message["error"]))
                return dict(message.get("result", {}))
        raise TimeoutError(method)

    def snapshot(self) -> dict[str, Any]:
        result = self.command("Runtime.evaluate", {"expression": SNAPSHOT, "returnByValue": True})
        return dict(result["result"]["value"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hours", type=float, default=6)
    parser.add_argument("--interval", type=float, default=300)
    args = parser.parse_args()
    if not 0 < args.hours <= 12 or args.interval < 10:
        parser.error("hours must be in (0, 12]; interval must be at least 10 seconds")
    config = json.loads((ROOT / "config/tailnet.json").read_text())
    origin = config["public_origin"]
    run = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = ROOT / "logs" / f"browser-soak-{run}"
    output.mkdir(parents=True)
    profile = ROOT / "tmp" / f"browser-soak-{uuid4().hex}"
    profile.mkdir()
    results: list[dict[str, Any]] = []
    browser: Browser | None = None
    http = requests.Session()
    http.trust_env = False
    with (output / "chromium.log").open("w") as browser_log:
        process = subprocess.Popen(
            [
                "/usr/bin/chromium",
                "--headless=new",
                "--disable-gpu",
                "--no-first-run",
                "--no-default-browser-check",
                "--remote-debugging-address=127.0.0.1",
                "--remote-debugging-port=0",
                f"--user-data-dir={profile}",
                f"--disk-cache-dir={ROOT / 'cache/browser-soak'}",
                "about:blank",
            ],
            stdout=browser_log,
            stderr=subprocess.STDOUT,
        )
        try:
            deadline = time.monotonic() + 90
            port_file = profile / "DevToolsActivePort"
            while not port_file.exists():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("Chromium did not start")
                time.sleep(1)
            port = int(port_file.read_text().splitlines()[0])
            pages = http.get(f"http://127.0.0.1:{port}/json", timeout=10).json()
            page = next(page for page in pages if page["type"] == "page")
            browser = Browser(page["webSocketDebuggerUrl"])
            browser.command(
                "Emulation.setDeviceMetricsOverride",
                {
                    "width": 1440,
                    "height": 900,
                    "deviceScaleFactor": 1,
                    "mobile": False,
                },
            )
            browser.command("Page.navigate", {"url": origin})
            time.sleep(30)
            end = time.monotonic() + args.hours * 3600
            next_image = 0.0
            while True:
                now = time.monotonic()
                record: dict[str, Any] = {"time": datetime.now(timezone.utc).isoformat()}
                try:
                    response = http.get(f"{origin}/healthz", timeout=10)
                    record["health"] = response.status_code
                    before = browser.snapshot()
                    time.sleep(2)
                    after = browser.snapshot()
                    record["browser"] = after
                    channels = ("chase_image", "color_image", "odom", "policy_state")
                    record["passed"] = (
                        response.status_code == 200
                        and after["cameraWidth"] == 640
                        and all(
                            before["channels"].get(name)
                            and after["channels"].get(name)
                            and after["channels"][name]["seq"] > before["channels"][name]["seq"]
                            for name in channels
                        )
                    )
                    if now >= next_image or not record["passed"]:
                        shot = browser.command("Page.captureScreenshot", {"format": "png"})
                        filename = f"checkpoint-{len(results):03d}.png"
                        (output / filename).write_bytes(base64.b64decode(shot["data"]))
                        record["screenshot"] = filename
                        next_image = now + 3600
                except Exception as error:
                    record["passed"] = False
                    record["error"] = f"{type(error).__name__}: {error}"
                results.append(record)
                with (output / "checkpoints.jsonl").open("a") as evidence:
                    evidence.write(json.dumps(record) + "\n")
                complete = time.monotonic() >= end
                passed = sum(bool(item["passed"]) for item in results)
                report = (
                    f"# Browser stability run {run}\n\n"
                    f"Status: {'finished' if complete else 'running'}. "
                    f"{passed}/{len(results)} checkpoints passed.\n\n"
                    f"Last check: {record['time']}.\n\n"
                    "Checks: trusted HTTPS readiness, live chase/head video, "
                    "odometry and policy updates "
                    "in a real headless Chromium browser. Screenshots hourly and on failures. "
                    "No motion commands or agent requests are sent.\n\n"
                    f"Evidence: logs/{output.name}/checkpoints.jsonl\n"
                )
                (ROOT / "docs/stability-report.md").write_text(report)
                print(json.dumps(record), flush=True)
                if complete:
                    break
                time.sleep(min(args.interval, max(0, end - time.monotonic())))
        finally:
            if browser is not None:
                browser.socket.close()
            process.terminate()
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            http.close()


if __name__ == "__main__":
    main()
