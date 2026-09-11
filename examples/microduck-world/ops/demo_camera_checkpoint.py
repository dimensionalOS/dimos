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

"""Bounded browser check for the project camera panels; no agent API calls."""

import base64
import json
import shutil
import subprocess
import time
from pathlib import Path
from uuid import uuid4

import requests
from demo_browser_soak import Browser
from demo_world_observer import WorldObserver

ROOT = Path(__file__).resolve().parents[1]
READ = """(() => {
 const row = [...document.querySelectorAll('tbody tr')]
  .find(r => r.cells[0]?.textContent === 'world_state');
 const raw = row?.cells[5]?.textContent;
 const world = null;
 return {world,
  fps: [...document.querySelectorAll('[data-testid$="3d-fps"]')].map(e => e.textContent),
  views: [...document.querySelectorAll('[data-testid$="3d-viewport"]')]
   .map(e => ({...e.dataset, display: getComputedStyle(e).display})),
  drive: document.querySelector('[data-testid=teleop-tele_cmd_vel]')?.dataset.state,
  notices: [...document.querySelectorAll('[role=status]')].map(e => e.textContent)};
})()"""


def main():
    profile = ROOT / "tmp" / f"camera-check-{uuid4().hex}"
    profile.mkdir()
    origin = json.loads((ROOT / "config/tailnet.json").read_text())["public_origin"]
    browser = None
    results = {}
    with requests.Session() as http, (ROOT / "logs/camera-check-chromium.log").open("w") as log:
        http.trust_env = False
        process = subprocess.Popen(
            [
                "/usr/bin/chromium",
                "--headless=new",
                "--enable-unsafe-swiftshader",
                "--no-first-run",
                "--no-default-browser-check",
                "--disable-dev-shm-usage",
                "--remote-debugging-address=127.0.0.1",
                "--remote-debugging-port=0",
                f"--user-data-dir={profile}",
                "about:blank",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        try:
            deadline = time.monotonic() + 30
            while not (profile / "DevToolsActivePort").exists():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("Chromium failed to start")
                time.sleep(0.2)
            port = int((profile / "DevToolsActivePort").read_text().splitlines()[0])
            page = next(
                p
                for p in http.get(f"http://127.0.0.1:{port}/json", timeout=5).json()
                if p["type"] == "page"
            )
            browser = Browser(page["webSocketDebuggerUrl"])
            browser.command(
                "Emulation.setDeviceMetricsOverride",
                {"width": 1440, "height": 900, "deviceScaleFactor": 1, "mobile": False},
            )
            browser.command("Page.navigate", {"url": origin + "?host"})

            def evaluate(js):
                reply = browser.command(
                    "Runtime.evaluate",
                    {"expression": js, "returnByValue": True, "awaitPromise": True},
                )
                if "exceptionDetails" in reply:
                    raise RuntimeError(str(reply["exceptionDetails"]))
                value = reply.get("result", {}).get("value")
                if js == READ:
                    value["world"] = observer.latest
                return value

            def until(predicate):
                limit = time.monotonic() + 45
                while time.monotonic() < limit:
                    state = evaluate(READ)
                    if predicate(state):
                        return state
                    time.sleep(0.5)
                raise TimeoutError(str(state))

            def select(prefix, value):
                evaluate(
                    f"(() => {{const e=document.querySelector('[data-testid={prefix}-renderer]');"
                    f"e.value='{value}';e.dispatchEvent(new Event('change',{{bubbles:true}}));}})()"
                )

            def shot(name):
                result = browser.command("Page.captureScreenshot", {"format": "png"})
                (ROOT / "logs" / f"camera-{name}.png").write_bytes(base64.b64decode(result["data"]))

            results["three"] = until(
                lambda s: len(s["views"]) == 2 and all(v.get("live") == "true" for v in s["views"])
            )
            shot("three")
            print("three ready and captured", flush=True)
            select("world3d", "jpeg")
            select("duck3d", "jpeg")
            results["jpeg"] = until(
                lambda s: (
                    s["world"]["comparison"]["frames"] > 10
                    and all("FPS" in fps for fps in s["fps"])
                )
            )
            assert all(v["display"] == "none" for v in results["jpeg"]["views"])
            shot("jpeg")
            print("jpeg ready and captured", flush=True)
            select("world3d", "three")
            select("duck3d", "three")
            results["idle"] = until(
                lambda s: (
                    not s["world"]["comparison"]["active"]
                    and all(v["display"] != "none" for v in s["views"])
                )
            )
            time.sleep(2)
            later = evaluate(READ)
            assert (
                later["world"]["comparison"]["frames"]
                == results["idle"]["world"]["comparison"]["frames"]
            )
            print("comparison idle confirmed", flush=True)
            # This headless host uses software WebGL. Pause its expensive world shadows
            # during the control check; keep the new Three.js duck camera live.
            select("world3d", "jpeg")
            until(lambda s: s["world"]["comparison"]["active"])
            evaluate("document.querySelector('[data-testid=teleop-tele_cmd_vel]').click()")
            before = until(lambda s: s["drive"] == "armed")
            evaluate("""(async()=>{
                const pad=document.querySelector('[data-testid=teleop-tele_cmd_vel]');try{
                pad.dispatchEvent(new KeyboardEvent('keydown',{key:'w',code:'KeyW',bubbles:true}));
                await new Promise(r=>setTimeout(r,2500));
              }finally{
                pad.dispatchEvent(new KeyboardEvent('keyup',{key:'w',code:'KeyW',bubbles:true}));
              }})()""")
            after = until(
                lambda s: abs(s["world"]["poses"][1][0] - before["world"]["poses"][1][0]) > 0.03
            )
            results["movement"] = {"before": before["views"], "after": after["views"]}
            evaluate("document.querySelector('[data-testid=world3d-canvas]').focus()")
            select("world3d", "three")
            until(lambda s: not s["world"]["comparison"]["active"])
            print("movement and POV tracking passed", flush=True)
            (ROOT / "logs/camera-checkpoint.json").write_text(json.dumps(results, indent=2))
        finally:
            if browser is not None:
                browser.socket.close()
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            shutil.rmtree(profile, ignore_errors=True)


if __name__ == "__main__":
    with WorldObserver() as observer:
        main()
