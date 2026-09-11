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

"""Bounded browser checkpoint, using only a free lobby slot and its UI controls."""

import base64
import json
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any
from uuid import uuid4

import requests
from demo_browser_soak import Browser
from microduck_world.scene import PROJECT_ROOT

ORIGIN = json.loads((PROJECT_ROOT / "config/tailnet.json").read_text())["public_origin"]


def evaluate(browser: Browser, expression: str) -> Any:
    expression = (
        "(()=>{const q=id=>document.querySelector(`[data-testid=${id}]`);"
        "const row=name=>JSON.parse([...document.querySelectorAll('tr')]"
        ".find(r=>r.cells[0]?.textContent===name)?.cells[5]?.textContent||'null');"
        f"return ({expression});}})()"
    )
    result = browser.command(
        "Runtime.evaluate", {"expression": expression, "returnByValue": True, "awaitPromise": True}
    )
    if "exceptionDetails" in result:
        raise RuntimeError(str(result["exceptionDetails"]))
    return result["result"].get("value")


def poll(browser: Browser, expression: str, timeout: float = 35) -> Any:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = evaluate(browser, expression)
        if result:
            return result
        time.sleep(0.2)
    raise TimeoutError(expression)


def click(browser: Browser, selector: str) -> None:
    evaluate(browser, f"document.querySelector({json.dumps(selector)}).click()")


def channel(browser: Browser, name: str) -> Any:
    return evaluate(
        browser,
        f"row({json.dumps(name)})",
    )


def shot(browser: Browser, directory: Path, name: str) -> None:
    image = browser.command("Page.captureScreenshot", {"format": "png"})
    (directory / name).write_bytes(base64.b64decode(image["data"]))


def main() -> None:
    directory = PROJECT_ROOT / "logs/football-checkpoint"
    directory.mkdir(parents=True, exist_ok=True)
    profile = PROJECT_ROOT / "tmp" / ("football-browser-" + uuid4().hex)
    profile.mkdir()
    page = root = None
    joined = False
    report = {}
    with requests.Session() as http, (directory / "browser.log").open("w") as log:
        http.trust_env = False
        process = subprocess.Popen(
            [
                "/usr/bin/chromium",
                "--headless=new",
                "--no-first-run",
                "--no-default-browser-check",
                "--disable-dev-shm-usage",
                "--enable-unsafe-swiftshader",
                "--use-gl=angle",
                "--use-angle=swiftshader",
                "--remote-debugging-address=127.0.0.1",
                "--remote-debugging-port=0",
                f"--user-data-dir={profile}",
                "about:blank",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        try:
            deadline = time.monotonic() + 15
            while not (profile / "DevToolsActivePort").exists():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("Chromium did not start")
                time.sleep(0.1)
            port = int((profile / "DevToolsActivePort").read_text().splitlines()[0])
            root = Browser(
                http.get(f"http://127.0.0.1:{port}/json/version", timeout=3).json()[
                    "webSocketDebuggerUrl"
                ]
            )
            target = root.command("Target.createTarget", {"url": ORIGIN})["targetId"]
            pages = http.get(f"http://127.0.0.1:{port}/json", timeout=3).json()
            page = Browser(next(p["webSocketDebuggerUrl"] for p in pages if p["id"] == target))
            page.command(
                "Emulation.setDeviceMetricsOverride",
                {"width": 1440, "height": 1000, "deviceScaleFactor": 1, "mobile": False},
            )
            poll(
                page,
                "q('choose-duck1') && !q('choose-duck1').disabled",
            )
            shot(page, directory, "lobby-desktop.png")
            click(page, "[data-testid=choose-duck1]")
            joined = True
            poll(
                page,
                "q('world3d-viewport')?.dataset.live === 'true'",
                60,
            )
            click(page, "[data-testid=world3d-football]")
            poll(
                page,
                "Number(q('world3d-viewport')?.dataset.frame) > 5",
            )
            report["desktop"] = evaluate(
                page,
                "({score:q('football-score')?.textContent,"
                "world:{...q('world3d-viewport').dataset},"
                "pov:{...q('duck3d-viewport').dataset},"
                "fps:[...document.querySelectorAll('[data-testid$=\"-fps\"]')].map(e=>e.textContent)})",
            )
            shot(page, directory, "browser-pitch.png")
            report["timer"] = evaluate(
                page,
                "new Promise(resolve=>{const gaps=[];"
                "let last=performance.now();"
                "const id=setInterval(()=>{const now=performance.now();"
                "gaps.push(now-last);last=now;"
                "if(gaps.length>=12){clearInterval(id);"
                "resolve({max:Math.max(...gaps),"
                "mean:gaps.reduce((a,b)=>a+b,"
                "0)/gaps.length});}},50);})",
            )
            # SwiftShader can block heartbeats on this server. Verify driving with
            # both GL contexts released while retaining the 3D visual checkpoint.
            for prefix in ("world3d", "duck3d"):
                evaluate(
                    page,
                    f"(()=>{{const e=document.querySelector('[data-testid={prefix}-renderer]');"
                    f"e.value='jpeg';e.dispatchEvent(new Event('change',"
                    f"{{bubbles:true}}));}})()",
                )
            poll(page, "q('duck3d-jpeg')?.width===640")
            before = channel(page, "odom")
            evaluate(
                page,
                "q('teleop-tele_cmd_vel').focus()",
            )
            poll(
                page,
                "document.querySelector('[data-testid=teleop-tele_cmd_vel]')?.dataset.state==='armed'",
            )
            page.command(
                "Input.dispatchKeyEvent",
                {"type": "keyDown", "key": "w", "code": "KeyW", "windowsVirtualKeyCode": 87},
            )
            try:
                moved = poll(
                    page,
                    f"(()=>{{const p=row('odom');"
                    f"return p&&Math.hypot(p.x-({before['x']}),"
                    f"p.y-({before['y']}))>.04&&p;"
                    f"}})()",
                    8,
                )
            except TimeoutError:
                report["driveFailure"] = {
                    "odom": channel(page, "odom"),
                    "policy": channel(page, "policy_state"),
                    "ui": evaluate(
                        page,
                        "({focus:document.activeElement?.outerHTML.slice(0,"
                        "200),pad:document.querySelector('[data-testid=teleop-tele_cmd_vel]')?.textContent,"
                        "hasFocus:document.hasFocus()})",
                    ),
                }
                shot(page, directory, "drive-failure.png")
                raise
            finally:
                page.command(
                    "Input.dispatchKeyEvent",
                    {"type": "keyUp", "key": "w", "code": "KeyW", "windowsVirtualKeyCode": 87},
                )
            report["movement"] = {"before": before, "after": moved}
            old_count = channel(page, "policy_state")["respawns"]
            click(page, "[data-testid=duck-respawn]")
            poll(
                page,
                f"(()=>{{return row('policy_state')?.respawns>{old_count};}})()",
                10,
            )
            report["respawn"] = {
                "before": old_count,
                "after": channel(page, "policy_state")["respawns"],
            }
            evaluate(
                page,
                "(()=>{const e=q('duck3d-renderer');"
                "e.value='jpeg';e.dispatchEvent(new Event('change',"
                "{bubbles:true}));})()",
            )
            poll(page, "q('duck3d-jpeg')?.width===640")
            report["jpeg"] = evaluate(
                page,
                "({width:q('duck3d-jpeg').width,height:q('duck3d-jpeg').height})",
            )
            shot(page, directory, "camera-and-respawn.png")
            click(page, "[data-testid=leave-world]")
            joined = False
            poll(
                page,
                "q('observe-world') && !q('observe-world').disabled",
            )
            click(page, "[data-testid=observe-world]")
            joined = True
            poll(
                page,
                "q('world3d-viewport')?.dataset.live==='true'",
            )
            click(page, "[data-testid=world3d-football]")
            report["observer"] = evaluate(
                page,
                "({respawn:!!q('duck-respawn'),"
                "teleop:!!q('teleop-tele_cmd_vel'),"
                "score:q('football-score')?.textContent})",
            )
            assert report["observer"]["respawn"] is False and report["observer"]["teleop"] is False
            shot(page, directory, "spectator-pitch.png")
            click(page, "[data-testid=leave-world]")
            joined = False
            page.command(
                "Emulation.setDeviceMetricsOverride",
                {"width": 390, "height": 844, "deviceScaleFactor": 1, "mobile": True},
            )
            poll(page, "!!q('choose-duck1')")
            report["mobile"] = evaluate(
                page,
                "({width:innerWidth,scroll:document.documentElement.scrollWidth,"
                "football:document.querySelector('[aria-label^=\"World map with the apartment\"]')"
                "?.textContent.includes('Football')})",
            )
            assert report["mobile"]["scroll"] <= report["mobile"]["width"]
            shot(page, directory, "lobby-mobile.png")
            print(json.dumps(report, indent=2))
        finally:
            if page:
                if joined:
                    evaluate(page, "q('leave-world')?.click()")
                page.socket.close()
            if root:
                root.socket.close()
            process.terminate()
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            shutil.rmtree(profile)
            (directory / "browser.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
