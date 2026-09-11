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

"""Verify the six-duck picker, deliberate walking previews and connection prompt."""

import base64
import json
import shutil
import subprocess
import time
from pathlib import Path
from uuid import uuid4

import requests
from demo_browser_soak import Browser

root = Path(__file__).resolve().parents[1]
out = root / "logs/picker-check"
out.mkdir(parents=True, exist_ok=True)
profile = root / "tmp" / f"picker-{uuid4().hex}"
profile.mkdir(parents=True)
browser = None
proc = None
results = {}
origin = json.loads((root / "config/tailnet.json").read_text())["public_origin"]
with requests.Session() as http, (out / "chromium.log").open("w") as log:
    http.trust_env = False
    try:
        before = http.get(origin + "/api/lobby").json()["slots"]
        proc = subprocess.Popen(
            [
                "/usr/bin/chromium",
                "--headless=new",
                "--enable-unsafe-swiftshader",
                "--no-first-run",
                "--no-default-browser-check",
                "--disable-dev-shm-usage",
                "--remote-debugging-port=0",
                f"--user-data-dir={profile}",
                "about:blank",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        end = time.monotonic() + 20
        while not (profile / "DevToolsActivePort").exists():
            if time.monotonic() > end:
                raise TimeoutError("chrome")
            time.sleep(0.2)
        port = int((profile / "DevToolsActivePort").read_text().splitlines()[0])
        root_browser = Browser(
            http.get(f"http://127.0.0.1:{port}/json/version").json()["webSocketDebuggerUrl"]
        )
        context = root_browser.command("Target.createBrowserContext")["browserContextId"]
        target = root_browser.command(
            "Target.createTarget", {"url": "about:blank", "browserContextId": context}
        )["targetId"]
        browser = Browser(f"ws://127.0.0.1:{port}/devtools/page/{target}")

        def ev(js):
            r = browser.command(
                "Runtime.evaluate", {"expression": js, "returnByValue": True, "awaitPromise": True}
            )
            if "exceptionDetails" in r:
                raise RuntimeError(str(r["exceptionDetails"]))
            return r.get("result", {}).get("value")

        def until(js, seconds=60):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                value = ev(js)
                if value:
                    return value
                time.sleep(0.25)
            raise TimeoutError(js + " " + str(ev("document.body.innerText.slice(0,2000)")))

        def shot(name, clip=None):
            params = {"format": "png"}
            if clip:
                params["clip"] = {**clip, "scale": 1}
            data = base64.b64decode(browser.command("Page.captureScreenshot", params)["data"])
            (out / name).write_bytes(data)
            return data

        def move(id=None):
            xy = (
                {"x": 12, "y": 85}
                if id is None
                else ev(
                    "(()=>{const r=document.querySelector('[data-preview="
                    + id
                    + "]').getBoundingClientRect();return{x:r.x+r.width/2,y:r.y+r.height/2};})()"
                )
            )
            browser.command("Input.dispatchMouseEvent", {"type": "mouseMoved", **xy})

        def click(selector):
            xy = ev(
                "(()=>{const e=document.querySelector("
                + json.dumps(selector)
                + ");e.scrollIntoView({block:'center'});const r=e.getBoundingClientRect"
                "();return{x:r.x+r.width/2,y:r.y+r.height/2};})()"
            )
            browser.command("Input.dispatchMouseEvent", {"type": "mouseMoved", **xy})
            browser.command(
                "Input.dispatchMouseEvent",
                {"type": "mousePressed", "button": "left", "clickCount": 1, **xy},
            )
            browser.command(
                "Input.dispatchMouseEvent",
                {"type": "mouseReleased", "button": "left", "clickCount": 1, **xy},
            )

        def frames():
            return ev(
                "Number(document.querySelector('[data-testid=duck-hover-canvas]')?.da"
                "taset.frames||0)"
            )

        browser.command(
            "Emulation.setDeviceMetricsOverride",
            {"width": 1440, "height": 1000, "deviceScaleFactor": 1, "mobile": False},
        )
        browser.command("Page.navigate", {"url": origin})
        until(
            "document.querySelectorAll('[data-testid^=card-duck] img').length===6"
            " && [...document.querySelectorAll('[data-testid^=card-duck] img')].e"
            "very(e=>e.naturalWidth>0)"
        )
        results["cards"] = ev(
            "[...document.querySelectorAll('[data-testid^=choose-duck]')].map(e=>"
            "({id:e.dataset.testid,disabled:e.disabled}))"
        )
        assert len(results["cards"]) == 6
        assert not ev("!!document.querySelector('button[data-team]')")
        assert ev(
            "!document.querySelector('dialog').open && document.querySelector('[d"
            "ata-testid=player-name]').getBoundingClientRect().width===0"
        )
        shot("six-duck-picker.png")
        print("Six visible duck choices, no team navigation or initial name field", flush=True)
        results["hover"] = {}
        for reduced in ["no-preference", "reduce"]:
            browser.command(
                "Emulation.setEmulatedMedia",
                {"features": [{"name": "prefers-reduced-motion", "value": reduced}]},
            )
            for id in ["duck1", "duck2", "duck3", "duck4", "duck5", "duck6"]:
                move()
                time.sleep(0.3)
                a = frames()
                move(id)
                until(
                    "document.querySelector('[data-preview="
                    + id
                    + "]')?.dataset.animated==='true'",
                    10,
                )
                until(
                    "Number(document.querySelector('[data-testid=duck-hover-canvas]')?.da"
                    "taset.frames||0)>" + str(a + 3),
                    10,
                )
                assert ev(
                    "document.querySelectorAll('[data-animated=true]').length===1 && docu"
                    "ment.querySelectorAll('[data-testid=duck-hover-canvas]').length===1"
                )
            results["hover"][reduced] = True
        rect = ev(
            "(()=>{const r=document.querySelector('[data-preview=duck6]').getBoun"
            "dingClientRect();return{x:r.x,y:r.y,width:r.width,height:r.height}})"
            "()"
        )
        a = shot("walk-a.png", rect)
        time.sleep(0.28)
        b = shot("walk-b.png", rect)
        assert a != b
        move()
        until("!document.querySelector('[data-animated=true]')", 10)
        a = frames()
        time.sleep(0.5)
        assert frames() == a
        results["stopsOnExit"] = True
        print(
            "All six previews animate, including reduced-motion; one canvas and stop-on-exit",
            flush=True,
        )
        free = next((s["id"] for s in before if not s["occupied"]), None)
        assert free, "Need one free slot to verify connection"
        click("[data-testid=choose-" + free + "]")
        until("document.querySelector('dialog').open")
        assert ev("document.activeElement.dataset.testid==='player-name'")
        assert not next(
            s for s in http.get(origin + "/api/lobby").json()["slots"] if s["id"] == free
        )["occupied"]
        shot("name-prompt.png")
        browser.command(
            "Input.dispatchKeyEvent",
            {"type": "keyDown", "key": "Escape", "code": "Escape", "windowsVirtualKeyCode": 27},
        )
        browser.command(
            "Input.dispatchKeyEvent",
            {"type": "keyUp", "key": "Escape", "code": "Escape", "windowsVirtualKeyCode": 27},
        )
        until("!document.querySelector('dialog').open")
        assert not next(
            s for s in http.get(origin + "/api/lobby").json()["slots"] if s["id"] == free
        )["occupied"]
        results["cancelDoesNotClaim"] = True
        browser.command(
            "Emulation.setDeviceMetricsOverride",
            {"width": 390, "height": 844, "deviceScaleFactor": 1, "mobile": True},
        )
        ev("document.querySelector('main').scrollTop=0")
        assert ev("document.documentElement.scrollWidth===innerWidth")
        assert ev("document.querySelectorAll('[data-testid^=choose-duck]').length===6")
        shot("picker-mobile.png")
        click("[data-testid=choose-" + free + "]")
        until("document.querySelector('dialog').open")
        shot("name-prompt-mobile.png")
        assert ev("document.querySelector('dialog').getBoundingClientRect().right<=innerWidth")
        ev(
            "(()=>{const e=document.querySelector('[data-testid=player-name]');Ob"
            "ject.getOwnPropertyDescriptor(HTMLInputElement.prototype,'value').se"
            "t.call(e,'Picker check');e.dispatchEvent(new Event('input',{bubbles:"
            "true}));})()"
        )
        click("[data-testid=confirm-join]")
        until(
            "document.querySelector('[data-testid=world-lobby]').dataset.robot==="
            + json.dumps(free)
        )
        until("document.querySelector('[data-testid=leave-world]')?.textContent.includes('Leave')")
        lobby = http.get(origin + "/api/lobby").json()["slots"]
        assert next(s for s in lobby if s["id"] == free)["displayName"] == "Picker check"
        results["nameAtConnection"] = True
        click("[data-testid=leave-world]")
        until("document.querySelector('[data-testid=world-lobby]').dataset.role==='observe'")
        click("[data-testid=observe-world]")
        until("!!document.querySelector('[data-testid=world3d-viewport]')")
        assert not ev("document.querySelector('dialog').open")
        results["spectatorSkipsName"] = True
        results["passed"] = True
        print("Name dialog focus/cancel/mobile/join and spectator path passed", flush=True)
    finally:
        if browser:
            try:
                ev(
                    "(async()=>{const token=sessionStorage.getItem('world-ticket');if(tok"
                    "en)await fetch('/api/lobby',{method:'POST',headers:{'content-type':'"
                    "application/json'},body:JSON.stringify({action:'observe',token})});}"
                    ")()"
                )
            except Exception:
                pass
            browser.socket.close()
        if proc:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        shutil.rmtree(profile, ignore_errors=True)
        (out / "results.json").write_text(json.dumps(results, indent=2))
        print(json.dumps(results), flush=True)
