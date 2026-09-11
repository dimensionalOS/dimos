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

"""Bounded multiplayer browser checkpoint using isolated Chromium contexts."""

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
 const row=[...document.querySelectorAll('tbody tr')]
   .find(r=>r.cells[0]?.textContent==='world_state');
 let state=null; try {state=JSON.parse(row?.cells[5]?.textContent)} catch {}
 return {role:document.querySelector('[data-testid=world-lobby]')?.dataset.role ?? null,
 robot:document.querySelector('[data-testid=world-lobby]')?.dataset.robot ?? null,
 capacity:document.querySelector('[data-testid=world-capacity]')?.textContent,
 views:[...document.querySelectorAll('[data-testid$="3d-viewport"]')].map(e=>({...e.dataset})),
 drive:document.querySelector('[data-testid=teleop-tele_cmd_vel]')?.dataset.state,
 notices:[...document.querySelectorAll('[role=status],[role=alert]')].map(e=>e.textContent),
 choices:[...document.querySelectorAll('[data-testid^="choose-duck"]')].map(e=>({id:e.dataset.testid,disabled:e.disabled})),
 portraits:[...document.querySelectorAll('article img')]
   .filter(e=>e.complete && e.naturalWidth).length,
 overflow:document.documentElement.scrollWidth>innerWidth,
 state};})()"""


def evaluate(page, js):
    result = page.command(
        "Runtime.evaluate",
        {
            "expression": js,
            "returnByValue": True,
            "awaitPromise": True,
        },
    )
    if "exceptionDetails" in result:
        raise RuntimeError(str(result["exceptionDetails"]))
    value = result.get("result", {}).get("value")
    if js == READ:
        value["state"] = observer.latest
    return value


def until(page, predicate, seconds=100):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        state = evaluate(page, READ)
        if predicate(state):
            return state
        time.sleep(0.3)
    raise TimeoutError(json.dumps(state)[:1800])


def click(page, testid):
    evaluate(page, f"document.querySelector('[data-testid={testid}]').click()")


def main():
    global observer
    origin = json.loads((ROOT / "config/tailnet.json").read_text())["public_origin"]
    profile = ROOT / "tmp" / f"multiplayer-check-{uuid4().hex}"
    profile.mkdir()
    pages = []
    results = {}
    process = None
    with requests.Session() as http, (ROOT / "logs/multiplayer-chromium.log").open("w") as log:
        http.trust_env = False
        try:
            lobby = http.get(origin + "/api/lobby", timeout=3).json()
            if any(slot["occupied"] for slot in lobby["slots"]):
                raise RuntimeError("This checkpoint needs all three duck slots free.")
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
            deadline = time.monotonic() + 30
            while not (profile / "DevToolsActivePort").exists():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("Chromium did not start")
                time.sleep(0.2)
            port = int((profile / "DevToolsActivePort").read_text().splitlines()[0])
            info = http.get(f"http://127.0.0.1:{port}/json/version", timeout=3).json()
            root = Browser(info["webSocketDebuggerUrl"])
            for index in range(4):
                context = root.command("Target.createBrowserContext", {})["browserContextId"]
                target = root.command(
                    "Target.createTarget",
                    {
                        "url": "about:blank",
                        "browserContextId": context,
                    },
                )["targetId"]
                page = Browser(f"ws://127.0.0.1:{port}/devtools/page/{target}")
                pages.append(page)
                page.command(
                    "Emulation.setDeviceMetricsOverride",
                    {
                        "width": 1440,
                        "height": 1000,
                        "deviceScaleFactor": 1,
                        "mobile": False,
                    },
                )
                page.command("Page.navigate", {"url": origin})
                until(page, lambda s: s["role"] == "observe")
                if index == 0:
                    until(page, lambda s: s["portraits"] == 3)
                    assert not evaluate(page, READ)["overflow"]
                    shot = page.command("Page.captureScreenshot", {"format": "png"})
                    (ROOT / "logs/multiplayer-lobby.png").write_bytes(
                        base64.b64decode(shot["data"])
                    )
                    page.command(
                        "Emulation.setDeviceMetricsOverride",
                        {
                            "width": 390,
                            "height": 844,
                            "deviceScaleFactor": 1,
                            "mobile": True,
                        },
                    )
                    assert not evaluate(page, READ)["overflow"]
                    shot = page.command("Page.captureScreenshot", {"format": "png"})
                    (ROOT / "logs/multiplayer-lobby-mobile.png").write_bytes(
                        base64.b64decode(shot["data"])
                    )
                    page.command(
                        "Emulation.setDeviceMetricsOverride",
                        {
                            "width": 1440,
                            "height": 1000,
                            "deviceScaleFactor": 1,
                            "mobile": False,
                        },
                    )
                page.command(
                    "Emulation.setDeviceMetricsOverride",
                    {
                        "width": 1000,
                        "height": 700,
                        "deviceScaleFactor": 1,
                        "mobile": False,
                    },
                )
                if index < 3:
                    robot = ["duck2", "duck3", "duck1"][index]
                    click(page, f"choose-{robot}")
                    until(page, lambda s: s["robot"] == robot and s["role"] != "observe")
                    until(
                        page,
                        lambda s: (
                            len(s["views"]) == 2
                            and all(v.get("live") == "true" for v in s["views"])
                        ),
                    )
                    results[robot] = evaluate(page, READ)
                    # Each Three.js view has been verified. Use native feeds while
                    # opening the next context so software rendering does not
                    # starve the transport and teleop timers on this test server.
                    evaluate(
                        page,
                        """document.querySelectorAll('[data-testid$="3d-renderer"]').forEach(e=>{
                        e.value='jpeg';e.dispatchEvent(new Event('change',{bubbles:true}));})""",
                    )
                    print(f"{robot} live", flush=True)
                else:
                    until(
                        page,
                        lambda s: (
                            len(s["choices"]) == 3
                            and all(c["disabled"] for c in s["choices"])
                            and "3 / 3" in s["capacity"]
                        ),
                    )
                    results["fullLobby"] = evaluate(page, READ)
                    conflict = evaluate(
                        page,
                        """(async()=>{const r=await fetch('/api/lobby',{
                      method:'POST',headers:{'content-type':'application/json'},
                      body:JSON.stringify({action:'host',token:sessionStorage.getItem('world-ticket')})});
                      return r.status;})()""",
                    )
                    assert conflict == 409
                    results["hostConflictStatus"] = conflict
                    click(page, "observe-world")
                    until(page, lambda s: s["views"] and s["views"][0].get("live") == "true")
                    results["observer"] = evaluate(page, READ)
                    shot = page.command("Page.captureScreenshot", {"format": "png"})
                    (ROOT / "logs/multiplayer-world.png").write_bytes(
                        base64.b64decode(shot["data"])
                    )
                    evaluate(
                        page,
                        """(()=>{const e=document.querySelector(
                        '[data-testid=world3d-renderer]');e.value='jpeg';
                        e.dispatchEvent(new Event('change',{bubbles:true}));})()""",
                    )
                    print("fourth participant can observe; all three slots enforced", flush=True)
            # Software WebGL across four headless contexts can starve the browser's
            # teleop timer. Exercise context-loss fallback after verifying the POVs;
            # retain JPEG world feeds and the live control/transport paths.
            for page in pages:
                evaluate(
                    page,
                    """document.querySelectorAll('canvas').forEach(c =>
                    c.getContext('webgl2')?.getExtension('WEBGL_lose_context')?.loseContext())""",
                )
            first = pages[0]
            before = until(first, lambda s: s["state"] is not None)
            model = http.get(origin + before["state"]["model"], timeout=10).json()
            roots = {a["id"]: model["bodyIds"].index(a["focusBody"]) for a in model["actors"]}
            click(first, "teleop-tele_cmd_vel")
            until(first, lambda s: s["drive"] == "armed")
            evaluate(
                first,
                """(async()=>{const p=document.querySelector(
                '[data-testid=teleop-tele_cmd_vel]');try{
                p.dispatchEvent(new KeyboardEvent('keydown',{key:'w',code:'KeyW',bubbles:true}));
                await new Promise(r=>setTimeout(r,2500));
                }finally{p.dispatchEvent(new KeyboardEvent('keyup',
                {key:'w',code:'KeyW',bubbles:true}));}})()""",
            )
            after = until(
                first,
                lambda s: (
                    s["state"] is not None
                    and abs(
                        s["state"]["poses"][roots["duck2"]][0]
                        - before["state"]["poses"][roots["duck2"]][0]
                    )
                    > 0.06
                ),
            )
            results["movement"] = {
                id: {
                    "before": before["state"]["poses"][root][:3],
                    "after": after["state"]["poses"][root][:3],
                }
                for id, root in roots.items()
            }
            for id in ("duck1", "duck3"):
                delta = results["movement"][id]
                assert abs(delta["before"][0] - delta["after"][0]) < 0.03
            evaluate(first, "document.querySelector('[data-testid=world3d-renderer]').focus()")
            first.command("Page.reload", {})
            until(first, lambda s: s["role"] == "visitor" and s["robot"] == "duck2")
            results["reload"] = until(
                first,
                lambda s: len(s["views"]) == 2 and all(v.get("live") == "true" for v in s["views"]),
            )
            evaluate(
                first,
                """document.querySelectorAll('canvas').forEach(c =>
                c.getContext('webgl2')?.getExtension('WEBGL_lose_context')?.loseContext())""",
            )
            print("isolated movement and same-duck reload passed", flush=True)
            click(pages[1], "leave-world")
            until(
                pages[1],
                lambda s: s["role"] == "observe" and len(s["choices"]) == 3,
            )
            evaluate(
                pages[1],
                """document.querySelectorAll('canvas').forEach(c =>
                c.getContext('webgl2')?.getExtension('WEBGL_lose_context')?.loseContext())""",
            )
            click(pages[3], "leave-world")
            until(
                pages[3],
                lambda s: any(
                    c["id"] == "choose-duck3" and not c["disabled"] for c in s["choices"]
                ),
            )
            click(pages[3], "choose-duck3")
            results["replacement"] = until(
                pages[3],
                lambda s: (
                    s["role"] == "visitor"
                    and s["robot"] == "duck3"
                    and len(s["views"]) == 2
                    and all(v.get("live") == "true" for v in s["views"])
                ),
            )
            print("leaving releases a slot; observer spawns into it", flush=True)
            (ROOT / "logs/multiplayer-checkpoint.json").write_text(json.dumps(results, indent=2))
        finally:
            for page in pages:
                try:
                    evaluate(
                        page,
                        """fetch('/api/lobby',{method:'POST',
                        headers:{'content-type':'application/json'},body:JSON.stringify({
                        action:'observe',token:sessionStorage.getItem('world-ticket')})})""",
                    )
                except Exception:
                    pass
                page.socket.close()
            if process is not None:
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
