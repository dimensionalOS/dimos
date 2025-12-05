import asyncio
import os
import ssl
import logging
import re
import subprocess
import time
from datetime import datetime
from typing import Callable, Optional
import threading
import json

from aiohttp import web, ClientSession, WSMsgType
from yarl import URL

# Optional .env support
try:
    from dotenv import load_dotenv
    load_dotenv()
except Exception:
    pass

session_name = "dimos-dashboard-6"
def html_code_gen(rrd_url, zellij_token: Optional[str], terminals) -> str:
    # TODO: vet this function for html-injections. Ex: each_value should have some kind of .replace("</script>", "<\\/script>") or something like that
    iframe_html = ""
    iframe_js = r"""
        // TODO: this function is more complicated than it needs to be, there was just an edgecase with space that made it get more complicated

        /**
        * Dispatch a sequence of keyboard (and optional input) events for a string.
        *
        * @param {Element} target - DOM element to dispatch events on (e.g. canvas, input, div).
        * @param {string} text - Text to convert into events.
        * @param {Object} [options]
        * @param {string[]} [options.eventTypes=["keydown", "keypress", "keyup"]] - Keyboard event types to fire.
        * @param {boolean} [options.includeInputEvent=false] - Also dispatch an "input" event per character.
        * @param {boolean} [options.bubbles=true] - Whether events bubble.
        * @param {boolean} [options.cancelable=true] - Whether events are cancelable.
        */
        function dispatchStringAsKeyEvents(target, text, options = {}) {
            if (!target) {
                throw new Error("dispatchStringAsKeyEvents: target element is required");
            }

            function charToKeyInfoOld(char) {
                // Handle some control characters specially
                if (char === "\n") {
                    return { key: "Enter", code: "Enter", keyCode: 13, which: 13 };
                }
                if (char === "\t") {
                    return { key: "Tab", code: "Tab", keyCode: 9, which: 9 };
                }

                const key = char;
                const isSingleChar = char.length === 1;
                const upper = isSingleChar ? char.toUpperCase() : "";
                const isLetter = /^[a-z]$/i.test(char);
                const code = isLetter ? `Key${upper}` : undefined;

                const keyCode = isSingleChar ? char.charCodeAt(0) : 0;

                return {
                    key,
                    code,
                    keyCode,
                    which: keyCode,
                };
            }

            // Works but duplicates non-space keys
            function sendKey(type, key, code, keyCode, target) {
                const evt = new KeyboardEvent(type, {
                    key,
                    code,
                    bubbles: true,
                    cancelable: true,
                });

                // Patch keyCode / which if possible
                try {
                    Object.defineProperty(evt, "keyCode", { get: () => keyCode });
                    Object.defineProperty(evt, "which", { get: () => keyCode });
                } catch (_) {}

                target.dispatchEvent(evt);
            }

            const {
                eventTypes = ["keydown", "keypress", "keyup"],
                includeInputEvent = false,
                bubbles = true,
                cancelable = true,
                useLegacyKeyboardEvent = false, // try old Firefox/WebKit path
            } = options;

            function charToKeyInfo(char) {
                // Control chars
                if (char === "\n") {
                    return { key: "Enter", code: "Enter", keyCode: 13, which: 13, charCode: 13 };
                }
                if (char === "\t") {
                    return { key: "Tab", code: "Tab", keyCode: 9, which: 9, charCode: 9 };
                }
                if (char === " ") {
                    // ✅ spacebar special case
                    return {
                        key: " ",
                        code: "Space",
                        keyCode: 32,
                        which: 32,
                        charCode: 32,
                    };
                }

                const isSingleChar = char.length === 1;
                const upper = isSingleChar ? char.toUpperCase() : "";
                const isLetter = /^[a-z]$/i.test(char);
                const code = isLetter ? `Key${upper}` : undefined;
                const keyCode = isSingleChar ? char.charCodeAt(0) : 0;

                return {
                    key: char,
                    code,
                    keyCode,
                    which: keyCode,
                    charCode: keyCode,
                };
            }

            function createKeyboardEventModern(type, info) {
                const evt = new KeyboardEvent(type, {
                    key: info.key,
                    code: info.code,
                    bubbles,
                    cancelable,
                });

                // Patch keyCode / which / charCode if the browser lets us
                const patch = (prop, value) => {
                    try {
                        Object.defineProperty(evt, prop, {
                            get() {
                                return value;
                            },
                        });
                    } catch (e) {
                        // Some browsers make these read-only; ignore
                    }
                };

                patch("keyCode", info.keyCode);
                patch("which", info.which);
                if (type === "keypress") {
                    patch("charCode", info.charCode);
                }

                return evt;
            }

            function createKeyboardEventLegacy(type, info) {
                // ⚠️ Deprecated / non-portable, but sometimes behaves differently
                const evt = document.createEvent("KeyboardEvent");

                const keyCode = info.keyCode || 0;
                const charCode = type === "keypress" ? info.charCode || keyCode : 0;

                // Different browsers support different init methods;
                // this is a best-effort hack.
                if (evt.initKeyEvent) {
                    evt.initKeyEvent(
                        type,          // type
                        bubbles,       // canBubble
                        cancelable,    // cancelable
                        window,        // view
                        false, false, false, false, // ctrl, alt, shift, meta
                        keyCode,
                        charCode
                    );
                } else if (evt.initKeyboardEvent) {
                    evt.initKeyboardEvent(
                        type,
                        bubbles,
                        cancelable,
                        window,
                        info.key,
                        0,
                        "",
                        false,
                        ""
                    );
                }

                return evt;
            }

            function createKeyboardEvent(type, info) {
                if (useLegacyKeyboardEvent) {
                    return createKeyboardEventLegacy(type, info);
                }
                return createKeyboardEventModern(type, info);
            }
            
            for (const char of text) {
                // non spaces
                if (char !== " ") {
                    const keyInfo = charToKeyInfoOld(char);

                    // Fire each keyboard event type
                    for (const type of eventTypes) {
                        const evt = new KeyboardEvent(type, {
                            key: keyInfo.key,
                            code: keyInfo.code,
                            keyCode: keyInfo.keyCode,
                            which: keyInfo.which,
                            bubbles,
                            cancelable,
                        });

                        target.dispatchEvent(evt);
                    }

                    // Optionally fire an "input" event (useful if target listens for text input)
                    if (includeInputEvent) {
                        const inputEvt = new InputEvent("input", {
                            data: char,
                            bubbles,
                            cancelable,
                        });
                        target.dispatchEvent(inputEvt);
                    }
                // spaces
                } else {
                    const info = charToKeyInfo(char);

                    for (const type of eventTypes) {
                        const evt = createKeyboardEvent(type, info);
                        target.dispatchEvent(evt);
                    }

                    if (includeInputEvent) {
                        const inputEvt = new InputEvent("input", {
                            data: char,
                            bubbles,
                            cancelable,
                        });
                        target.dispatchEvent(inputEvt);
                    }
                }
            }
        }
    """
    # for each_key, each_value in terminals.items():
    #     iframe_html += f"""
    #     <iframe id="iframe-{each_key}" src="/{each_key}" frameborder="0" onload="this.style.opacity = '1'"> </iframe>
    #     """
    #     iframe_js += f"""
    #         var iframe = document.getElementById("iframe-{each_key}")
    #         if (iframe) {{
    #             console.log("iframe-{each_key} loaded", iframe)
    #             iframe.contentWindow?.term?._core?.paste({json.dumps(each_value+"\n")})
    #         }}
    #     """
    each_key = session_name
    iframe_html += f"""
        <iframe id="iframe-{each_key}" src="/{each_key}" frameborder="0" onload="this.style.opacity = '1'"> </iframe>
    """
    # iframe_js += f"""
    #     var iframe = document.getElementById("iframe-{each_key}")
    #     if (iframe) {{
    #         console.log("iframe-{each_key} loaded", iframe)
    #         iframe.contentWindow?.term?._core?.paste({json.dumps(each_value+"\n")})
    #     }}
    # """
    token_json = json.dumps(str(zellij_token)) if zellij_token else "null"
    return """
<!DOCTYPE html>
<html lang="en">
    <head>
        <meta charset="UTF-8" />
        <meta name="viewport" content="width=device-width, initial-scale=1.0" />
        <title>DimOS Viewer</title>
         <style>
            @-ms-viewport {
                width: device-width;
            }
            article, aside, details, figcaption, figure, footer, header, hgroup, menu, nav, section, main, summary {
                display: block;
            }

            *, *::before, *::after {
                box-sizing: inherit;
            }

            html {
                /* 1 */
                box-sizing: border-box;
                /* 2 */
                touch-action: manipulation;
                /* 3 */
                -webkit-text-size-adjust: 100%;
                -ms-text-size-adjust: 100%;
                /* 4 */
                -ms-overflow-style: scrollbar;
                /* 5 */
                -webkit-tap-highlight-color: rgba(0, 0, 0, 0);
            }

            body {
                line-height: 1;
            }

            html, body, div, span, applet, object, iframe, h1, h2, h3, h4, h5, h6, p, blockquote, pre, a, abbr, acronym, address, big, cite, code, del, dfn, em, img, ins, kbd, q, s, samp, small, strike, strong, sub, sup, tt, var, b, u, i, center, dl, dt, dd, ol, ul, li, fieldset, form, label, legend, table, caption, tbody, tfoot, thead, tr, th, td, article, aside, canvas, details, embed, figure, figcaption, footer, header, hgroup, menu, nav, output, ruby, section, summary, time, mark, audio, video, main {
                font-size: 100%;
                font: inherit;
                vertical-align: baseline;
            }

            ol, ul {
                list-style: none;
            }

            blockquote, q {
                quotes: none;
            }

            blockquote::before, blockquote::after, q::before, q::after {
                content: "";
                content: none;
            }

            table {
                border-collapse: collapse;
                border-spacing: 0;
            }

            hr {
                /* 1 */
                box-sizing: content-box;
                height: 0;
                /* 2 */
                overflow: visible;
            }

            pre, code, kbd, samp {
                /* 1 */
                font-family: monospace, monospace;
            }

            pre {
                /* 2 */
                overflow: auto;
                /* 3 */
                -ms-overflow-style: scrollbar;
            }

            a {
                /* 1 */
                background-color: transparent;
                /* 2 */
                -webkit-text-decoration-skip: objects;
            }

            abbr[title] {
                /* 1 */
                border-bottom: none;
                /* 2 */
                text-decoration: underline;
                text-decoration: underline dotted;
            }

            b, strong {
                font-weight: bolder;
            }

            small {
                font-size: 80%;
            }

            sub, sup {
                font-size: 75%;
                line-height: 0;
                position: relative;
            }

            sub {
                bottom: -0.25em;
            }

            sup {
                top: -0.5em;
            }

            img {
                border-style: none;
            }

            svg:not(:root) {
                overflow: hidden;
            }

            button {
                border-radius: 0;
            }

            input, button, select, optgroup, textarea {
                font-family: inherit;
                font-size: inherit;
                line-height: inherit;
            }

            button, [type=reset], [type=submit], html [type=button] {
                -webkit-appearance: button;
            }

            input[type=date], input[type=time], input[type=datetime-local], input[type=month] {
                -webkit-appearance: listbox;
            }

            fieldset {
                min-width: 0;
            }

            [tabindex="-1"]:focus {
                outline: 0 !important;
            }

            button, input {
                overflow: visible;
            }

            button, select {
                text-transform: none;
            }

            button::-moz-focus-inner, [type=button]::-moz-focus-inner, [type=reset]::-moz-focus-inner, [type=submit]::-moz-focus-inner {
                border-style: none;
                padding: 0;
            }

            legend {
                /* 1 */
                max-width: 100%;
                white-space: normal;
                /* 2 */
                color: inherit;
                /* 3 */
                display: block;
            }

            progress {
                vertical-align: baseline;
            }

            textarea {
                overflow: auto;
            }

            [type=checkbox], [type=radio] {
                /* 1 */
                box-sizing: border-box;
                /* 2 */
                padding: 0;
            }

            [type=number]::-webkit-inner-spin-button, [type=number]::-webkit-outer-spin-button {
                height: auto;
            }

            [type=search] {
                /* 1 */
                -webkit-appearance: textfield;
                /* 2 */
                outline-offset: -2px;
            }

            [type=search]::-webkit-search-cancel-button, [type=search]::-webkit-search-decoration {
                -webkit-appearance: none;
            }

            ::-webkit-file-upload-button {
                /* 1 */
                -webkit-appearance: button;
                /* 2 */
                font: inherit;
            }

            template {
                display: none;
            }

            [hidden] {
                display: none;
            }

            button:focus {
                outline: 1px dotted;
                outline: 5px auto -webkit-focus-ring-color;
            }

            button:-moz-focusring, [type=button]:-moz-focusring, [type=reset]:-moz-focusring, [type=submit]:-moz-focusring {
                outline: 1px dotted ButtonText;
            }

            html, body, div, span, applet, object, iframe, h1, h2, h3, h4, h5, h6, p, blockquote, pre, a, abbr, acronym, address, big, cite, code, del, dfn, em, img, ins, kbd, q, s, samp, small, strike, strong, sub, sup, tt, var, b, u, i, center, dl, dt, dd, ol, ul, li, fieldset, form, label, legend, table, caption, tbody, tfoot, thead, tr, th, td, article, aside, canvas, details, embed, figure, figcaption, footer, header, hgroup, menu, nav, output, ruby, section, summary, time, mark, audio, video, main {
                margin: 0;
                padding: 0;
                border: 0;
            }

            input, button, select, optgroup, textarea {
                margin: 0;
            }

            body {
                width: 100vw;
                min-height: 100vh;
                overflow: visible;
                scroll-behavior: auto;
            }

            textarea {
                resize: vertical;
            }

            br {
                display: block;
                content: "";
                border-bottom: 0px solid transparent;
            }
         </style>
         <style>
            body {
                --terminal-panel-width: 30vw;
            }
            #terminal-side {
                width: var(--terminal-panel-width);
                min-width: 20rem;
                display: flex;
                height: 100vh;
                min-height: 50rem;
                overflow: auto;
                flex-direction: column;
            }
            iframe {
                width: 100%;
                height: 100%;
                border: none;
                margin: 0;  
                zoom: 0.8;
            }
            body canvas {
                width: calc(100vw - var(--terminal-panel-width)) !important;
                height: 100vh !important;
            }
            .terminal-side {
                display: grid;
                width: 30vw;                /* total width */
                grid-template-columns: repeat(2, 1fr);  /* at most 2 items per row */
                grid-auto-rows: 50vh;       /* every row is 50vh tall */
                gap: 0.5rem;                /* optional spacing */
            }

            /* Basic item styling so you can see them */
            .terminal-side > div {
                border: 1px solid #ccc;
                box-sizing: border-box;
            }

            /* If there is only one item total, make it full width */
            .terminal-side > :only-child {
                grid-column: 1 / -1;
            }

            /* If the last item is alone in its row (odd count), make it span both columns */
            .terminal-side > :nth-child(odd):last-child {
                grid-column: 1 / -1;
            }

         </style>
    </head>
    <body style="display: flex; justify-content: center; flex-direction: row; background-color: #0d1011;">
        <div id="terminal-side">
            """+iframe_html+"""
        </div>
    </body>
    <script type="module">
        // 
        // rerun
        // 
        import { WebViewer } from "https://esm.sh/@rerun-io/web-viewer@0.27.2";
        const rrdUrl = """+json.dumps(str(rrd_url))+""";
        const zellijToken = """+token_json+""";
        const parentElement = document.body;
        const viewer = new WebViewer();
        await viewer.start(rrdUrl, parentElement);
        
        // 
        // zellij
        // 
        const iframes = document.querySelectorAll("iframe")
        await new Promise((r) => setTimeout(r, 200))
        for (let each of iframes) {
            let input
            if ((input = each.contentDocument.body?.querySelector("#remember"))) {
                input.checked = true
            }
            if ((input = each.contentDocument.body?.querySelector("#token"))) {
                if (zellijToken) {
                    input.value = zellijToken
                    if ((input = each.contentDocument.body?.querySelector("#submit"))) {
                        input.click()
                    }
                }
            }
            if (input) {
                await new Promise(r=>setTimeout(r,300))
            }
            each?.contentWindow?.term?._core?.paste("\\n")
            // to get past the startup "press enter"
            // dispatchStringAsKeyEvents(each,"\\n")
        }
        
        """+iframe_js+"""
    </script>
</html>
"""


def env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def normalize_path_prefix(prefix: str) -> str:
    if not prefix.startswith("/"):
        prefix = "/" + prefix
    return prefix.rstrip("/") or "/"

def path_matches(prefix: str, path: str) -> bool:
    return path == prefix or path.startswith(prefix + "/")

def build_target_url(
    request: web.Request,
    target_base: str,
    strip_prefix: Optional[str] = None,
    add_prefix: Optional[str] = None,
) -> URL:
    target = URL(target_base)
    path = request.rel_url.path

    if strip_prefix and path_matches(strip_prefix, path):
        path = path[len(strip_prefix) :] or "/"
        if not path.startswith("/"):
            path = "/" + path

    if add_prefix:
        add_prefix = add_prefix.rstrip("/")
        path = f"{add_prefix}{path}"

    full_path = target.path.rstrip("/") + path
    return target.with_path(full_path or "/").with_query(request.rel_url.query)


SESSION_LINE_RE = re.compile(r"^(.+?)\s+\[Created\s+(.+?)\s+ago\](.*)$")
def parse_zellij_sessions(output: str):
    sessions = []
    for line in output.strip().splitlines():
        line = line.strip()
        if not line:
            continue
        match = SESSION_LINE_RE.match(line)
        if match:
            session_name = match.group(1).strip()
            created_ago = match.group(2).strip()
            additional = match.group(3).strip()
            sessions.append(
                {
                    "name": session_name,
                    "createdAgo": created_ago,
                    "status": additional or "active",
                    "raw": line,
                }
            )
    return sessions

HOP_BY_HOP_HEADERS = {
    "connection",
    "keep-alive",
    "proxy-authenticate",
    "proxy-authorization",
    "te",
    "trailers",
    "transfer-encoding",
    "upgrade",
}

def run_proxy_server(config: dict, log: logging.Logger):
    port = config["port"]
    proxy_host = config["proxy_host"]
    zellij_host = config["zellij_host"]
    zellij_port = config["zellij_port"]
    backend_host = config["backend_host"]
    backend_port = config["backend_port"]
    frontend_host = config["frontend_host"]
    frontend_port = config["frontend_port"]
    frontend_base_path = config["frontend_base_path"]
    api_base_path = config["api_base_path"]
    https_enabled = config["https_enabled"]
    https_key_path = config["https_key_path"]
    https_cert_path = config["https_cert_path"]
    protocol = config["protocol"]
    zellij_target = config["zellij_target"]
    backend_target = config["backend_target"]
    frontend_target = config["frontend_target"]
    rrd_url = config["rrd_url"]
    terminals = config["terminals"]
    zellij_token_holder = {"token": config.get("zellij_token")}

    async def proxy_http(
        request: web.Request,
        target_base: str,
        strip_prefix: Optional[str] = None,
        add_prefix: Optional[str] = None,
    ) -> web.StreamResponse:
        session: ClientSession = request.app["client"]
        target_url = build_target_url(request, target_base, strip_prefix, add_prefix)

        try:
            data = await request.read()
            headers = {
                k: v for k, v in request.headers.items() if k.lower() not in HOP_BY_HOP_HEADERS
            }

            async with session.request(
                request.method,
                target_url,
                headers=headers,
                data=data if data else None,
                allow_redirects=False,
            ) as resp:
                resp_headers = {
                    k: v
                    for k, v in resp.headers.items()
                    if k.lower() not in HOP_BY_HOP_HEADERS
                }
                body = await resp.read()
                return web.Response(status=resp.status, headers=resp_headers, body=body)
        except Exception as exc:  # pragma: no cover - network errors
            log.error("Proxy error to %s: %s", target_url, exc)
            return web.Response(status=502, text="Upstream unavailable")

    async def proxy_websocket(
        request: web.Request,
        target_base: str,
        strip_prefix: Optional[str] = None,
        add_prefix: Optional[str] = None,
    ) -> web.StreamResponse:
        session: ClientSession = request.app["client"]
        target_url = build_target_url(request, target_base, strip_prefix, add_prefix)
        target_url = target_url.with_scheme("wss" if target_url.scheme == "https" else "ws")

        ws_server = web.WebSocketResponse()
        await ws_server.prepare(request)

        headers = {
            k: v for k, v in request.headers.items() if k.lower() not in HOP_BY_HOP_HEADERS
        }

        try:
            async with session.ws_connect(target_url, headers=headers) as ws_client:
                async def relay(ws_from, ws_to):
                    async for msg in ws_from:
                        if msg.type == WSMsgType.TEXT:
                            await ws_to.send_str(msg.data)
                        elif msg.type == WSMsgType.BINARY:
                            await ws_to.send_bytes(msg.data)
                        elif msg.type == WSMsgType.CLOSE:
                            await ws_to.close()
                            break
                        elif msg.type == WSMsgType.ERROR:
                            break

                tasks = [
                    asyncio.create_task(relay(ws_server, ws_client)),
                    asyncio.create_task(relay(ws_client, ws_server)),
                ]
                done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                for task in pending:
                    task.cancel()
                await asyncio.gather(*pending, return_exceptions=True)
        except Exception as exc:  # pragma: no cover - network errors
            log.error("WebSocket proxy error to %s: %s", target_url, exc)
        finally:
            await ws_server.close()

        return ws_server

    async def is_zellij_running(session: ClientSession) -> bool:
        try:
            async with session.get(f"{zellij_target}/", timeout=2) as resp:
                return resp.status < 500
        except Exception:
            return False

    async def start_zellij_process() -> Optional[asyncio.subprocess.Process]:
        cmd = ["zellij", "web", "--port", str(zellij_port), ]
        log.info("Zellij not detected, starting: %s", " ".join(cmd))

        token_re = re.compile(
            r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}", re.IGNORECASE
        )

        async def capture_token():
            if zellij_token_holder["token"]:
                return
            proc = await asyncio.create_subprocess_exec(
                *["zellij", "web", "--create-token"],
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            streams = [proc.stdout, proc.stderr]
            for stream in streams:
                if stream is None:
                    continue
                try:
                    while True:
                        line = await asyncio.wait_for(stream.readline(), timeout=3)
                        if not line:
                            break
                        text = line.decode(errors="ignore")
                        match = token_re.search(text)
                        if match:
                            zellij_token_holder["token"] = match.group(0)
                            log.info("Discovered zellij web token")
                            return
                except asyncio.TimeoutError:
                    continue

        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            await capture_token()
            return proc
        except FileNotFoundError:
            log.error("zellij executable not found; please install zellij.")
        except Exception as exc:  # pragma: no cover - runtime failure
            log.error("Failed to start zellij web: %s", exc)
        return None

    async def run_zellij_list_sessions() -> dict:
        proc = await asyncio.create_subprocess_shell(
            "zellij list-sessions --no-formatting",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        stdout, stderr = await proc.communicate()
        stdout_text = stdout.decode()
        stderr_text = stderr.decode()

        if stderr_text:
            log.warning("zellij stderr: %s", stderr_text.strip())

        if proc.returncode != 0:
            if proc.returncode == 1 and not stdout_text.strip():
                return {
                    "success": True,
                    "sessions": [],
                    "count": 0,
                    "message": "No active zellij sessions found",
                }
            raise RuntimeError(
                f"zellij list-sessions failed (code {proc.returncode}): {stderr_text or stdout_text}"
            )

        sessions = parse_zellij_sessions(stdout_text)
        log.info("Found %s sessions", len(sessions))
        return {"success": True, "sessions": sessions, "count": len(sessions)}

    def add_cors_headers(resp: web.StreamResponse) -> web.StreamResponse:
        resp.headers["Access-Control-Allow-Origin"] = "*"
        resp.headers["Access-Control-Allow-Methods"] = "GET,OPTIONS"
        resp.headers["Access-Control-Allow-Headers"] = "*"
        return resp

    async def handle_api(request: web.Request, subpath: str) -> web.StreamResponse:
        if request.method == "OPTIONS":
            return add_cors_headers(web.Response(status=204))

        if subpath.startswith("/"):
            subpath = subpath[1:]

        if subpath in ("health", "health/"):
            data = {
                "status": "ok",
                "timestamp": datetime.utcnow().isoformat() + "Z",
            }
            return add_cors_headers(web.json_response(data))

        if subpath in ("sessions", "sessions/"):
            try:
                data = await run_zellij_list_sessions()
                return add_cors_headers(web.json_response(data))
            except Exception as exc:
                log.error("Error fetching zellij sessions: %s", exc)
                data = {"success": False, "error": str(exc)}
                return add_cors_headers(web.json_response(data, status=500))

        return add_cors_headers(web.json_response({"error": "Not found"}, status=404))

    async def dispatch(request: web.Request) -> web.StreamResponse:
        path = request.rel_url.path
        is_ws = request.headers.get("upgrade", "").lower() == "websocket"

        if path in ("/", "", "/zviewer", "/zviewer/"):
            return web.Response(text=html_code_gen(rrd_url, zellij_token_holder["token"], terminals=terminals), content_type="text/html")

        if path == "/health":
            return web.json_response(
                {
                    "status": "ok",
                    "timestamp": datetime.utcnow().isoformat() + "Z",
                    "services": {
                        "zellij": zellij_target,
                        "frontend": f"{frontend_target}{frontend_base_path}",
                        "backend": f"{protocol}://{proxy_host}:{port}{api_base_path}",
                    },
                }
            )

        if path_matches(api_base_path, path):
            if is_ws:
                return web.Response(status=400, text="WebSocket not supported on API")
            subpath = path[len(api_base_path) :]
            return await handle_api(request, subpath)

        if path_matches(frontend_base_path, path):
            proxy_fn = proxy_websocket if is_ws else proxy_http
            return await proxy_fn(request, frontend_target)

        proxy_fn = proxy_websocket if is_ws else proxy_http
        return await proxy_fn(request, zellij_target)

    async def on_startup(app: web.Application):
        app["client"] = ClientSession()
        session: ClientSession = app["client"]

        if not await is_zellij_running(session):
            app["zellij_process"] = await start_zellij_process()
            for _ in range(10):
                if await is_zellij_running(session):
                    break
                await asyncio.sleep(0.5)
            else:
                log.warning("Zellij web did not become ready after startup attempt.")
        else:
            app["zellij_process"] = None
            if not zellij_token_holder["token"]:
                log.warning("Zellij token not available; inline auto-login will be disabled.")

        log.info("🚀 Starting Zellij Session Viewer Reverse Proxy (Python)")
        log.info("🎯 Reverse Proxy Server running on %s://%s:%s", protocol, proxy_host, port)
        log.info("📋 Service Routes:")
        log.info("   🖥️  Zellij Web Client:     %s://%s:%s/", protocol, proxy_host, port)
        log.info(
            "   📱 Session Manager UI:    %s://%s:%s%s/",
            protocol,
            proxy_host,
            port,
            frontend_base_path,
        )
        log.info(
            "   🔌 Backend API:           %s://%s:%s%s/",
            protocol,
            proxy_host,
            port,
            api_base_path,
        )
        log.info("   ❤️  Health Check:         %s://%s:%s/health", protocol, proxy_host, port)
        log.info("🚀 Ready to tunnel port %s!", port)

    async def on_cleanup(app: web.Application):
        client: ClientSession = app["client"]
        await client.close()
        proc: Optional[asyncio.subprocess.Process] = app.get("zellij_process")
        if proc and proc.returncode is None:
            proc.terminate()
            try:
                await asyncio.wait_for(proc.wait(), timeout=2)
            except asyncio.TimeoutError:
                proc.kill()

    def create_app() -> web.Application:
        app = web.Application()
        app.router.add_route("*", "/{path:.*}", dispatch)
        app.on_startup.append(on_startup)
        app.on_cleanup.append(on_cleanup)
        return app

    def build_ssl_context() -> Optional[ssl.SSLContext]:
        if not https_enabled:
            return None

        if not https_key_path or not https_cert_path:
            raise RuntimeError("HTTPS enabled but HTTPS_KEY_PATH or HTTPS_CERT_PATH not set")

        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(certfile=https_cert_path, keyfile=https_key_path)
        return context

    ssl_context = build_ssl_context()
    app = create_app()
    try:
        web.run_app(
            app,
            host=proxy_host,
            port=port,
            ssl_context=ssl_context,
            access_log=None,
            handle_signals=False,
        )
    except Exception as exc:  # pragma: no cover - runtime errors
        log.error("Failed to start proxy: %s", exc)
        raise

def dimos_dashboard_func(
    *,
    port: int = int(os.environ.get("PROXY_PORT", "4000")),
    proxy_host: str = os.environ.get("PROXY_HOST", "localhost"),
    zellij_host: str = os.environ.get("ZELLIJ_HOST", "127.0.0.1"),
    zellij_port: int = int(os.environ.get("ZELLIJ_PORT", "8083")),
    backend_host: str = os.environ.get("BACKEND_HOST", "localhost"),
    backend_port: int = int(os.environ.get("BACKEND_PORT", "3001")),
    frontend_host: str = os.environ.get("FRONTEND_HOST", "localhost"),
    frontend_port: int = int(os.environ.get("FRONTEND_PORT", "5173")),
    frontend_base_path: str = normalize_path_prefix(os.environ.get("FRONTEND_BASE_PATH", "/zviewer")),
    api_base_path: str = normalize_path_prefix(os.environ.get("API_BASE_PATH", "/zviewer/api")),
    https_enabled: bool = env_bool("HTTPS_ENABLED", False),
    https_key_path: Optional[str] = os.environ.get("HTTPS_KEY_PATH"),
    https_cert_path: Optional[str] = os.environ.get("HTTPS_CERT_PATH"),
    zellij_target: Optional[str] = None,
    backend_target: Optional[str] = None,
    frontend_target: Optional[str] = None,
    terminal_commands: Optional[dict[str, str]] = None,
    zellij_token: Optional[str] = os.environ.get("ZELLIJ_TOKEN"),
    zellij_namespace: Optional[str] = "dimos",
    logger: Optional[logging.Logger] = None,
    rrd_url: Optional[str] = None,
) -> threading.Thread:
    if not logger:
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(message)s",
        )
        log = logging.getLogger("proxy")
    else:
        log = logger

    normalized_frontend_base_path = normalize_path_prefix(frontend_base_path)
    normalized_api_base_path = normalize_path_prefix(api_base_path)
    protocol = "https" if https_enabled else "http"

    effective_zellij_target = zellij_target or f"{protocol}://{zellij_host}:{zellij_port}"
    effective_backend_target = backend_target or f"http://{backend_host}:{backend_port}"
    effective_frontend_target = frontend_target or f"http://{frontend_host}:{frontend_port}"
    terminals = { f"{zellij_namespace}-{key}": value for key, value in (terminal_commands or {}).items() }

    config = {
        "port": port,
        "proxy_host": proxy_host,
        "zellij_host": zellij_host,
        "zellij_port": zellij_port,
        "backend_host": backend_host,
        "backend_port": backend_port,
        "frontend_host": frontend_host,
        "frontend_port": frontend_port,
        "frontend_base_path": normalized_frontend_base_path,
        "api_base_path": normalized_api_base_path,
        "https_enabled": https_enabled,
        "https_key_path": https_key_path,
        "https_cert_path": https_cert_path,
        "protocol": protocol,
        "zellij_target": effective_zellij_target,
        "backend_target": effective_backend_target,
        "frontend_target": effective_frontend_target,
        "rrd_url": rrd_url,
        "zellij_token": zellij_token,
        "terminals": terminals,
    }
    
    def launch_zellij_in_background(terminals_map: dict[str, str]) -> None:
        try:
            kill_proc = subprocess.Popen(
                ["zellij", "kill-session", session_name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            try:
                kill_proc.communicate(timeout=2)
            except subprocess.TimeoutExpired:
                kill_proc.kill()
        except FileNotFoundError:
            log.error("zellij executable not found; cannot manage session %s", session_name)
            return
        except Exception as exc:
            log.warning("Unable to kill session %s: %s", session_name, exc)

        try:
            files_to_run = []
            for command in terminals_map.values():
                sanitized_command = re.sub(r"[^A-Za-z\s_\-\=\*]", "", command)
                file_path = f"/tmp/{sanitized_command}.sh"
                with open(file_path, 'w') as the_file:
                    the_file.write(f"""
                        source ./venv/bin/activate
                        {command}
                    """)
                files_to_run.append(file_path)
            zellij_path = "/tmp/.zellij_layout.kdl"
            with open(zellij_path, 'w') as the_file:
                the_file.write("""
                    layout {
                        """+"\n".join(
                            f"""pane command=\"zsh\" {{
                                args "{file_path}"
                            }}""" for file_path in files_to_run
                        )+"""
                    }
                """)
            
            subprocess.Popen(
                # zellij attach --create-background my-session-name options --default-layout
                # ["zellij", "attach", "--create-background", session_name, "options", "--web-sharing=on",],
                ["zellij", "attach", "--create-background", session_name, "options", "--web-sharing=on", "--default-layout", zellij_path],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
            )
            print(f'''started: {session_name} with panes {files_to_run}''')
        except Exception as exc:
            log.error("Failed to start zellij session %s: %s", session_name, exc)

    threading.Thread(
        target=launch_zellij_in_background,
        args=(terminals,),
        daemon=True,
        name="zellij-launcher",
    ).start()
        
    thread = threading.Thread(
        target=run_proxy_server,
        args=(config, log),
        daemon=True,
        name="proxy-server",
    )
    thread.start()
    return thread


if __name__ == "__main__":
    t = dimos_dashboard_func(terminal_commands={
        "agent-spy": "dimos agentspy",
        "lcm-spy": "dimos lcmspy",
        # "skill-spy": "dimos skillspy",
    })
    try:
        while t.is_alive():
            t.join(timeout=0.5)
    except KeyboardInterrupt:
        print("Received interrupt; shutting down.")
