from functools import lru_cache
import json
import os

DEFAULT_SESSION_NAME = "dimos-dashboard"

path_to_baseline_css = os.path.join(os.path.dirname(__file__), "css_baseline.css")
with open(filepath,'r') as f:
    css_baseline_contents = f.read()

# prevent any kind of html injection (even accidental)
escape_js_for_html = lambda text: text.replace("</script>", "<\\/script>")
escape_js_value = lambda text: escape_js_for_html(json.dumps(text))
escape_css_for_html = lambda text: text.replace("</style>", "\\003C/style>")
session_name_regex = re.compile(r"^[A-Za-z0-9_-]+$")
def ensure_session_name_valid(value: str) -> str:
    """
    Note: this function is enforcing two restrictions:
        - the value must be valid if embedded html attribute (no double quotes)
        - the value must be valid as a zellij session name
    """
    if not isinstance(value, str):
        raise TypeError(f"Expected str, got {type(value).__name__}")
    if not session_name_regex.match(value):
        raise ValueError("session name may only contain letters, numbers, underscores, or dashes. Got "+value)
    if len(value) < 2:
        raise ValueError("session name must be at least 2 characters long. Got: "+value)
    
    return value

@lru_cache(maxsize=2)
def html_code_gen(rrd_url: str, zellij_token: Optional[str] = None, session_name: str = DEFAULT_SESSION_NAME) -> str:
    # TODO: download "https://esm.sh/@rerun-io/web-viewer@0.27.2" so that this works offline
    return """
<!DOCTYPE html>
<html lang="en">
    <head>
        <meta charset="UTF-8" />
        <meta name="viewport" content="width=device-width, initial-scale=1.0" />
        <title>DimOS Viewer</title>
        <style>"""+escape_css_for_html(css_baseline_contents)+"""</style>
    </head>
    <body style="display: flex; justify-content: center; flex-direction: row; background-color: #0d1011;">
        <div id="terminal-side">
            <iframe data-is-zellij="true" id="iframe-"""+ensure_session_name_valid(session_name)+"""}" src="/"""+ensure_session_name_valid(session_name)+"""}" frameborder="0" onload="this.style.opacity = '1'"> </iframe>
        </div>
    </body>
    <script type="module">
        // 
        // rerun
        // 
        import { WebViewer } from "https://esm.sh/@rerun-io/web-viewer@0.27.2";
        const rrdUrl = """+escape_js_value(rrd_url)+""";
        const parentElement = document.body;
        const viewer = new WebViewer();
        await viewer.start(rrdUrl, parentElement);
        
        // 
        // zellij
        // 
        const zellijToken = """+escape_js_value(zellij_token)+""";
        const iframes = document.querySelectorAll("iframe[data-is-zellij="true"]")
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
            // to get past the startup "press enter"
            sendEnterKeyTo(each)
        }
        function sendEnterKeyTo(target) {
            const eventTypes = ["keydown", "keypress", "keyup"]
            const keyInfo = { key: "Enter", code: "Enter", keyCode: 13, which: 13 }
            for (const type of eventTypes) {
                const evt = new KeyboardEvent(type, {
                    key: keyInfo.key,
                    code: keyInfo.code,
                    keyCode: keyInfo.keyCode,
                    which: keyInfo.which,
                    bubbles,
                    cancelable,
                })

                target.dispatchEvent(evt)
            }
        }
    </script>
</html>
"""