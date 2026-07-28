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

"""Small, dependency-free operator UI for a hosted replay repository."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from html import escape


@dataclass(frozen=True)
class RepositoryObjectView:
    """Display-safe input for one repository object."""

    filename: str
    size_bytes: int
    sha256: str
    content_type: str
    object_url: str


_STYLE = """
:root {
  color-scheme: light;
  --bg: #f4f5f7;
  --panel: #ffffff;
  --text: #171a1f;
  --muted: #667085;
  --line: #d8dde5;
  --accent: #2457d6;
  --ok: #177245;
  --error: #b42318;
}
* { box-sizing: border-box; }
body {
  margin: 0;
  background: var(--bg);
  color: var(--text);
  font: 14px/1.5 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
}
a { color: var(--accent); }
code, .mono { font-family: ui-monospace, SFMono-Regular, Consolas, monospace; }
.shell { width: min(1080px, calc(100% - 32px)); margin: 0 auto; }
.topbar {
  background: #171a1f;
  border-bottom: 1px solid #2d333d;
  color: #fff;
}
.topbar .shell {
  min-height: 54px;
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 16px;
}
.brand { font-weight: 650; letter-spacing: .01em; }
.node { color: #b9c0cc; font-size: 13px; }
main { padding: 24px 0 48px; }
.summary {
  display: flex;
  align-items: flex-start;
  justify-content: space-between;
  gap: 20px;
  margin-bottom: 18px;
}
h1 { margin: 0; font-size: 22px; line-height: 1.3; }
h2 { margin: 0 0 14px; font-size: 16px; }
p { margin: 6px 0 0; color: var(--muted); }
.online { color: var(--ok); white-space: nowrap; font-weight: 600; }
.online::before { content: ""; display: inline-block; width: 7px; height: 7px;
  margin-right: 7px; border-radius: 50%; background: var(--ok); }
.panel {
  margin-bottom: 16px;
  padding: 18px;
  border: 1px solid var(--line);
  border-radius: 8px;
  background: var(--panel);
  box-shadow: 0 1px 2px rgb(16 24 40 / 4%);
}
.facts {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  padding: 0;
  list-style: none;
}
.facts li { padding: 0 18px; border-right: 1px solid var(--line); }
.facts li:first-child { padding-left: 0; }
.facts li:last-child { border-right: 0; }
.label { display: block; margin-bottom: 4px; color: var(--muted); font-size: 12px; }
.value { font-weight: 600; }
.upload-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 14px;
}
label { display: grid; gap: 6px; color: #344054; font-size: 13px; }
input {
  width: 100%;
  min-height: 38px;
  padding: 8px 10px;
  border: 1px solid #c8ced8;
  border-radius: 6px;
  background: #fff;
  color: var(--text);
  font: inherit;
}
input:focus { outline: 2px solid rgb(36 87 214 / 20%); border-color: var(--accent); }
.file-row { grid-column: 1 / -1; }
.actions { display: flex; align-items: center; gap: 12px; grid-column: 1 / -1; }
button {
  min-height: 38px;
  padding: 8px 16px;
  border: 1px solid #1f4cbd;
  border-radius: 6px;
  background: var(--accent);
  color: #fff;
  font: inherit;
  font-weight: 600;
  cursor: pointer;
}
button:disabled { cursor: wait; opacity: .6; }
.secondary {
  border-color: #c8ced8;
  background: #fff;
  color: #344054;
}
progress { width: min(320px, 100%); accent-color: var(--accent); }
.upload-status { min-height: 21px; margin-top: 10px; }
.upload-status.error { color: var(--error); }
.hint { font-size: 12px; }
.object-list { display: grid; gap: 12px; }
.object {
  display: grid;
  grid-template-columns: minmax(0, 1fr) auto;
  gap: 14px;
  align-items: center;
  padding: 14px 0;
  border-top: 1px solid var(--line);
}
.object:first-child { padding-top: 0; border-top: 0; }
.object h2 { margin: 0; font-size: 14px; }
.object-meta { overflow-wrap: anywhere; font-size: 12px; }
.object video {
  grid-column: 1 / -1;
  display: block;
  width: 100%;
  max-height: 480px;
  background: #000;
}
.download { white-space: nowrap; }
.capabilities { display: flex; flex-wrap: wrap; gap: 7px; padding: 0; list-style: none; }
.capabilities li {
  padding: 4px 8px;
  border: 1px solid var(--line);
  border-radius: 4px;
  color: #475467;
  background: #fafafa;
  font-size: 12px;
}
footer { padding-top: 8px; color: var(--muted); font-size: 12px; }
@media (max-width: 680px) {
  .shell { width: min(100% - 20px, 1080px); }
  .summary, .topbar .shell { align-items: flex-start; flex-direction: column; }
  .topbar .shell { justify-content: center; padding: 10px 0; gap: 2px; }
  .upload-grid, .facts { grid-template-columns: 1fr; }
  .facts li { padding: 10px 0; border-right: 0; border-bottom: 1px solid var(--line); }
  .facts li:last-child { border-bottom: 0; }
  .actions { align-items: stretch; flex-direction: column; }
  progress { width: 100%; }
}
"""


UPLOAD_SCRIPT = r"""
(() => {
  "use strict";

  const byId = (id) => document.getElementById(id);
  const form = byId("browser-upload-form");
  if (!form) return;

  const owner = byId("browser-owner");
  const repository = byId("browser-repository");
  const token = byId("browser-token");
  const files = byId("browser-files");
  const button = byId("browser-upload-button");
  const browse = byId("browser-browse-button");
  const progress = byId("browser-upload-progress");
  const status = byId("browser-upload-status");
  const validName = /^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$/;
  const maximumChunkSize = 4 * 1024 * 1024;
  const minimumChunkSize = 256 * 1024;
  const demoFileLimit = 16 * 1024 * 1024;
  const demoChunkSize = 1024 * 1024;
  const pause = (milliseconds) => new Promise((resolve) =>
    window.setTimeout(resolve, milliseconds));
  const authorizationHeaders = () => token.value
    ? {"Authorization": `Bearer ${token.value}`}
    : {};

  const repositoryUrl = () => {
    const query = new URLSearchParams({
      owner: owner.value.trim(),
      repository: repository.value.trim(),
    });
    return `/?${query.toString()}`;
  };

  const namesAreValid = () => {
    const valid = validName.test(owner.value.trim()) &&
      validName.test(repository.value.trim());
    if (!valid) {
      status.classList.add("error");
      status.textContent = "Owner and repository names may use letters, numbers, dots, underscores, and hyphens.";
    }
    return valid;
  };

  browse.addEventListener("click", () => {
    status.classList.remove("error");
    if (namesAreValid()) window.location.assign(repositoryUrl());
  });

  const requestJson = (method, target, options = {}) => new Promise((resolve, reject) => {
    const request = new XMLHttpRequest();
    request.open(method, target);
    for (const [name, value] of Object.entries(options.headers || {})) {
      request.setRequestHeader(name, value);
    }
    if (options.onProgress) request.upload.onprogress = options.onProgress;
    request.onerror = () => {
      const error = new Error("Network error while uploading.");
      error.status = 0;
      reject(error);
    };
    request.onload = () => {
      let payload = {};
      try {
        payload = JSON.parse(request.responseText || "{}");
      } catch (_) {
        // The HTTP status still provides a useful fallback.
      }
      if (request.status >= 200 && request.status < 300) {
        resolve(payload);
        return;
      }
      const error = new Error(payload.error || `Upload failed (HTTP ${request.status}).`);
      error.status = request.status;
      reject(error);
    };
    request.send(options.body || null);
  });

  const withRetries = async (operation) => {
    let lastError;
    for (let attempt = 0; attempt < 4; attempt += 1) {
      try {
        return await operation();
      } catch (error) {
        lastError = error;
        if (error.status > 0 && error.status < 500) throw error;
        if (attempt < 3) await pause(300 * (attempt + 1));
      }
    }
    throw lastError;
  };

  const uploadOne = async (file, index, total) => {
    const collection = `/api/v1/repositories/${encodeURIComponent(owner.value.trim())}/` +
      `${encodeURIComponent(repository.value.trim())}/uploads`;

    const created = await withRetries(() => requestJson("POST", collection, {
      headers: {
        ...authorizationHeaders(),
        "X-Dimos-Filename": encodeURIComponent(file.name),
        "X-Dimos-Size": String(file.size),
        "X-Dimos-Content-Type": file.type || "application/octet-stream",
      },
    }));
    const sessionUrl = `${collection}/${encodeURIComponent(created.upload_id)}`;
    let offset = Number(created.received_bytes || 0);
    let chunkSize = file.size <= demoFileLimit ? demoChunkSize : maximumChunkSize;
    let consecutiveFailures = 0;

    while (offset < file.size) {
      const chunkEnd = Math.min(offset + chunkSize, file.size);
      const chunkStart = offset;
      try {
        const updated = await requestJson("PUT", sessionUrl, {
          body: file.slice(chunkStart, chunkEnd),
          headers: {
            ...authorizationHeaders(),
            "Content-Type": "application/octet-stream",
            "X-Dimos-Offset": String(chunkStart),
          },
          onProgress: (event) => {
            if (!event.lengthComputable) return;
            const fileProgress = (chunkStart + event.loaded) / Math.max(file.size, 1);
            progress.value = ((index + fileProgress) / total) * 100;
            status.textContent = `Uploading ${file.name} (${Math.round(fileProgress * 100)}%)`;
          },
        });
        offset = Number(updated.received_bytes);
        consecutiveFailures = 0;
      } catch (error) {
        consecutiveFailures += 1;
        if (
          consecutiveFailures >= 6 ||
          (error.status > 0 && error.status < 500 && error.status !== 409)
        ) {
          throw error;
        }
        const proxyFailure = error.status === 0 || error.status >= 500;
        await pause(300 * consecutiveFailures);
        try {
          const state = await requestJson("GET", sessionUrl, {
            headers: authorizationHeaders(),
          });
          const serverOffset = Number(state.received_bytes);
          if (serverOffset < chunkStart || serverOffset > chunkEnd) {
            throw new Error("Server returned an invalid upload offset.");
          }
          if (
            proxyFailure &&
            serverOffset === chunkStart &&
            chunkSize > minimumChunkSize
          ) {
            chunkSize = Math.max(minimumChunkSize, Math.floor(chunkSize / 2));
            status.textContent = `Proxy interrupted the upload. Retrying ${file.name} with ` +
              `${Math.round(chunkSize / 1024)}KB chunks...`;
          }
          offset = serverOffset;
        } catch (statusError) {
          if (statusError.message === "Server returned an invalid upload offset.") {
            throw statusError;
          }
          if (proxyFailure && chunkSize > minimumChunkSize) {
            chunkSize = Math.max(minimumChunkSize, Math.floor(chunkSize / 2));
          }
        }
      }
    }

    return withRetries(() => requestJson("POST", `${sessionUrl}/complete`, {
      headers: authorizationHeaders(),
    }));
  };

  form.addEventListener("submit", async (event) => {
    event.preventDefault();
    status.classList.remove("error");
    const selected = Array.from(files.files || []);
    const ownerName = owner.value.trim();
    const repositoryName = repository.value.trim();
    if (!form.reportValidity() || !selected.length) return;
    if (!namesAreValid()) return;

    button.disabled = true;
    try {
      for (let index = 0; index < selected.length; index += 1) {
        await uploadOne(selected[index], index, selected.length);
      }
      progress.value = 100;
      status.textContent = `${selected.length} file(s) uploaded. Refreshing objects...`;
      window.setTimeout(() => window.location.assign(repositoryUrl()), 400);
    } catch (error) {
      status.classList.add("error");
      status.textContent = error instanceof Error ? error.message : String(error);
    } finally {
      button.disabled = false;
    }
  });
})();
"""


def _document(*, title: str, node_name: str, content: str) -> str:
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>{escape(title)}</title>
  <style>{_STYLE}</style>
</head>
<body>
  <nav class="topbar">
    <div class="shell">
      <div class="brand">DimOS Replay Repository</div>
      <div class="node mono">{escape(node_name)}</div>
    </div>
  </nav>
  <main class="shell">{content}</main>
  <script src="/assets/hosted-data.js?v=6" defer></script>
</body>
</html>"""


def render_upload_panel(
    owner: str = "",
    repository: str = "",
    *,
    auth_required: bool = True,
) -> str:
    """Render a same-origin upload form without embedding or storing its token."""
    if auth_required:
        token_control = (
            '<label>Upload token'
            '<input id="browser-token" type="password" required autocomplete="off">'
            '</label>'
        )
        initial_status = "Ready. The token is used only for this upload."
    else:
        token_control = '<input id="browser-token" type="hidden" value="">'
        initial_status = "Ready. This demo node accepts uploads without a token."
    return f"""
<section class="panel" aria-labelledby="upload-title">
  <h2 id="upload-title">Upload data</h2>
  <form id="browser-upload-form" class="upload-grid">
    <label>Owner
      <input id="browser-owner" required maxlength="64" value="{escape(owner, quote=True)}">
    </label>
    <label>Repository
      <input id="browser-repository" required maxlength="64"
             value="{escape(repository, quote=True)}">
    </label>
    {token_control}
    <label class="file-row">Files
      <input id="browser-files" type="file" multiple required
             accept="video/*,.db,.mcap,.json,.bin">
    </label>
    <div class="actions">
      <button id="browser-upload-button" type="submit">Upload</button>
      <button id="browser-browse-button" class="secondary" type="button">Open repository</button>
      <progress id="browser-upload-progress" max="100" value="0"></progress>
    </div>
  </form>
  <p id="browser-upload-status" class="upload-status">{initial_status}</p>
  <p class="hint">Uploads use adaptive resumable chunks. The DimOS CLI remains available for automation.</p>
</section>"""


def render_status_page(
    *,
    node_name: str,
    region: str,
    access_mode: str,
    capabilities: Iterable[str],
    owner: str = "",
    repository: str = "",
    objects: Iterable[RepositoryObjectView] = (),
    public_write: bool = False,
) -> str:
    """Render the public service status and upload page."""
    capability_items = "".join(f"<li>{escape(capability)}</li>" for capability in capabilities)
    repository_panel = ""
    if owner and repository:
        repository_panel = _render_objects_panel(
            owner=owner,
            repository=repository,
            objects=objects,
        )
    content = f"""
<section class="summary">
  <div>
    <h1>Hosted replay data</h1>
    <p>Upload, verify, share, and replay robotics datasets.</p>
  </div>
  <div class="online">Service online</div>
</section>
<section class="panel">
  <ul class="facts" aria-label="Node details">
    <li><span class="label">Region</span><span class="value">{escape(region)}</span></li>
    <li><span class="label">Access</span><span class="value">{escape(access_mode)}</span></li>
    <li><span class="label">API</span><span class="value">Hosted data v1</span></li>
  </ul>
</section>
{render_upload_panel(owner, repository, auth_required=not public_write)}
{repository_panel}
<section class="panel">
  <h2>Capabilities</h2>
  <ul class="capabilities">{capability_items}</ul>
</section>
<footer>
  Operator endpoints: <a href="/healthz">health</a> &middot;
  <a href="/api/v1/nodes">node discovery</a>
</footer>"""
    return _document(
        title=f"DimOS replay - {node_name}",
        node_name=node_name,
        content=content,
    )


def render_repository_page(
    *,
    node_name: str,
    owner: str,
    repository: str,
    objects: Iterable[RepositoryObjectView],
) -> str:
    """Render one repository with browser previews and explicit downloads."""
    content = f"""
<section class="summary">
  <div>
    <h1>{escape(owner)} / {escape(repository)}</h1>
    <p>Repository objects are content-addressed and SHA-256 verified.</p>
  </div>
</section>
{render_upload_panel(owner, repository)}
{_render_objects_panel(owner=owner, repository=repository, objects=objects)}"""
    return _document(
        title=f"{owner}/{repository} - DimOS replay",
        node_name=node_name,
        content=content,
    )


def _render_objects_panel(
    *,
    owner: str,
    repository: str,
    objects: Iterable[RepositoryObjectView],
) -> str:
    rows: list[str] = []
    for item in objects:
        object_url = escape(item.object_url, quote=True)
        preview = (
            f'<video controls preload="metadata" src="{object_url}"></video>'
            if item.content_type.startswith("video/")
            else ""
        )
        rows.append(
            '<article class="object">'
            "<div>"
            f"<h2>{escape(item.filename)}</h2>"
            f'<p class="object-meta mono">{item.size_bytes:,} bytes &middot; '
            f"SHA-256 {escape(item.sha256)}</p>"
            "</div>"
            f'<a class="download" href="{object_url}?download=1">Download</a>'
            f"{preview}"
            "</article>"
        )
    object_list = "".join(rows) or '<p class="empty">No objects uploaded.</p>'
    return f"""
<section class="panel">
  <h2>Objects &middot; {escape(owner)} / {escape(repository)}</h2>
  <div class="object-list">{object_list}</div>
</section>"""
