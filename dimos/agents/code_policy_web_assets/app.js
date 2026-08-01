(() => {
  "use strict";

  const notebook = document.getElementById("notebook");
  const empty = document.getElementById("empty");
  const connection = document.getElementById("connection");
  const generation = document.getElementById("generation");
  const policySession = document.getElementById("policy-session");
  const clientSession = document.getElementById("client-session");
  const recording = document.getElementById("recording");
  const follow = document.getElementById("follow");
  const cells = new Map();
  const pendingClear = new Set();
  const ansi = /[\u001b\u009b][[\]()#;?]*(?:(?:[a-zA-Z\d]*(?:;[-a-zA-Z\d\/#&.:=?%@~_]+)*)?\u0007|(?:(?:\d{1,4}(?:;\d{0,4})*)?[\dA-PR-TZcf-nq-uy=><~]))/g;

  function text(value) {
    return String(value ?? "").replace(ansi, "");
  }

  function scrollLatest() {
    if (follow.checked) window.scrollTo({ top: document.body.scrollHeight, behavior: "smooth" });
  }

  function updateHeader(state) {
    const status = text(state.status || "connecting");
    connection.textContent = status;
    connection.className = `badge ${status === "ready" ? "ready" : "pending"}`;
    generation.textContent = `generation ${state.kernel_generation ?? "—"}`;
    policySession.textContent = state.code_policy_session_id || "—";
    clientSession.textContent = state.jupyter_client_session_id || "—";
    recording.textContent = state.recording_path || "—";
    recording.title = state.recording_path || "";
  }

  function refreshSession() {
    return fetch("/api/session", { cache: "no-store" })
      .then((response) => response.json())
      .then(updateHeader);
  }

  function lifecycle(event) {
    const divider = document.createElement("div");
    divider.className = "lifecycle";
    divider.textContent = text(event.content?.event || "lifecycle");
    notebook.appendChild(divider);
    updateHeader({
      status: event.content?.event,
      kernel_generation: event.kernel_generation,
      code_policy_session_id: event.code_policy_session_id,
      jupyter_client_session_id: clientSession.textContent === "—" ? null : clientSession.textContent,
      recording_path: recording.textContent === "—" ? null : recording.textContent,
    });
    refreshSession().catch(() => {});
  }

  function createCell(event) {
    empty?.remove();
    const root = document.createElement("article");
    root.className = "cell";
    const prompt = document.createElement("div");
    prompt.className = "prompt";
    prompt.textContent = `In [${event.execution_count ?? "?"}]:`;
    const body = document.createElement("div");
    body.className = "cell-body";
    const source = document.createElement("pre");
    source.className = "source";
    source.textContent = text(event.content?.code || (event.truncated ? "# [source truncated]" : ""));
    const outputs = document.createElement("div");
    outputs.className = "outputs";
    const completion = document.createElement("div");
    completion.className = "completion";
    completion.hidden = true;
    body.append(source, outputs, completion);
    root.append(prompt, body);
    notebook.appendChild(root);
    const cell = { root, outputs, completion };
    cells.set(event.parent_message_id, cell);
    return cell;
  }

  function outputNode(value, className = "") {
    const node = document.createElement("pre");
    node.className = `output ${className}`.trim();
    node.textContent = text(value);
    return node;
  }

  function clearIfPending(parent, cell) {
    if (pendingClear.has(parent)) {
      cell.outputs.replaceChildren();
      pendingClear.delete(parent);
    }
  }

  function richOutput(data, cell) {
    const plain = data?.["text/plain"];
    if (plain !== undefined) {
      cell.outputs.appendChild(outputNode(plain, "result"));
      return;
    }
    for (const mime of ["image/png", "image/jpeg"]) {
      if (typeof data?.[mime] === "string") {
        const wrapper = document.createElement("div");
        wrapper.className = "output result";
        const image = document.createElement("img");
        image.alt = `Observed ${mime} output`;
        image.src = `data:${mime};base64,${data[mime]}`;
        wrapper.appendChild(image);
        cell.outputs.appendChild(wrapper);
        return;
      }
    }
    cell.outputs.appendChild(outputNode("[rich output retained in recording]", "notice"));
  }

  function observation(event) {
    if (event.kind === "lifecycle") {
      lifecycle(event);
      scrollLatest();
      return;
    }
    if (event.kind === "record") {
      const cell = cells.get(event.parent_message_id);
      if (cell) {
        const duration = Number(event.content?.monotonic_duration_s);
        const suffix = Number.isFinite(duration) ? ` in ${duration.toFixed(2)}s` : "";
        cell.completion.textContent = `${text(event.content?.status || "completed")}${suffix}`;
        cell.completion.hidden = false;
      }
      scrollLatest();
      return;
    }
    const parent = event.parent_message_id;
    if (event.message_type === "execute_input") {
      createCell(event);
      scrollLatest();
      return;
    }
    const cell = cells.get(parent);
    if (!cell) return;
    if (event.message_type === "clear_output") {
      if (event.content?.wait) pendingClear.add(parent);
      else cell.outputs.replaceChildren();
      return;
    }
    clearIfPending(parent, cell);
    if (event.message_type === "stream") {
      const kind = event.content?.name === "stderr" ? "stderr" : "stdout";
      cell.outputs.appendChild(outputNode(event.content?.text, kind));
    } else if (["execute_result", "display_data"].includes(event.message_type)) {
      richOutput(event.content?.data, cell);
    } else if (event.message_type === "error") {
      const traceback = Array.isArray(event.content?.traceback)
        ? event.content.traceback.join("\n")
        : `${event.content?.ename || "Error"}: ${event.content?.evalue || ""}`;
      cell.outputs.appendChild(outputNode(traceback, "error"));
    }
    if (event.truncated) {
      const refs = Array.isArray(event.artifact_refs) ? event.artifact_refs.join(", ") : "none";
      cell.outputs.appendChild(outputNode(`[observation truncated; artifacts: ${refs}]`, "notice"));
    }
    scrollLatest();
  }

  refreshSession()
    .catch(() => updateHeader({ status: "session unavailable" }));

  const events = new EventSource("/api/events");
  events.addEventListener("open", () => {
    if (connection.textContent === "connecting") connection.textContent = "attached";
  });
  events.addEventListener("observation", (message) => observation(JSON.parse(message.data)));
  events.addEventListener("observer-close", () => {
    connection.textContent = "observer closed";
    connection.className = "badge error";
    events.close();
  });
  events.onerror = () => {
    connection.textContent = "reconnecting";
    connection.className = "badge pending";
  };
})();
