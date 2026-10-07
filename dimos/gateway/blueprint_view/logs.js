// Logs: a run's log in a modal over the view, followed live (GET /dimos/runs/{runId}/log?after=&level=&q=), with a
// lowest-level filter, a search and follow (keep the newest line in view).

const LEVELS = ["debug", "info", "warning", "error", "critical"]
const MAX_RECORDS = 5000

const shortTime = (timestamp) => /T?(\d\d:\d\d:\d\d)/.exec(timestamp)?.[1] ?? timestamp

export function openLogs({ runId, title, h, getJson, onClose }) {
    let level = "info"
    let query = ""
    let follow = true
    let offset = 0
    let records = []
    let timer
    let closed = false
    const lines = h("div", { class: "log-body" })
    const close = () => {
        closed = true
        clearTimeout(timer)
        scrim.remove()
        removeEventListener("keydown", escape, true)
        onClose?.()
    }
    const escape = (event) => {
        if (event.key === "Escape") {
            event.stopPropagation()
            close()
        }
    }
    addEventListener("keydown", escape, true)
    const restart = (delay) => {
        clearTimeout(timer)
        offset = 0
        records = []
        draw(null)
        timer = setTimeout(poll, delay)
    }
    const levelPick = h(
        "select",
        { "aria-label": "lowest level", onchange: (event) => {
            level = event.target.value
            restart(0)
        } },
        LEVELS.map((name) => h("option", { value: name }, `${name} and up`)),
    )
    levelPick.value = level
    const followBox = h("input", { type: "checkbox", onchange: (event) => (follow = event.target.checked) })
    followBox.checked = true
    const dialog = h(
        "div",
        { class: "modal logs", role: "dialog", "aria-label": `${title} logs`, "data-bp-logs-modal": true },
        h(
            "div",
            { class: "modal-head" },
            h("span", { class: "label" }, `Logs · ${title}`),
            h("span", { class: "cfg-sub" }, runId),
            h("span", { class: "spacer" }),
            h("button", { type: "button", class: "btn", onclick: close }, "Close"),
        ),
        h(
            "div",
            { class: "log-tools" },
            levelPick,
            h("input", {
                type: "search",
                placeholder: "search the log…",
                "aria-label": "search the log",
                oninput: (event) => {
                    query = event.target.value
                    restart(250)
                },
            }),
            h("label", { class: "follow" }, followBox, "follow"),
        ),
        lines,
    )
    const scrim = h("div", { class: "scrim", onpointerdown: (event) => event.target === scrim && close() }, dialog)
    document.body.append(scrim)

    function draw(error) {
        lines.replaceChildren(...[
            error && h("div", { class: "hint" }, error),
            !error && records.length === 0 && h("div", { class: "hint" }, "No log lines yet."),
            records.map((record) =>
                h(
                    "div",
                    { class: "log-line", title: record.raw },
                    h("span", { class: "ts" }, shortTime(record.timestamp)),
                    h("span", { class: `lv ${record.level.toLowerCase()}` }, record.level),
                    h(
                        "span",
                        { class: "ev" },
                        h("span", { class: "lg" }, record.logger),
                        record.event,
                        Object.keys(record.extra ?? {}).length > 0 && h(
                            "span",
                            { class: "lg" },
                            " " + Object.entries(record.extra).map(([key, value]) =>
                                `${key}=${typeof value === "string" ? value : JSON.stringify(value)}`
                            ).join(" "),
                        ),
                    ),
                )
            ),
        ].flat().filter(Boolean))
        if (follow) {
            lines.scrollTop = lines.scrollHeight
        }
    }

    async function poll() {
        const params = new URLSearchParams({ after: String(offset), level })
        if (query) {
            params.set("q", query)
        }
        try {
            const page = await getJson(`runs/${encodeURIComponent(runId)}/log?${params}`)
            if (closed) {
                return
            }
            offset = page.offset
            if (page.records.length) {
                records = [...records, ...page.records].slice(-MAX_RECORDS)
                draw(null)
            } else if (records.length === 0) {
                draw(null)
            }
        } catch (error) {
            if (!closed) {
                draw(String(error.message ?? error))
            }
        }
        if (!closed) {
            timer = setTimeout(poll, 1500)
        }
    }
    poll()
}
