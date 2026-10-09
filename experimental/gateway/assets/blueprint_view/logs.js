// Logs: a run's log in a modal over the view, followed live (GET /dimos/runs/{runId}/log?after=&level=&q=), with a
// lowest-level filter, a search and follow: pinned to the newest line (where a failed run's error is) until scrolled up,
// pinned again once scrolled back to the bottom. A record's fields are chips; a traceback is drawn as one: library frames
// folded away, the project's frames opening in the editor, the exceptions it raised in red at its end.

import { exceptionsOf, isTraceback, parseTraceback, projectPath } from "./traceback.js"

const LEVELS = ["debug", "info", "warning", "error", "critical"]
const MAX_RECORDS = 5000
// this close to the bottom (px) counts as at the bottom
const BOTTOM_SLACK = 24

const shortTime = (timestamp) => /T?(\d\d:\d\d:\d\d)/.exec(timestamp)?.[1] ?? timestamp

/**
 * `openInEditor(file, line, status)`: open a project file at a line (null: no editor here); `status`, an element to
 * show the outcome in
 */
export function openLogs({ runId, title, h, getJson, openInEditor, onClose }) {
    let level = "info"
    let query = ""
    let follow = true
    let offset = 0
    let count = 0
    // bumped by a restart: an answer to an older poll is dropped
    let generation = 0
    let timer
    let closed = false
    const hint = h("div", { class: "hint" }, "loading…")
    const content = h("div", { class: "log-content" })
    const lines = h("div", { class: "log-body" }, hint, content)
    const status = h("div", { class: "opened", hidden: true })
    const atBottom = () => lines.scrollHeight - lines.scrollTop - lines.clientHeight <= BOTTOM_SLACK
    const pin = () => {
        if (follow) {
            lines.scrollTop = lines.scrollHeight
        }
    }
    const setFollow = (on) => {
        follow = on
        followBox.checked = on
        pin()
    }
    // scrolled up: stop following; back at the bottom: follow again
    lines.addEventListener("scroll", () => {
        if (atBottom() !== follow) {
            setFollow(atBottom())
        }
    })
    // opening a folded frame is reading, not following
    lines.addEventListener("toggle", (event) => event.target.open && setFollow(false), true)
    // the log, or the modal, changed size (lines came in, it was laid out, a font loaded): keep the bottom in view
    const resized = new ResizeObserver(pin)
    resized.observe(lines)
    resized.observe(content)
    const close = () => {
        closed = true
        clearTimeout(timer)
        resized.disconnect()
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
        generation += 1
        offset = 0
        count = 0
        content.replaceChildren()
        showHint("loading…")
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
    const followBox = h("input", { type: "checkbox", onchange: (event) => setFollow(event.target.checked) })
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
        status,
        lines,
    )
    const scrim = h("div", { class: "scrim", onpointerdown: (event) => event.target === scrim && close() }, dialog)
    document.body.append(scrim)

    function showHint(text) {
        hint.hidden = !text
        hint.textContent = text ?? ""
    }

    /** a project frame's file:line: opens in the editor (when there is one) */
    const fileLink = (file, line) => {
        const shown = `${projectPath(file)}:${line}`
        return openInEditor
            ? h("button", {
                type: "button",
                class: "tb-file",
                title: `${file}:${line} (open in the editor)`,
                "data-bp-log-open": true,
                onclick: () => openInEditor(projectPath(file), line, status),
            }, shown)
            : h("span", { class: "tb-file", title: `${file}:${line}` }, shown)
    }

    function frameRows(frame) {
        const head = h(
            "div",
            { class: `tb-row tb-frame${frame.library ? " lib" : " own"}` },
            h("span", { class: "tb-gutter" }, frame.gutter),
            frame.indent,
            frame.library ? h("span", { class: "tb-path", title: frame.file }, `${frame.file}:${frame.line}`) : fileLink(frame.file, frame.line),
            frame.func && h("span", { class: "tb-func" }, ` in ${frame.func}`),
        )
        return [
            head,
            frame.code.map(({ text, marker }) =>
                h(
                    "div",
                    { class: `tb-row ${marker ? "tb-marker" : "tb-code"}${frame.library ? "" : " own"}` },
                    h("span", { class: "tb-gutter" }, frame.gutter),
                    text,
                )
            ),
        ]
    }

    function tracebackBlock(text) {
        const items = parseTraceback(text)
        const rows = []
        for (let i = 0; i < items.length; i++) {
            const item = items[i]
            if (item.kind === "frame" && item.library) {
                // a run of library frames: folded into one toggle
                let end = i
                while (items[end + 1]?.kind === "frame" && items[end + 1].library) {
                    end += 1
                }
                const run = items.slice(i, end + 1)
                rows.push(h(
                    "details",
                    { class: "tb-lib" },
                    h(
                        "summary",
                        { class: "tb-row" },
                        h("span", { class: "tb-gutter" }, item.gutter),
                        item.indent,
                        `${run.length} library frame${run.length === 1 ? "" : "s"}`,
                    ),
                    run.map(frameRows).flat(2),
                ))
                i = end
            } else if (item.kind === "frame") {
                rows.push(frameRows(item))
            } else if (item.kind === "exception") {
                rows.push(item.lines.map((line) =>
                    h("div", { class: "tb-row tb-exc" }, h("span", { class: "tb-gutter" }, item.gutter), line)
                ))
            } else {
                rows.push(h(
                    "div",
                    { class: `tb-row tb-${item.kind}` },
                    h("span", { class: "tb-gutter" }, item.gutter),
                    item.text,
                ))
            }
        }
        const raised = exceptionsOf(items)
        return h(
            "div",
            { class: "tb" },
            h("div", { class: "tb-lines" }, rows.flat(2)),
            raised.length > 0 && h(
                "div",
                { class: "tb-raised", "data-bp-log-raised": true },
                raised.map(({ depth, text }, index) => {
                    const leaf = !(raised[index + 1]?.depth > depth) && !/^\w*ExceptionGroup:/.test(text)
                    return h("div", { class: `tb-raised-one${leaf ? " leaf" : ""}`, "--depth": String(depth) }, text)
                }),
            ),
        )
    }

    const fieldText = (value) => typeof value === "string" ? value : JSON.stringify(value)

    function recordNode(record) {
        const extra = { ...(record.extra ?? {}) }
        const traceback = Array.isArray(extra.traceback_lines)
            ? extra.traceback_lines.join("")
            : isTraceback(extra.exception)
            ? extra.exception
            : null
        if (traceback) {
            // the traceback says all of these (its last lines are the exception's type and message)
            for (const key of ["traceback_lines", "exception", "exception_type", "exception_message"]) {
                delete extra[key]
            }
        }
        // where in the code it was logged: the logger's tooltip, not a chip on every line
        const where = [extra.lineno !== undefined && `line ${extra.lineno}`, extra.func_name && `in ${extra.func_name}`]
            .filter(Boolean).join(" ")
        delete extra.lineno
        delete extra.func_name
        const long = Object.entries(extra).filter(([, value]) => typeof value === "string" && value.includes("\n"))
        const short = Object.entries(extra).filter(([, value]) => !(typeof value === "string" && value.includes("\n")))
        return h(
            "div",
            { class: `log-rec lv-${record.level.toLowerCase()}`, title: traceback ? undefined : record.raw },
            h(
                "div",
                { class: "log-line" },
                h("span", { class: "ts" }, shortTime(record.timestamp)),
                h("span", { class: `lv ${record.level.toLowerCase()}` }, record.level),
                h(
                    "span",
                    { class: "ev" },
                    h("span", { class: "lg", title: where ? `${record.logger} ${where}` : record.logger }, record.logger),
                    record.event,
                    short.map(([key, value]) =>
                        h("span", { class: "chip" }, h("span", { class: "k" }, key), fieldText(value))
                    ),
                ),
            ),
            long.map(([key, value]) =>
                h("div", { class: "log-field" }, h("span", { class: "k" }, key), h("pre", {}, value.replace(/\n$/, "")))
            ),
            traceback && tracebackBlock(traceback),
        )
    }

    async function poll() {
        const asked = generation
        const params = new URLSearchParams({ after: String(offset), level })
        if (query) {
            params.set("q", query)
        }
        try {
            const page = await getJson(`runs/${encodeURIComponent(runId)}/log?${params}`)
            if (closed || asked !== generation) {
                return
            }
            offset = page.offset
            // the newest MAX_RECORDS at most: a long first page starts that far from its end
            const fresh = page.records.slice(-MAX_RECORDS)
            content.append(...fresh.map(recordNode))
            count += fresh.length
            while (count > MAX_RECORDS) {
                content.firstChild.remove()
                count -= 1
            }
            showHint(count === 0 ? "No log lines yet." : null)
            pin()
        } catch (error) {
            if (closed || asked !== generation) {
                return
            }
            showHint(String(error.message ?? error))
        }
        if (!closed && asked === generation) {
            timer = setTimeout(poll, 1500)
        }
    }
    poll()
}
