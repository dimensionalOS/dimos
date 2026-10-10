import { openConfig, same } from "./config.js"
import { className, Graph, LAYOUTS, topicOf, typeColor, typeName } from "./graph.js"
import { openLogs } from "./logs.js"

const PARAMS = new URLSearchParams(location.search)
const NAME = PARAMS.get("name") ?? ""
const RUN = PARAMS.get("run") || null
const VIEW = PARAMS.get("view")
const FRAMED = parent !== window
const $ = (selector) => document.querySelector(selector)

const state = {
    modules: null,
    selected: null,
    hovered: null,
    code: null,
    source: null,
    running: false,
    extras: [],
    showExtras: false,
    pinnedTopic: null,
    rateRows: [],
    launch: null,
    saved: null,
    relaunching: false,
    relaunchedPid: null,
    actionError: null,
}

function applySkin() {
    try {
        document.documentElement.dataset.skin = localStorage.getItem("portal.theme") || "portal"
        const corners = localStorage.getItem("portal.corners")
        if (corners === "sharp" || corners === "rounded") {
            document.documentElement.dataset.corners = corners
        } else {
            delete document.documentElement.dataset.corners
        }
    } catch {
    }
}
applySkin()
addEventListener("storage", (event) => {
    if (event.key === null || event.key === "portal.theme" || event.key === "portal.corners") {
        applySkin()
        graph.nodes = []
        graph.setModel(state.modules ?? [], state.showExtras ? state.extras : [])
    }
})

const h = (tag, attrs = {}, ...children) => {
    const node = document.createElement(tag)
    for (const [key, value] of Object.entries(attrs)) {
        if (value === undefined || value === null || value === false) {
            continue
        }
        if (key.startsWith("on")) {
            node.addEventListener(key.slice(2), value)
        } else if (key === "style") {
            Object.assign(node.style, value)
        } else if (key.startsWith("--")) {
            node.style.setProperty(key, value)
        } else {
            node.setAttribute(key, value === true ? "" : value)
        }
    }
    node.append(...children.flat().filter((child) => child !== null && child !== undefined && child !== false))
    return node
}

async function getJson(url, options) {
    const response = await fetch(url, options)
    const data = await response.json().catch(() => null)
    if (!response.ok) {
        throw Object.assign(new Error(data?.error ?? `${response.status} ${response.statusText}`), {
            status: response.status,
        })
    }
    return data
}

const send = (method, url, body) =>
    getJson(url, { method, headers: { "content-type": "application/json" }, body: JSON.stringify(body ?? {}) })

const reads = (stream) => stream.direction !== "out"
const writes = (stream) => stream.direction !== "in"
const typeVar = (type) => `var(--bv-${typeColor(type)})`
const aliasOf = (module) => module.name !== className(module).toLowerCase() ? module.name : null

const graph = new Graph($("#graph"), {
    module: (name) => {
        state.pinnedTopic = null
        open(name)
    },
    topic: (name) => {
        state.pinnedTopic = state.pinnedTopic === name ? null : name
        graph.setSpot(spotNow())
    },
    hover: (target) => graph.setSpot(target ?? spotNow()),
})
const spotNow = () => {
    if (state.hovered) {
        return { module: state.hovered }
    }
    if (state.pinnedTopic) {
        return { topic: state.pinnedTopic }
    }
    return state.selected ? { module: state.selected } : null
}
const tabs = $("#layouts")
for (const { id, label } of LAYOUTS) {
    tabs.append(h("button", {
        type: "button",
        role: "tab",
        "data-layout": id,
        "aria-selected": String(id === graph.layoutId),
        onclick: () => {
            graph.setLayout(id)
            for (const tab of tabs.children) {
                tab.setAttribute("aria-selected", String(tab.dataset.layout === id))
            }
        },
    }, label))
}
$("#zoomIn").onclick = () => graph.zoom(1.25)
$("#zoomOut").onclick = () => graph.zoom(0.8)
$("#zoomFit").onclick = () => {
    graph.moved = false
    graph.fit()
}

function render() {
    const side = $("#side")
    const module = state.modules?.find((m) => m.name === state.selected)
    side.replaceChildren(module ? moduleView(module) : moduleList())
    graph.setSpot(spotNow())
    renderCode()
    renderBar()
}

function section(key, openByDefault, summary, ...children) {
    let open = openByDefault
    try {
        const kept = key && localStorage.getItem(key)
        open = kept == null ? openByDefault : kept === "1"
    } catch {
    }
    const box = h(
        "details",
        {
            ontoggle: (event) => {
                try {
                    if (key) localStorage.setItem(key, event.currentTarget.open ? "1" : "0")
                } catch {
                }
            },
        },
        h("summary", { class: "label" }, ...summary),
        ...children,
    )
    box.open = open
    return box
}

function moduleList() {
    const modules = state.modules
    const list = section(
        "bp.modulesOpen",
        true,
        ["Modules", modules && h("span", { class: "count" }, ` · ${modules.length}`)],
        state.error && h("div", { class: "note" }, state.error),
        !modules && !state.error && h("div", { class: "note" }, "loading…"),
        h(
            "ul",
            { onmouseleave: () => hover(null) },
            (modules ?? []).map((module) =>
                h(
                    "li",
                    {},
                    h(
                        "button",
                        {
                            type: "button",
                            class: "module",
                            "data-bp-module": module.name,
                            onclick: () => open(module.name),
                            onmouseenter: (event) => hover(module.name, event.currentTarget),
                            onfocus: (event) => hover(module.name, event.currentTarget),
                            onblur: () => hover(null),
                        },
                        h(
                            "span",
                            { class: "name" },
                            className(module),
                            aliasOf(module) && h("span", { class: "alias" }, module.name),
                        ),
                        module.summary && h("span", { class: "summary" }, module.summary),
                        ioCounts(module),
                        h("span", { class: "chev", "aria-hidden": "true" }, "›"),
                    ),
                )
            ),
        ),
    )
    list.classList.add("modules")
    return h("div", { class: "list" }, ratesSection(), list)
}

function ioCounts(module) {
    const streams = module.streams ?? []
    const inputs = streams.filter((stream) => stream.direction !== "out").length
    const outputs = streams.filter((stream) => stream.direction !== "in").length
    return h(
        "span",
        { class: "io" },
        h("span", { class: "io-in" }, `Inputs ${inputs}`),
        h("span", { class: "io-out" }, `Outputs ${outputs}`),
    )
}

function ratesSection() {
    const box = section(
        null,
        true,
        ["Topic rates", h("span", { class: "count", id: "ratesCount" })],
        h("div", { class: "rates-scroll" }, h("table", {}, h("tbody", { id: "ratesBody" }))),
    )
    box.classList.add("rates")
    box.id = "rates"
    queueMicrotask(fillRates)
    return box
}

function showTypeTip(cell, type) {
    let tip = document.getElementById("typeTip")
    if (!tip) {
        tip = h("div", { id: "typeTip", class: "type-tip" })
        document.body.append(tip)
    }
    const box = cell.getBoundingClientRect()
    tip.textContent = type
    tip.style.left = `${box.left}px`
    tip.style.top = `${box.bottom + 4}px`
    tip.hidden = false
}
function hideTypeTip() {
    const tip = document.getElementById("typeTip")
    if (tip) tip.hidden = true
}

function heat(value, max) {
    if (!(value > 0) || !(max > 0)) {
        return undefined
    }
    const t = Math.log1p(value) / Math.log1p(max)
    const hue = t < 0.65 ? 215 + (145 * t) / 0.65 : 360 - (35 * (t - 0.65)) / 0.35
    return { background: `hsl(${hue.toFixed(0)} 78% ${(42 + 14 * t).toFixed(0)}% / ${(0.22 + 0.5 * t).toFixed(2)})` }
}

function rateRows() {
    const heard = new Set(state.rateRows.map((row) => `${row.topic} ${row.type}`))
    const known = new Set(state.rateRows.map((row) => row.topic))
    const own = (state.modules ?? []).flatMap((m) => m.streams.map((stream) => ({ topic: `/${topicOf(stream)}`, type: stream.type })))
    const unheard = []
    for (const { topic, type } of own) {
        if (!known.has(topic) && !heard.has(`${topic} ${type}`)) {
            heard.add(`${topic} ${type}`)
            known.add(topic)
            unheard.push({ topic, type, hz: 0, bps: 0, messages: 0, lastSeen: null })
        }
    }
    return [...state.rateRows, ...unheard.sort((a, b) => a.topic.localeCompare(b.topic))]
}

function bandwidth(bytesPerSecond) {
    const units = ["B/s", "KB/s", "MB/s", "GB/s"]
    let value = bytesPerSecond
    let unit = 0
    while (value >= 1000 && unit < units.length - 1) {
        value /= 1000
        unit += 1
    }
    return `${value.toFixed(value < 10 && unit > 0 ? 1 : 0)} ${units[unit]}`
}

function fillRates() {
    const body = $("#ratesBody")
    if (!body) {
        return
    }
    const rows = rateRows()
    $("#ratesCount").textContent = rows.length ? ` · ${rows.filter((row) => row.hz > 0).length}/${rows.length} live` : ""
    const top = (key) => Math.max(...rows.map((row) => row[key] ?? 0), 0)
    const [maxHz, maxBps] = [top("hz"), top("bps")]
    const heardText = (row) =>
        row.lastSeen === null || row.lastSeen === undefined
            ? "never heard"
            : `${row.messages ?? 0} message${row.messages === 1 ? "" : "s"}, last ${row.lastSeen < 2 ? "just now" : `${Math.round(row.lastSeen)} s ago`}`
    body.replaceChildren(
        ...rows.map((row) => {
            const type = String(row.type ?? "").replace(/\//g, ".")
            return h(
                "tr",
                { class: row.hz > 0 ? "" : "quiet" },
                h(
                    "td",
                    {
                        class: "topic",
                        title: `${row.topic}\n${type || "unknown type"}\n${heardText(row)}`,
                        "--type": `var(--bv-${typeColor(type)})`,
                        onmouseenter: (event) => showTypeTip(event.currentTarget, type || "unknown type"),
                        onmouseleave: hideTypeTip,
                    },
                    row.topic,
                ),
                h("td", { class: "num", style: heat(row.hz ?? 0, maxHz) }, `${(row.hz ?? 0).toFixed(1)} Hz`),
                h("td", { class: "num", style: heat(row.bps ?? 0, maxBps) }, bandwidth(row.bps ?? 0)),
            )
        }),
        ...(rows.length ? [] : [
            h("tr", {}, h("td", { class: "empty", colspan: "3" }, state.ratesError ?? "no topics on the bus")),
        ]),
    )
}

function hover(name, row) {
    state.hovered = name
    graph.setSpot(spotNow())
    const flyout = $("#flyout")
    const module = state.modules?.find((m) => m.name === name)
    if (!module || !row || state.selected) {
        flyout.hidden = true
        return
    }
    const section = (label, list) =>
        h(
            "div",
            { class: "sec" },
            h("div", { class: "lab" }, label, h("span", { class: "n" }, String(list.length))),
            h(
                "div",
                { class: "pills" },
                list.length
                    ? list.map((stream) =>
                        h(
                            "span",
                            { class: "pill", title: stream.type, "--type": typeVar(stream.type) },
                            stream.name,
                            h("span", { class: "stype" }, typeName(stream.type)),
                        )
                    )
                    : h("span", { class: "none" }, "none"),
            ),
        )
    flyout.replaceChildren(
        section("Inputs", module.streams.filter(reads)),
        section("Outputs", module.streams.filter(writes)),
    )
    flyout.hidden = false
    const rect = row.getBoundingClientRect()
    const box = flyout.getBoundingClientRect()
    const left = rect.right + 8 + box.width < innerWidth ? rect.right + 8 : Math.max(8, rect.left - box.width - 8)
    flyout.style.left = `${left}px`
    flyout.style.top = `${Math.max(8, Math.min(rect.top, innerHeight - box.height - 8))}px`
}

function open(name) {
    state.selected = name
    state.code = null
    state.hovered = null
    $("#flyout").hidden = true
    render()
    $("#side").scrollTo(0, 0)
    graph.reveal(`m:${name}`)
}

function codeButton(where, marker) {
    const on = state.code?.file === where.file && state.code?.line === where.line
    return h("button", {
        type: "button",
        class: `btn${on ? " on" : ""}`,
        [marker]: true,
        onclick: () => {
            state.code = on ? null : where
            render()
        },
    }, on ? "Hide code" : "Show code")
}

function moduleView(module) {
    const expanded = { key: null }
    const streams = (title, list) =>
        h(
            "section",
            {},
            h("h4", {}, title),
            list.length === 0 && h("div", { class: "none" }, "none"),
            list.map((stream) => streamRow(module, stream, expanded)),
        )
    const rpcs = module.rpcs ?? []
    const skills = module.skills ?? []
    const known = module.doc !== undefined
    return h(
        "div",
        { class: "module-view", "data-bp-module-view": module.name },
        h("nav", {}, h("button", { type: "button", class: "btn", onclick: () => open(null), "data-bp-back": true }, "‹ Modules")),
        h(
            "div",
            { class: "title" },
            h(
                "div",
                { class: "names" },
                h("h3", { title: module.class }, className(module)),
                aliasOf(module) && h("span", { class: "alias" }, module.name),
            ),
            module.file && codeButton({ file: module.file, line: module.line ?? 1 }, "data-bp-show-code"),
        ),
        module.doc
            ? h("div", { class: "doc" }, module.doc)
            : h("div", { class: "none" }, known ? "No docstring." : "This dimos is too old to describe its modules."),
        h("div", { class: "io" }, streams("Inputs", module.streams.filter(reads)), streams("Outputs", module.streams.filter(writes))),
        skills.length > 0 && methods("Skills", skills, true),
        rpcs.length > 0 && methods("RPC methods", rpcs, false),
        known && skills.length + rpcs.length === 0 &&
            h("section", {}, h("h4", {}, "RPC methods"), h("div", { class: "none" }, "none of its own")),
    )
}

function streamRow(module, stream, expanded) {
    const topic = topicOf(stream)
    const box = h("div", { class: "stream", "--type": typeVar(stream.type) })
    const toggle = (key, button, make, spot) => {
        const next = expanded.key === key ? null : key
        expanded.key = next
        for (const other of document.querySelectorAll(".stream .ends")) {
            other.remove()
        }
        for (const other of document.querySelectorAll(".stream .row button.on")) {
            other.classList.remove("on")
        }
        if (next) {
            button.classList.add("on")
            box.append(make())
        }
        graph.setSpot(next && spot ? spot : { module: module.name })
    }
    const name = h("button", {
        type: "button",
        class: "sname",
        title: `who else is on /${topic} (lit in the graph)`,
        "data-bp-stream": stream.name,
        onclick: () => toggle(`topic:${topic}`, name, () => topicEnds(topic, module.name), { topic }),
    }, stream.name)
    const type = h("button", {
        type: "button",
        class: "stype",
        title: `${stream.type}: every module with one`,
        onclick: () => toggle(`type:${stream.type}`, type, () => typeUsers(stream.type, module.name)),
    }, typeName(stream.type))
    box.append(h("div", { class: "row" }, h("i", { class: "dot" }), name, type))
    return box
}

function ends(label, list, self) {
    return h(
        "div",
        { class: "end" },
        h("span", { class: "k" }, label),
        list.length === 0 && h("span", { class: "none" }, "nothing"),
        list.map((m) =>
            m.name === self
                ? h("span", { class: "self" }, className(m))
                : h("button", { type: "button", "data-bp-jump": m.name, onclick: () => open(m.name) }, className(m))
        ),
    )
}

function topicEnds(topic, self) {
    const on = (test) => state.modules.filter((m) => m.streams.some((s) => topicOf(s) === topic && test(s)))
    return h("div", { class: "ends" }, h("div", { class: "topic" }, `/${topic}`), ends("from", on(writes), self), ends("to", on(reads), self))
}

function typeUsers(type, self) {
    const users = (test) => state.modules.filter((m) => m.streams.some((s) => s.type === type && test(s)))
    return h("div", { class: "ends" }, h("div", { class: "topic" }, type), ends("out", users(writes), self), ends("in", users(reads), self))
}

function methods(title, list, skill) {
    const grid = h("div", { class: "cards" })
    const card = (method) => {
        const doc = (method.doc ?? "").trim()
        const returns = method.return_type ?? "None"
        const button = h("button", { type: "button", class: "card", "aria-expanded": "false", title: doc || undefined })
        const fill = (opened) => {
            button.classList.toggle("open", opened)
            button.setAttribute("aria-expanded", String(opened))
            const name = h("span", { class: "fn" })
            method.name.split(/(?<=_)/).forEach((part, i) => name.append(...(i ? [h("wbr"), part] : [part])))
            const params = method.params.length
                ? method.params.map((p) =>
                    `${p.name}${p.type ? `: ${p.type}` : ""}${opened && p.default !== null ? ` = ${p.default}` : ""}`
                ).join(", ")
                : "no params"
            button.replaceChildren(
                name,
                h("span", { class: "ps" }, params),
                ...(doc
                    ? [h("span", { class: "d" }, opened ? doc : doc.split(/\n\s*\n/)[0].replace(/\s*\n\s*/g, " "))]
                    : []),
                h("span", { class: "r", "--type": typeVar(returns) }, `→ ${typeName(returns)}`),
            )
        }
        button.addEventListener("click", () => {
            const opened = !button.classList.contains("open")
            for (const other of grid.querySelectorAll(".card.open")) {
                other.dispatchEvent(new CustomEvent("shut"))
            }
            fill(opened)
        })
        button.addEventListener("shut", () => fill(false))
        fill(false)
        return button
    }
    grid.append(...list.map(card))
    return h("section", { class: skill ? "skills" : "rpcs" }, h("h4", {}, title, h("span", { class: "n" }, String(list.length))), grid)
}

let shownCode = null
async function renderCode() {
    const pane = $("#code")
    const want = state.code
    if (!want) {
        pane.hidden = true
        shownCode = null
        return
    }
    if (shownCode && shownCode.file === want.file && shownCode.line === want.line) {
        return
    }
    shownCode = want
    pane.hidden = false
    const pre = h("pre")
    const status = h("div", { class: "opened", hidden: true })
    pane.replaceChildren(
        h(
            "div",
            { class: "code-head" },
            h("span", { class: "file", title: want.file }, want.file),
            h("span", { class: "at" }, `line ${want.line}`),
            h("span", { class: "spacer" }),
            FRAMED && h("button", {
                type: "button",
                class: "btn",
                "data-bp-open-editor": true,
                onclick: () => openInEditor(want.file, want.line, status),
            }, "Open in editor"),
            h("button", {
                type: "button",
                class: "btn",
                onclick: () => {
                    state.code = null
                    render()
                },
            }, "Hide code"),
        ),
        status,
        h("div", { class: "note" }, "loading…"),
        pre,
    )
    try {
        const { text } = await getJson(`source?file=${encodeURIComponent(want.file)}`)
        if (shownCode !== want) {
            return
        }
        pane.querySelector(".note").remove()
        highlightPython(text).forEach((parts, i) => {
            pre.append(h(
                "div",
                { "data-line": String(i + 1), class: i + 1 === want.line ? "at" : undefined },
                h("span", { class: "ln" }, String(i + 1)),
                h("span", { class: "tx" }, parts),
            ))
        })
        const target = pre.querySelector(`[data-line="${want.line}"]`)
        if (target) {
            pre.scrollTop = target.offsetTop - pre.offsetTop - 8
        }
    } catch (error) {
        pane.querySelector(".note").textContent = String(error.message ?? error)
    }
}

let editorStatus = null

function openInEditor(file, line, status) {
    editorStatus = status
    status.hidden = true
    parent.postMessage({ type: "dimos:open-in-editor", file, line }, location.origin)
}

addEventListener("message", (event) => {
    if (event.origin !== location.origin || event.source !== parent) {
        return
    }
    if (event.data?.type === "dimos:chrome-ok") {
        document.body.classList.add("chrome")
    } else if (event.data?.type === "dimos:open-in-editor-result") {
        const status = editorStatus
        if (status?.isConnected) {
            status.hidden = false
            status.className = `opened${event.data.ok ? "" : " failed"}`
            status.textContent = String(event.data.text ?? "")
            status.title = status.textContent
        }
    }
})

const PYTHON =
    /(#[^\n]*)|("""[\s\S]*?"""|'''[\s\S]*?'''|[rbfu]{0,2}"(?:\\.|[^"\\\n])*"|[rbfu]{0,2}'(?:\\.|[^'\\\n])*')|(@[\w.]+)|\b(\d[\d_.]*)\b|\b(def|class|return|if|elif|else|for|while|in|not|and|or|is|None|True|False|import|from|as|with|try|except|finally|raise|yield|async|await|lambda|pass|break|continue|self|global|nonlocal|assert|del)\b/g
const KINDS = ["com", "str", "dec", "num", "kw"]

function highlightPython(text) {
    const lines = [[]]
    const add = (piece, kind) => {
        piece.split("\n").forEach((part, i) => {
            if (i > 0) {
                lines.push([])
            }
            if (part) {
                lines[lines.length - 1].push(kind ? h("span", { class: kind }, part) : part)
            }
        })
    }
    let last = 0
    for (const match of text.matchAll(PYTHON)) {
        add(text.slice(last, match.index))
        add(match[0], KINDS[match.slice(1).findIndex((group) => group !== undefined)])
        last = match.index + match[0].length
    }
    add(text.slice(last))
    return lines
}

addEventListener("keydown", (event) => {
    if (event.key !== "Escape") {
        return
    }
    if (state.code) {
        state.code = null
        render()
    } else if (state.selected) {
        open(null)
    } else if (FRAMED) {
        parent.postMessage({ type: "dimos:close" }, location.origin)
    }
})

const extrasBox = $("#extras")
$("#extrasToggle").addEventListener("change", (event) => {
    state.showExtras = event.target.checked
    graph.setModel(state.modules ?? [], state.showExtras ? state.extras : [])
})

const logsRun = () => RUN ?? state.launch?.runId ?? null

function showLogs() {
    openLogs({ runId: logsRun(), title: NAME, h, getJson, openInEditor: FRAMED ? openInEditor : null })
}

async function pollRuns() {
    try {
        const { launch } = await getJson("runs")
        state.launch = launch?.blueprint === NAME ? launch : null
        state.running = !!state.launch && (launch.phase === "running" || launch.phase === "starting")
        if (state.relaunching === "starting" && state.launch?.pid !== state.relaunchedPid && state.launch?.phase !== "starting") {
            state.relaunching = false
        }
    } catch {
        state.running = false
    }
    renderBar()
    $("#liveBadge").hidden = !state.running
    if (!state.running) {
        graph.setRates(null)
        extrasBox.hidden = true
    }
}

async function pollRates() {
    let answer
    try {
        answer = await getJson("topics/rates")
    } catch (error) {
        state.ratesError = String(error.message ?? error)
        fillRates()
        return
    }
    state.ratesError = answer.up ? null : answer.error
    state.rateRows = answer.topics ?? []
    fillRates()
    if (!state.running || !state.modules) {
        return
    }
    const rates = new Map()
    for (const row of answer.topics ?? []) {
        const topic = String(row.topic).replace(/^\/+/, "")
        rates.set(topic, (rates.get(topic) ?? 0) + (row.hz ?? 0))
    }
    const wired = new Set(state.modules.flatMap((m) => m.streams.map(topicOf)))
    state.extras = [...rates.keys()].filter((topic) => !wired.has(topic)).sort().map((topic) => ({
        topic,
        type: (answer.topics.find((row) => String(row.topic).replace(/^\/+/, "") === topic)?.type ?? "").replace(/\//g, "."),
    }))
    $("#extrasCount").textContent = String(state.extras.length)
    extrasBox.hidden = state.extras.length === 0
    $("#extrasList").replaceChildren(
        ...state.extras.map(({ topic }) => h("li", {}, `/${topic}`, h("span", { class: "hz" }, `${(rates.get(topic) ?? 0).toFixed(1)} Hz`))),
    )
    if (state.showExtras) {
        graph.setModel(state.modules, state.extras)
    }
    graph.setRates(rates)
}

function launchedWithOther(launch, saved) {
    if (!launch?.overrides || !saved) {
        return false
    }
    const merge = (base, over = {}) => {
        const merged = { ...base }
        for (const [key, value] of Object.entries(over)) {
            if (value === null) {
                delete merged[key]
            } else {
                merged[key] = value
            }
        }
        return merged
    }
    const differs = (a, b, base = {}) =>
        [...new Set([...Object.keys(a), ...Object.keys(b)])].some((key) =>
            !same(key in a ? a[key] : base[key], key in b ? b[key] : base[key])
        )
    const oneOff = launch.oneOff ?? {}
    if (differs(merge(saved.global, oneOff.global), launch.overrides, saved.defaults)) {
        return true
    }
    const ran = launch.modules ?? {}
    const names = new Set([...Object.keys(saved.modules), ...Object.keys(oneOff.modules ?? {}), ...Object.keys(ran)])
    return [...names].some((name) => differs(merge(saved.modules[name] ?? {}, oneOff.modules?.[name]), ran[name] ?? {}))
}

async function loadSaved() {
    try {
        const [global, modules] = await Promise.all([getJson("global-config"), getJson(`blueprints/${encodeURIComponent(NAME)}/config`)])
        state.saved = { global: global.overrides ?? {}, modules: modules.overrides ?? {}, defaults: global.defaults ?? {} }
    } catch {
        state.saved = null
    }
    renderBar()
}

async function act(action) {
    state.actionError = null
    try {
        await action()
    } catch (error) {
        state.actionError = String(error.message ?? error)
    }
    await pollRuns()
}

const STEP_LABELS = {
    starting: "Starting dimOS",
    building: "Building the blueprint",
    starting_modules: "Starting modules",
    downloading_data: "Downloading the recording",
}

function startingStep() {
    const launch = state.launch
    if (!launch || state.relaunching && launch.pid === state.relaunchedPid) {
        return STEP_LABELS.starting
    }
    const download = (launch.output ?? "").split(/[\r\n]+/).filter((line) => line.includes("Downloading LFS objects")).at(-1)
    if (download && !/\b100%|\bdone\b/i.test(download)) {
        return STEP_LABELS.downloading_data
    }
    const step = (launch.steps ?? []).find((each) => each.state === "now")
    const deployed = step?.code === "starting_modules" ? step.data?.deployed : undefined
    const label = STEP_LABELS[step?.code] ?? STEP_LABELS.starting
    return typeof deployed === "number" && deployed ? `${label} (${deployed} started)` : label
}

function relaunchLabel(stale) {
    const phase = state.launch?.phase
    if (state.relaunching === "stopping" && state.launch?.pid === state.relaunchedPid || !state.relaunching && phase === "stopping") {
        return "Stopping…"
    }
    if (state.relaunching || phase === "starting") {
        return `${startingStep()}…`
    }
    return stale ? "Relaunch to apply" : "Relaunch"
}

function renderBar() {
    const launch = state.launch
    const phase = !launch ? "not running" : state.running ? launch.phase : launch.phase === "failed" ? "failed" : "last run"
    const stale = state.running && launchedWithOther(launch, state.saved)
    const busy = !!state.relaunching || launch?.phase === "starting" || launch?.phase === "stopping"
    const codeOn = state.source && state.code?.file === state.source.file && state.code?.line === state.source.line
    $("#bar").replaceChildren(...[
        h("strong", { class: "name" }, NAME),
        h("span", { class: `phase ${phase.replace(/ /g, "-")}` }, phase),
        state.actionError && h("span", { class: "failed", title: state.actionError }, state.actionError),
        h("span", { class: "spacer" }),
        launch && h("button", {
            type: "button",
            class: `btn relaunch${busy ? " busy" : launch.phase === "running" ? " live" : state.running ? "" : " primary"}${stale && !busy ? " stale" : ""}`,
            disabled: busy,
            "data-bp-relaunch": true,
            title: stale
                ? "the saved config changed since this run started"
                : state.running ? "stops this live run, then starts it again" : undefined,
            onclick: () => {
                state.relaunching = state.running ? "stopping" : "starting"
                state.relaunchedPid = state.launch?.pid
                renderBar()
                act(() => send("POST", "runs/restart").then(() => {
                    state.relaunching = "starting"
                }, (error) => {
                    state.relaunching = false
                    throw error
                }))
            },
        }, busy ? h("span", { class: "spinner", "aria-hidden": "true" }) : h("span", { "aria-hidden": "true" }, "↻ "), relaunchLabel(stale)),
        state.running && h("button", {
            type: "button",
            class: "btn",
            "data-bp-stop": true,
            onclick: () => act(() => send("POST", "runs/stop", launch.runId ? { runId: launch.runId } : {})),
        }, "Stop"),
        h("button", {
            type: "button",
            class: "btn",
            "data-bp-configure": true,
            onclick: () => openConfig({ name: NAME, h, getJson, send, onSaved: (saved) => {
                state.saved = saved
                renderBar()
            } }),
        }, "Configure"),
        state.source && h("button", {
            type: "button",
            class: `btn${codeOn ? " on" : ""}`,
            "data-bp-blueprint-code": true,
            onclick: () => {
                state.code = codeOn ? null : state.source
                render()
            },
        }, codeOn ? "Hide code" : "Show code"),
        h("button", {
            type: "button",
            class: "btn",
            disabled: !logsRun(),
            title: logsRun() ? `${NAME}'s log` : "no log yet",
            "data-bp-logs": true,
            onclick: showLogs,
        }, "Logs"),
        FRAMED && h("button", {
            type: "button",
            class: "close",
            "aria-label": "Close",
            onclick: () => parent.postMessage({ type: "dimos:close" }, location.origin),
        }, "✕"),
    ].filter(Boolean))
}

async function load() {
    document.title = `${NAME} · blueprint`
    render()
    const usage = getJson("catalog").then(moduleUsage, () => new Map())
    try {
        const detail = await getJson(`blueprints/${encodeURIComponent(NAME)}`)
        state.source = detail.file ? { file: detail.file, line: detail.line ?? 1 } : null
        state.modules = detail.modules
        await Promise.race([document.fonts.ready, new Promise((resolve) => setTimeout(resolve, 1500))])
        graph.setModel(state.modules)
        render()
        const uses = await usage
        state.modules = byRarity(state.modules, uses)
        render()
    } catch (error) {
        state.error = String(error.message ?? error)
        render()
    }
}

const moduleKey = (name) => name.toLowerCase().replace(/[^a-z0-9]/g, "")

export function moduleUsage(catalog) {
    const uses = new Map()
    for (const blueprint of catalog.blueprints ?? []) {
        for (const key of new Set((blueprint.modules ?? []).map(moduleKey))) {
            uses.set(key, (uses.get(key) ?? 0) + 1)
        }
    }
    return uses
}

export function byRarity(modules, uses) {
    const count = (module) => uses.get(moduleKey(module.name)) ?? 0
    return modules.map((module, index) => ({ module, index }))
        .sort((a, b) => count(a.module) - count(b.module) || a.index - b.index)
        .map(({ module }) => module)
}

if (FRAMED) {
    document.body.classList.add("framed")
    parent.postMessage({ type: "dimos:chrome" }, location.origin)
}
load()
loadSaved()
pollRuns().then(() => {
    if (VIEW === "logs" && logsRun()) {
        showLogs()
    }
})
pollRates()
let pollTick = 0
setInterval(() => {
    pollTick += 1
    if (state.relaunching || state.launch?.phase === "starting" || state.launch?.phase === "stopping" || pollTick % 3 === 0) {
        pollRuns()
    }
}, 1000)
setInterval(pollRates, 2000)
