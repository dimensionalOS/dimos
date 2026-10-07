// The blueprint view (GET /dimos/blueprint_view?name=<blueprint>): the blueprint's modules, rarest first, beside its
// module graph. A module row shows its docstring's first line and, on hover, its streams; a click (in the list or the
// graph) drills into the module: docstring, typed streams linking to their topics' other ends, skills and RPC methods
// as cards, its code. Served by the dimos gateway; inside dimOS Desktop (same origin, /dimos/ proxied) it takes
// Desktop's theme, its live topic rates (GET /api/topics/rates) and its editor.
//
// postMessage to the parent window (Desktop's modal), on this page's origin:
//   {type: "dimos:open-in-editor", file, line}  → the parent opens it, answering {type: "dimos:open-in-editor-result",
//                                                  ok, text} (what it ran, or why it couldn't)
//   {type: "dimos:close"}                        → Escape with nothing left to step back from

import { className, Graph, LAYOUTS, topicOf, typeColor, typeName } from "./graph.js"

const NAME = new URLSearchParams(location.search).get("name") ?? ""
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
}

// ── theme: Desktop's skin (localStorage portal.theme on Desktop's origin), live ──
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
        // no storage: the Portal tokens
    }
}
applySkin()
addEventListener("storage", (event) => {
    if (event.key === null || event.key === "portal.theme" || event.key === "portal.corners") {
        applySkin()
        // another skin can mean other fonts, so other node sizes
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

async function getJson(url) {
    const response = await fetch(url)
    const data = await response.json().catch(() => null)
    if (!response.ok) {
        throw new Error(data?.error ?? `${response.status} ${response.statusText}`)
    }
    return data
}

const reads = (stream) => stream.direction !== "out"
const writes = (stream) => stream.direction !== "in"
const typeVar = (type) => `var(--bv-${typeColor(type)})`
const aliasOf = (module) => module.name !== className(module).toLowerCase() ? module.name : null

// ── the graph ──
const graph = new Graph($("#graph"), {
    module: (name) => open(name),
    hover: (target) => graph.setSpot(target ?? spotNow()),
})
const spotNow = () => {
    const name = state.hovered ?? state.selected
    return name ? { module: name } : null
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

// ── the side panel ──
function render() {
    const side = $("#side")
    const module = state.modules?.find((m) => m.name === state.selected)
    side.replaceChildren(module ? moduleView(module) : moduleList())
    graph.setSpot(spotNow())
    renderCode()
}

function moduleList() {
    const modules = state.modules
    return h(
        "div",
        { class: "list" },
        h(
            "div",
            { class: "head" },
            h("div", { class: "label" }, `Modules${modules ? ` · ${modules.length}` : ""}`),
            state.source && codeButton(state.source, "data-bp-blueprint-code"),
        ),
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
                        h("span", { class: "chev", "aria-hidden": "true" }, "›"),
                    ),
                )
            ),
        ),
    )
}

/** a hovered module's streams beside its row: Inputs then Outputs, pills tinted by their type */
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
            h("h3", { title: module.class }, className(module)),
            aliasOf(module) && h("span", { class: "alias" }, module.name),
        ),
        module.file && codeButton({ file: module.file, line: module.line ?? 1 }, "data-bp-show-code"),
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

/** a stream: its name (click: who else is on its topic, lit in the graph) and type (click: every module with one) */
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

/** RPC methods or skills as a grid of cards; a click opens one across the grid with its whole docstring and defaults */
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
            // a long name breaks after an underscore, not mid-word
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

// ── code, over the graph ──
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
                onclick: () => {
                    status.hidden = true
                    parent.postMessage({ type: "dimos:open-in-editor", file: want.file, line: want.line }, location.origin)
                },
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
    pane.status = status
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

addEventListener("message", (event) => {
    if (event.origin !== location.origin || event.source !== parent) {
        return
    }
    if (event.data?.type === "dimos:open-in-editor-result") {
        const status = $("#code").status
        if (status) {
            status.hidden = false
            status.className = `opened${event.data.ok ? "" : " failed"}`
            status.textContent = String(event.data.text ?? "")
            status.title = status.textContent
        }
    }
})

// Python, enough to read: comments, strings, decorators, numbers, keywords
const PYTHON =
    /(#[^\n]*)|("""[\s\S]*?"""|'''[\s\S]*?'''|[rbfu]{0,2}"(?:\\.|[^"\\\n])*"|[rbfu]{0,2}'(?:\\.|[^'\\\n])*')|(@[\w.]+)|\b(\d[\d_.]*)\b|\b(def|class|return|if|elif|else|for|while|in|not|and|or|is|None|True|False|import|from|as|with|try|except|finally|raise|yield|async|await|lambda|pass|break|continue|self|global|nonlocal|assert|del)\b/g
const KINDS = ["com", "str", "dec", "num", "kw"]

/** a file's lines, each a list of text and highlighted spans (a multi-line string spans lines) */
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

// Escape steps back: the code, then the module, then (framed) the modal
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

// ── live: while this blueprint runs, Desktop's topic rates annotate its topics ──
const extrasBox = $("#extras")
$("#extrasToggle").addEventListener("change", (event) => {
    state.showExtras = event.target.checked
    graph.setModel(state.modules ?? [], state.showExtras ? state.extras : [])
})

async function pollRuns() {
    try {
        const { launch } = await getJson("runs")
        state.running = launch?.blueprint === NAME && (launch.phase === "running" || launch.phase === "starting")
    } catch {
        state.running = false
    }
    $("#liveBadge").hidden = !state.running
    if (!state.running) {
        graph.setRates(null)
        extrasBox.hidden = true
    }
}

async function pollRates() {
    if (!state.running || !state.modules) {
        return
    }
    let answer
    try {
        answer = await getJson("../api/topics/rates")
    } catch {
        // not inside Desktop (or no rates there): the graph stays static
        return
    }
    const rates = new Map()
    for (const row of answer.topics ?? []) {
        const topic = String(row.topic).replace(/^\/+/, "")
        rates.set(topic, (rates.get(topic) ?? 0) + (row.hz ?? 0))
    }
    const wired = new Set(state.modules.flatMap((m) => m.streams.map(topicOf)))
    // topics on the bus that this blueprint doesn't wire (another run, a tool): listed apart, drawn only on request
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

// ── load ──
async function load() {
    document.title = `${NAME} · blueprint`
    render()
    const usage = getJson("catalog").then(moduleUsage, () => new Map())
    try {
        const detail = await getJson(`blueprints/${encodeURIComponent(NAME)}`)
        state.source = detail.file ? { file: detail.file, line: detail.line ?? 1 } : null
        state.modules = detail.modules
        // fonts first: node sizes are measured from the rendered text
        await Promise.race([document.fonts.ready, new Promise((resolve) => setTimeout(resolve, 1500))])
        graph.setModel(state.modules)
        render()
        // rarest first, once the catalog is in (it imports every blueprint: slow the first time)
        const uses = await usage
        state.modules = byRarity(state.modules, uses)
        render()
    } catch (error) {
        state.error = String(error.message ?? error)
        render()
    }
}

// The module list's order: the modules fewest blueprints use first, so what makes this blueprint itself leads
/** a module's name as both the catalog (rerun-bridge-module) and the blueprint (RerunBridgeModule) spell it */
const moduleKey = (name) => name.toLowerCase().replace(/[^a-z0-9]/g, "")

/** how many blueprints use each module (GET /dimos/catalog) */
export function moduleUsage(catalog) {
    const uses = new Map()
    for (const blueprint of catalog.blueprints ?? []) {
        for (const key of new Set((blueprint.modules ?? []).map(moduleKey))) {
            uses.set(key, (uses.get(key) ?? 0) + 1)
        }
    }
    return uses
}

/** the modules, the ones fewest blueprints use first (ties keep the blueprint's order) */
export function byRarity(modules, uses) {
    const count = (module) => uses.get(moduleKey(module.name)) ?? 0
    return modules.map((module, index) => ({ module, index }))
        .sort((a, b) => count(a.module) - count(b.module) || a.index - b.index)
        .map(({ module }) => module)
}

load()
pollRuns()
setInterval(pollRuns, 5000)
setInterval(pollRates, 2000)
