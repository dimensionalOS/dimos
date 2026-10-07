// The module graph: SVG modules (cards) and topics (pills, tinted by message type), edges publisher → topic →
// reader. Laid out from the blueprint's wiring with each node's real size (layout.js), so nothing overlaps and no label
// is cut; any change to the node set (a live topic outside the blueprint appearing or going) lays the whole graph out
// again. Pan by dragging or scrolling sideways, zoom with the wheel; it fits the pane on load, on a new layout and on
// resize (until panned), except that a wide Hierarchy keeps a readable size and scrolls instead of shrinking.

import { layout, LAYOUTS, overlaps } from "./layout.js"

const SVG = "http://www.w3.org/2000/svg"
// modules are the big cards (an accent bar on the left, "MOD", the name, a chevron), topics the small boxes between
// them (the name only; the type is on hover, the rate shows while the blueprint runs). Every box has a port on each side.
const MODULE_H = 46
const MODULE_BAR = 4
const MODULE_PAD_X = 16
const TAG_GAP = 10
const CHEV_W = 22
const TOPIC_H = 34
const PAD_X = 14
const PORT = 7
// a topic's second line may later read "999.9 Hz": its width is kept for that from the start
const RATE_RESERVE = "999.9 Hz"

const el = (name, attrs = {}, parent) => {
    const node = document.createElementNS(SVG, name)
    for (const [key, value] of Object.entries(attrs)) {
        node.setAttribute(key, value)
    }
    parent?.appendChild(node)
    return node
}

let measurer = null
function textWidth(text, font) {
    measurer ??= document.createElement("canvas").getContext("2d")
    measurer.font = font
    return measurer.measureText(text).width
}

export { LAYOUTS }

export class Graph {
    /** `pane`: the element the SVG fills. `on`: { module(name), topic(name|null), hover(target|null) } */
    constructor(pane, on) {
        this.pane = pane
        this.on = on
        // first in the pane: its title, controls and the code view sit over it
        this.svg = el("svg", { class: "graph", role: "img", "aria-label": "Module graph" })
        pane.prepend(this.svg)
        const defs = el("defs", {}, this.svg)
        const marker = el("marker", {
            id: "bv-arrow",
            viewBox: "0 0 10 10",
            refX: "9",
            refY: "5",
            markerWidth: "7",
            markerHeight: "7",
            orient: "auto-start-reverse",
        }, defs)
        el("path", { d: "M0 1 L10 5 L0 9 z", fill: "context-stroke" }, marker)
        this.view = el("g", {}, this.svg)
        this.edgeLayer = el("g", { class: "edges" }, this.view)
        this.nodeLayer = el("g", { class: "nodes" }, this.view)
        this.nodes = []
        this.edges = []
        this.layoutId = "hierarchy"
        this.at = { x: 0, y: 0, k: 1 }
        this.moved = false
        this.spot = null
        this.rates = new Map()
        this.listen()
        // a resized pane (until panned): Hierarchy may turn to fit it better, and the graph fits it again
        new ResizeObserver(() => !this.moved && this.nodes.length && this.relayout()).observe(pane)
    }

    /** the blueprint's modules (as GET /dimos/blueprints/{name} lists them) and, optionally, other live topics */
    setModel(modules, extraTopics = []) {
        const fontOf = (name) => getComputedStyle(this.pane).getPropertyValue(name).trim() || "sans-serif"
        const sans = fontOf("--sans")
        const mono = fontOf("--mono")
        const nodes = []
        const edges = []
        const topics = new Map()
        for (const module of modules) {
            const label = className(module)
            const tagW = Math.ceil(textWidth("MOD", `600 9.5px ${sans}`) * 1.25)
            nodes.push({
                id: `m:${module.name}`,
                kind: "module",
                name: module.name,
                label,
                tagW,
                w: MODULE_BAR + MODULE_PAD_X + tagW + TAG_GAP + Math.ceil(textWidth(label, `600 15px ${sans}`)) + CHEV_W,
                h: MODULE_H,
            })
            for (const stream of module.streams) {
                const topic = topicOf(stream)
                if (!topics.has(topic)) {
                    topics.set(topic, { type: stream.type, extra: false })
                }
                const reads = stream.direction !== "out"
                const writes = stream.direction !== "in"
                if (writes) {
                    edges.push({ from: `m:${module.name}`, to: `t:${topic}`, topic })
                }
                if (reads) {
                    edges.push({ from: `t:${topic}`, to: `m:${module.name}`, topic })
                }
            }
        }
        for (const extra of extraTopics) {
            if (!topics.has(extra.topic)) {
                topics.set(extra.topic, { type: extra.type ?? "", extra: true })
            }
        }
        // a topic both written and read here is connected; one with only a writer or only a reader is drawn dashed
        const written = new Set(edges.filter((e) => e.from.startsWith("m:")).map((e) => e.topic))
        const read = new Set(edges.filter((e) => e.to.startsWith("m:")).map((e) => e.topic))
        for (const [topic, { type, extra }] of topics) {
            const width = Math.max(textWidth(topic, `500 12px ${mono}`), textWidth(RATE_RESERVE, `10px ${mono}`))
            nodes.push({
                id: `t:${topic}`,
                kind: "topic",
                topic,
                type,
                extra,
                loose: !(written.has(topic) && read.has(topic)),
                label: topic,
                w: Math.ceil(width) + PAD_X * 2,
                h: TOPIC_H,
            })
        }
        const same = this.nodes.length === nodes.length &&
            this.nodes.every((node, i) => node.id === nodes[i].id && node.w === nodes[i].w)
        if (same) {
            return
        }
        this.nodes = nodes
        this.edges = edges
        this.relayout()
    }

    setLayout(id) {
        this.layoutId = id
        this.relayout()
    }

    relayout() {
        const { positions, routes, vertical } = layout(this.layoutId, this.nodes, this.edges)
        this.vertical = !!vertical
        for (const node of this.nodes) {
            Object.assign(node, positions.get(node.id))
        }
        this.routes = routes
        // a layout that still overlapped would be a bug: say so where the page's checks look
        this.svg.dataset.overlaps = String(overlaps(this.nodes, positions))
        this.draw()
        this.moved = false
        this.fit()
    }

    draw() {
        this.edgeLayer.replaceChildren()
        this.nodeLayer.replaceChildren()
        const byId = new Map(this.nodes.map((node) => [node.id, node]))
        this.edges.forEach((edge, index) => {
            const from = byId.get(edge.from)
            const to = byId.get(edge.to)
            const path = el("path", {
                class: "edge",
                d: ["hierarchy", "vertical"].includes(this.layoutId)
                    ? flowPath(from, to, this.routes.get(index) ?? [], this.vertical)
                    : straightPath(from, to),
                "marker-end": "url(#bv-arrow)",
            }, this.edgeLayer)
            path.style.setProperty("--type", `var(--bv-${typeColor(byId.get(`t:${edge.topic}`)?.type ?? "")})`)
            edge.el = path
        })
        for (const node of this.nodes) {
            const g = el("g", {
                class: `node ${node.kind}${node.extra ? " extra" : ""}`,
                transform: `translate(${node.x - node.w / 2} ${node.y - node.h / 2})`,
                tabindex: node.kind === "module" ? "0" : "-1",
                "data-node": node.id,
            }, this.nodeLayer)
            el("rect", { class: "box", width: node.w, height: node.h }, g)
            const title = el("title", {}, g)
            if (node.kind === "module") {
                el("rect", { class: "bar", width: MODULE_BAR, height: node.h }, g)
                const x = MODULE_BAR + MODULE_PAD_X
                el("text", { x, y: node.h / 2, class: "tag" }, g).textContent = "MOD"
                el("text", { x: x + node.tagW + TAG_GAP, y: node.h / 2, class: "name" }, g).textContent = node.label
                el("text", { x: node.w - CHEV_W / 2 - 2, y: node.h / 2, class: "more" }, g).textContent = "›"
                title.textContent = node.name
            } else {
                g.style.setProperty("--type", `var(--bv-${typeColor(node.type)})`)
                if (node.loose) {
                    g.classList.add("loose")
                }
                node.nameEl = el("text", { x: PAD_X, y: node.h / 2, class: "name" }, g)
                node.nameEl.textContent = node.label
                node.sub = el("text", { x: PAD_X, y: 25, class: "sub" }, g)
                title.textContent = `/${node.topic}\n${node.type}${node.extra ? "\n(not wired in this blueprint)" : ""}` +
                    (node.loose ? "\n(only written or only read here)" : "")
            }
            // a port on each side, where edges leave (right) and arrive (left)
            for (const x of [0, node.w]) {
                el("rect", { class: "port", x: x - PORT / 2, y: node.h / 2 - PORT / 2, width: PORT, height: PORT }, g)
            }
            node.el = g
        }
        this.applyRates()
        this.applySpot()
    }

    // ── view ──
    fit() {
        const box = this.pane.getBoundingClientRect()
        if (!this.nodes.length || box.width < 10 || box.height < 10) {
            return
        }
        const left = Math.min(...this.nodes.map((n) => n.x - n.w / 2))
        const right = Math.max(...this.nodes.map((n) => n.x + n.w / 2))
        const top = Math.min(...this.nodes.map((n) => n.y - n.h / 2))
        const bottom = Math.max(...this.nodes.map((n) => n.y + n.h / 2))
        // room for the title above and the layout tabs below
        const pad = { x: 24, top: 52, bottom: 60 }
        const fitHeight = Math.min(1.4, (box.height - pad.top - pad.bottom) / (bottom - top || 1))
        let k = Math.min(fitHeight, (box.width - pad.x * 2) / (right - left || 1))
        // a wide Hierarchy stays readable and scrolls sideways (from its left end) instead of shrinking to fit
        if (this.layoutId === "hierarchy") {
            k = Math.max(k, Math.min(fitHeight, 0.8))
        }
        const spare = box.width - pad.x * 2 - (right - left) * k
        this.at = {
            k,
            x: pad.x + Math.max(spare, 0) / 2 - left * k,
            y: pad.top + (box.height - pad.top - pad.bottom - (bottom - top) * k) / 2 - top * k,
        }
        this.applyView()
    }

    zoom(factor, cx, cy) {
        const box = this.pane.getBoundingClientRect()
        cx ??= box.width / 2
        cy ??= box.height / 2
        const k = Math.max(0.1, Math.min(4, this.at.k * factor))
        this.at = { k, x: cx - ((cx - this.at.x) * k) / this.at.k, y: cy - ((cy - this.at.y) * k) / this.at.k }
        this.moved = true
        this.applyView()
    }

    /** pan (keeping the zoom) so node `id` is in view, if it isn't already: a module picked in the side panel */
    reveal(id) {
        const node = this.nodes.find((n) => n.id === id)
        const box = this.pane.getBoundingClientRect()
        if (!node || box.width < 10) {
            return
        }
        const { k } = this.at
        // room for the title above and the layout tabs below, as in fit()
        const margin = { x: 40, top: 60, bottom: 70 }
        const left = this.at.x + (node.x - node.w / 2) * k
        const right = this.at.x + (node.x + node.w / 2) * k
        const top = this.at.y + (node.y - node.h / 2) * k
        const bottom = this.at.y + (node.y + node.h / 2) * k
        if (left >= margin.x && right <= box.width - margin.x && top >= margin.top && bottom <= box.height - margin.bottom) {
            return
        }
        const from = { ...this.at }
        const to = { k, x: box.width / 2 - node.x * k, y: (box.height + margin.top - margin.bottom) / 2 - node.y * k }
        const start = performance.now()
        const step = (now) => {
            const t = Math.min((now - start) / 260, 1)
            const ease = 1 - (1 - t) ** 3
            this.at = { k, x: from.x + (to.x - from.x) * ease, y: from.y + (to.y - from.y) * ease }
            this.applyView()
            if (t < 1) {
                requestAnimationFrame(step)
            }
        }
        this.moved = true
        requestAnimationFrame(step)
    }

    applyView() {
        this.view.setAttribute("transform", `translate(${this.at.x} ${this.at.y}) scale(${this.at.k})`)
    }

    listen() {
        this.svg.addEventListener("wheel", (event) => {
            event.preventDefault()
            // sideways (a trackpad swipe, or shift + wheel) scrolls; up and down zooms
            const sideways = event.shiftKey ? event.deltaY || event.deltaX : event.deltaX
            if (Math.abs(sideways) > Math.abs(event.shiftKey ? 0 : event.deltaY)) {
                this.at = { ...this.at, x: this.at.x - sideways }
                this.moved = true
                this.applyView()
                return
            }
            const box = this.pane.getBoundingClientRect()
            this.zoom(Math.exp(-event.deltaY * 0.0016), event.clientX - box.left, event.clientY - box.top)
        }, { passive: false })
        let drag = null
        this.svg.addEventListener("pointerdown", (event) => {
            drag = { x: event.clientX, y: event.clientY, at: { ...this.at }, far: false }
        })
        addEventListener("pointermove", (event) => {
            if (!drag) {
                return
            }
            const dx = event.clientX - drag.x
            const dy = event.clientY - drag.y
            if (!drag.far && Math.hypot(dx, dy) < 4) {
                return
            }
            drag.far = true
            this.svg.classList.add("panning")
            this.at = { ...drag.at, x: drag.at.x + dx, y: drag.at.y + dy }
            this.moved = true
            this.applyView()
        })
        addEventListener("pointerup", (event) => {
            const was = drag
            drag = null
            this.svg.classList.remove("panning")
            if (was && !was.far) {
                const node = this.nodeAt(event.target)
                if (node?.kind === "module") {
                    this.on.module(node.name)
                } else if (event.target.closest?.("svg") === this.svg) {
                    // a topic stays lit once clicked (as if hovered); clicking empty space lets it go
                    this.on.topic?.(node?.kind === "topic" ? node.topic : null)
                }
            }
        })
        this.svg.addEventListener("pointerover", (event) => {
            const node = this.nodeAt(event.target)
            this.on.hover(node ? (node.kind === "module" ? { module: node.name } : { topic: node.topic }) : null)
        })
        this.svg.addEventListener("pointerleave", () => this.on.hover(null))
        this.svg.addEventListener("keydown", (event) => {
            const node = this.nodeAt(event.target)
            if (event.key === "Enter" && node?.kind === "module") {
                this.on.module(node.name)
            }
        })
    }

    nodeAt(target) {
        const g = target?.closest?.("[data-node]")
        return g ? this.nodes.find((node) => node.id === g.dataset.node) : null
    }

    // ── what's lit and what's live ──
    /** light a module (it, its topics and the modules on them) or a topic (it and its ends); null: nothing */
    setSpot(target) {
        this.spot = target
        this.applySpot()
    }

    applySpot() {
        const target = this.spot
        const lit = new Set()
        if (target?.module) {
            const id = `m:${target.module}`
            lit.add(id)
            for (const edge of this.edges) {
                if (edge.from === id || edge.to === id) {
                    lit.add(edge.from === id ? edge.to : edge.from)
                }
            }
        } else if (target?.topic) {
            const id = `t:${target.topic}`
            lit.add(id)
            for (const edge of this.edges) {
                if (edge.from === id || edge.to === id) {
                    lit.add(edge.from === id ? edge.to : edge.from)
                }
            }
        }
        this.svg.classList.toggle("spot", lit.size > 0)
        for (const node of this.nodes) {
            node.el?.classList.toggle("lit", lit.has(node.id))
        }
        const focus = target?.module ? `m:${target.module}` : target?.topic ? `t:${target.topic}` : null
        for (const edge of this.edges) {
            edge.el?.classList.toggle("lit", edge.from === focus || edge.to === focus)
        }
    }

    /** topic → Hz (null: not running, so no live marks at all) */
    setRates(rates) {
        this.rates = rates
        this.applyRates()
    }

    applyRates() {
        const running = this.rates !== null
        this.svg.classList.toggle("running", running)
        for (const node of this.nodes) {
            if (node.kind !== "topic" || !node.el) {
                continue
            }
            const hz = running ? this.rates.get(node.topic) ?? 0 : null
            node.el.classList.toggle("live", !!hz)
            // the name sits in the middle, or above the rate while it has one
            node.nameEl.setAttribute("y", hz ? 13 : node.h / 2)
            node.sub.textContent = hz ? `${hz >= 100 ? hz.toFixed(0) : hz.toFixed(1)} Hz` : ""
        }
        for (const edge of this.edges) {
            edge.el?.classList.toggle("live", running && !!this.rates.get(edge.topic))
        }
    }
}

/** where an edge leaves (after) or arrives at (before) a node: just outside its port */
function side(node, after, vertical) {
    const sign = after ? 1 : -1
    const reach = (vertical ? node.h : node.w) / 2 + PORT / 2
    return vertical ? { x: node.x, y: node.y + sign * reach } : { x: node.x + sign * reach, y: node.y }
}

/** an edge along the flow (left to right, or top to bottom), drawn like dot: it leaves `from` on its far side (right,
 * or bottom) and arrives at `to` on its near side (left, or top). A long edge (it has bends: one per layer it crosses,
 * all on one track) arcs onto its track, runs straight along it, and arcs off into `to`; a feedback edge does the same
 * the other way. An edge between neighbouring layers is one S-curve. */
function flowPath(from, to, bends, vertical) {
    // work in (u along the flow, v across it), then map back
    const uv = (p) => (vertical ? { u: p.y, v: p.x } : { u: p.x, v: p.y })
    const xy = (u, v) => (vertical ? `${v} ${u}` : `${u} ${v}`)
    const a = uv(side(from, true, vertical))
    const b = uv(side(to, false, vertical))
    if (!bends.length) {
        const half = Math.max(Math.abs(b.u - a.u) / 2, 30)
        return `M${xy(a.u, a.v)} C${xy(a.u + half, a.v)} ${xy(b.u - half, b.v)} ${xy(b.u, b.v)}`
    }
    const track = bends.map((p) => uv(p).v).sort((m, n) => m - n)[Math.floor(bends.length / 2)]
    const forward = b.u > a.u
    // how far along the flow each end's arc takes to reach the track
    const span = Math.abs(b.u - a.u)
    const r = Math.max(32, Math.min(110, forward ? span / 3 : 80))
    let d = `M${xy(a.u, a.v)}`
    if (forward) {
        // arc onto the track, straight along it, arc off into `to`
        d += ` C${xy(a.u + r / 2, a.v)} ${xy(a.u + r / 2, track)} ${xy(a.u + r, track)}`
        d += ` L${xy(b.u - r, track)}`
        d += ` C${xy(b.u - r / 2, track)} ${xy(b.u - r / 2, b.v)} ${xy(b.u, b.v)}`
    } else {
        // feedback: out of `from`'s far side and round onto the track, back along it, round into `to`'s near side
        d += ` C${xy(a.u + r, a.v)} ${xy(a.u + r, track)} ${xy(a.u, track)}`
        d += ` L${xy(b.u, track)}`
        d += ` C${xy(b.u - r, track)} ${xy(b.u - r, b.v)} ${xy(b.u, b.v)}`
    }
    return d
}

/** where the line between two centers leaves a box */
function exit(node, toward) {
    const dx = toward.x - node.x
    const dy = toward.y - node.y
    const scale = 1 / Math.max(Math.abs(dx) / (node.w / 2) || 0, Math.abs(dy) / (node.h / 2) || 0, 1e-6)
    return { x: node.x + dx * Math.min(scale, 1), y: node.y + dy * Math.min(scale, 1) }
}

function straightPath(from, to) {
    const a = exit(from, to)
    const b = exit(to, from)
    return `M${a.x} ${a.y} L${b.x} ${b.y}`
}

/** a module's class name, capitalized as written (`dimos.robot.GO2Connection` → `GO2Connection`) */
export const className = (module) => module.class.split(".").pop() || module.name
/** a message type's own name (`dimos.msgs.nav_msgs.Path.Path` → `Path`) */
export const typeName = (type) => (type ?? "").split(".").pop() || type || ""
/** the topic a stream rides: its wired topic (newer dimos) else its name, no leading slash */
export const topicOf = (stream) => (stream.topic ?? stream.name).replace(/^\/+/, "")

// a type's color: by type, else by its *_msgs package, else hashed (the names are view.css's --bv-<color>)
const TYPE_COLORS = {
    Image: "warn",
    CompressedImage: "warn",
    CameraInfo: "warn",
    PointCloud2: "violet",
    LaserScan: "violet",
    OccupancyGrid: "cat-1",
    Path: "cat-1",
    Odometry: "info",
    PoseStamped: "info",
    Twist: "info",
    TFMessage: "ok",
    Bool: "cat-2",
    String: "cat-2",
}
const PACKAGE_COLORS = {
    sensor_msgs: "warn",
    geometry_msgs: "info",
    nav_msgs: "cat-1",
    tf2_msgs: "ok",
    vision_msgs: "violet",
    std_msgs: "cat-2",
}
const PALETTE = ["info", "warn", "cat-1", "violet", "ok", "cat-2", "cat-3", "cat-4"]
export function typeColor(type) {
    const pkg = (type ?? "").split(".").find((part) => part.endsWith("_msgs")) ?? ""
    return TYPE_COLORS[typeName(type)] ?? PACKAGE_COLORS[pkg] ??
        PALETTE[[...(type ?? "")].reduce((sum, c) => sum + c.charCodeAt(0), 0) % PALETTE.length]
}
