import { layout, LAYOUTS, overlaps } from "./layout.js"

const SVG = "http://www.w3.org/2000/svg"
const MODULE_H = 46
const MODULE_BAR = 4
const MODULE_PAD_X = 16
const TAG_GAP = 10
const CHEV_W = 22
const TOPIC_H = 30
const PAD_X = 9
const PORT = 7
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
    constructor(pane, on) {
        this.pane = pane
        this.on = on
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
        new ResizeObserver(() => !this.moved && this.nodes.length && this.relayout()).observe(pane)
    }

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
                node.sub = el("text", { x: PAD_X, y: 22, class: "sub" }, g)
                title.textContent = `/${node.topic}\n${node.type}${node.extra ? "\n(not wired in this blueprint)" : ""}` +
                    (node.loose ? "\n(only written or only read here)" : "")
            }
            for (const x of [0, node.w]) {
                el("rect", { class: "port", x: x - PORT / 2, y: node.h / 2 - PORT / 2, width: PORT, height: PORT }, g)
            }
            node.el = g
        }
        this.applyRates()
        this.applySpot()
    }

    fit() {
        const box = this.pane.getBoundingClientRect()
        if (!this.nodes.length || box.width < 10 || box.height < 10) {
            return
        }
        const left = Math.min(...this.nodes.map((n) => n.x - n.w / 2))
        const right = Math.max(...this.nodes.map((n) => n.x + n.w / 2))
        const top = Math.min(...this.nodes.map((n) => n.y - n.h / 2))
        const bottom = Math.max(...this.nodes.map((n) => n.y + n.h / 2))
        const pad = { x: 24, top: 52, bottom: 60 }
        const fitHeight = Math.min(1.4, (box.height - pad.top - pad.bottom) / (bottom - top || 1))
        let k = Math.min(fitHeight, (box.width - pad.x * 2) / (right - left || 1))
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

    reveal(id) {
        const node = this.nodes.find((n) => n.id === id)
        const box = this.pane.getBoundingClientRect()
        if (!node || box.width < 10) {
            return
        }
        const { k } = this.at
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
        const grid = 22 * this.at.k
        this.pane.style.backgroundSize = `${grid}px ${grid}px`
        this.pane.style.backgroundPosition = `${this.at.x}px ${this.at.y}px`
    }

    listen() {
        this.svg.addEventListener("wheel", (event) => {
            event.preventDefault()
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
            node.nameEl.setAttribute("y", hz ? 11 : node.h / 2)
            node.sub.textContent = hz ? `${hz >= 100 ? hz.toFixed(0) : hz.toFixed(1)} Hz` : ""
        }
        for (const edge of this.edges) {
            edge.el?.classList.toggle("live", running && !!this.rates.get(edge.topic))
        }
    }
}

function side(node, after, vertical) {
    const sign = after ? 1 : -1
    const reach = (vertical ? node.h : node.w) / 2 + PORT / 2
    return vertical ? { x: node.x, y: node.y + sign * reach } : { x: node.x + sign * reach, y: node.y }
}

function flowPath(from, to, bends, vertical) {
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
    const span = Math.abs(b.u - a.u)
    const r = Math.max(32, Math.min(110, forward ? span / 3 : 80))
    let d = `M${xy(a.u, a.v)}`
    if (forward) {
        d += ` C${xy(a.u + r / 2, a.v)} ${xy(a.u + r / 2, track)} ${xy(a.u + r, track)}`
        d += ` L${xy(b.u - r, track)}`
        d += ` C${xy(b.u - r / 2, track)} ${xy(b.u - r / 2, b.v)} ${xy(b.u, b.v)}`
    } else {
        d += ` C${xy(a.u + r, a.v)} ${xy(a.u + r, track)} ${xy(a.u, track)}`
        d += ` L${xy(b.u, track)}`
        d += ` C${xy(b.u - r, track)} ${xy(b.u - r, b.v)} ${xy(b.u, b.v)}`
    }
    return d
}

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

export const className = (module) => module.class.split(".").pop() || module.name
export const typeName = (type) => (type ?? "").split(".").pop() || type || ""
export const topicOf = (stream) => (stream.topic ?? stream.name).replace(/^\/+/, "")

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
