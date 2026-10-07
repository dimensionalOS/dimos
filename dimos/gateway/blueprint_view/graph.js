// The module graph: SVG modules (cards) and topics (pills, tinted by message type), edges publisher → topic →
// reader. Laid out from the blueprint's wiring with each node's real size (layout.js), so nothing overlaps and no label
// is cut; any change to the node set (a live topic outside the blueprint appearing or going) lays the whole graph out
// again. Pan by dragging, zoom with the wheel; it fits the pane on load, on a new layout and on resize (until panned).

import { layout, LAYOUTS, overlaps } from "./layout.js"

const SVG = "http://www.w3.org/2000/svg"
const MODULE_H = 34
const TOPIC_H = 36
const PAD_X = 14
// a topic's second line may later read "<Type> · 999.9 Hz": its width is kept for that from the start
const RATE_RESERVE = " · 99.9 Hz"

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
    /** `pane`: the element the SVG fills. `on`: { module(name), hover(target|null) } */
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
            nodes.push({
                id: `m:${module.name}`,
                kind: "module",
                name: module.name,
                label,
                w: Math.ceil(textWidth(label, `600 12px ${sans}`)) + PAD_X * 2,
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
        for (const [topic, { type, extra }] of topics) {
            const second = typeName(type) + RATE_RESERVE
            const width = Math.max(
                textWidth(`/${topic}`, `500 11.5px ${mono}`),
                textWidth(second, `10.5px ${mono}`),
            )
            nodes.push({
                id: `t:${topic}`,
                kind: "topic",
                topic,
                type,
                extra,
                label: `/${topic}`,
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
        const box = this.pane.getBoundingClientRect()
        const { positions, routes, vertical } = layout(
            this.layoutId,
            this.nodes,
            this.edges,
            box.height > 40 ? box.width / (box.height - 100) : 1.5,
        )
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
                d: this.layoutId === "hierarchy"
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
            el("rect", { width: node.w, height: node.h }, g)
            if (node.kind === "module") {
                el("text", { x: PAD_X, y: node.h / 2, class: "name" }, g).textContent = node.label
            } else {
                g.style.setProperty("--type", `var(--bv-${typeColor(node.type)})`)
                el("text", { x: PAD_X, y: 14, class: "name" }, g).textContent = node.label
                node.sub = el("text", { x: PAD_X, y: 28, class: "sub" }, g)
                node.sub.textContent = typeName(node.type)
                const title = el("title", {}, g)
                title.textContent = `${node.label}\n${node.type}${node.extra ? "\n(not wired in this blueprint)" : ""}`
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
        const k = Math.min(
            1.4,
            (box.width - pad.x * 2) / (right - left || 1),
            (box.height - pad.top - pad.bottom) / (bottom - top || 1),
        )
        this.at = {
            k,
            x: pad.x + (box.width - pad.x * 2 - (right - left) * k) / 2 - left * k,
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

    applyView() {
        this.view.setAttribute("transform", `translate(${this.at.x} ${this.at.y}) scale(${this.at.k})`)
    }

    listen() {
        this.svg.addEventListener("wheel", (event) => {
            event.preventDefault()
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
            node.sub.textContent = typeName(node.type) + (hz ? ` · ${hz >= 100 ? hz.toFixed(0) : hz.toFixed(1)} Hz` : "")
        }
        for (const edge of this.edges) {
            edge.el?.classList.toggle("live", running && !!this.rates.get(edge.topic))
        }
    }
}

function side(node, after, vertical) {
    const sign = after ? 1 : -1
    return vertical ? { x: node.x, y: node.y + (sign * node.h) / 2 } : { x: node.x + (sign * node.w) / 2, y: node.y }
}

/** a curve along the flow (left to right, or top to bottom) from `from`'s far side through the bends to `to`'s near
 * side (a reversed edge: the sides facing each other) */
function flowPath(from, to, bends, vertical) {
    const forward = vertical ? from.y < to.y : from.x < to.x
    const points = [side(from, forward, vertical), ...bends, side(to, !forward, vertical)]
    let d = `M${points[0].x} ${points[0].y}`
    for (let i = 1; i < points.length; i++) {
        const a = points[i - 1]
        const b = points[i]
        if (vertical) {
            const dy = (b.y - a.y) / 2
            d += ` C${a.x} ${a.y + dy} ${b.x} ${b.y - dy} ${b.x} ${b.y}`
        } else {
            const dx = (b.x - a.x) / 2
            d += ` C${a.x + dx} ${a.y} ${b.x - dx} ${b.y} ${b.x} ${b.y}`
        }
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
