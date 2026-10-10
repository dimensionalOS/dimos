const LAYER_GAP = 90
const NODE_GAP = 22
const MARGIN = 10

export const LAYOUTS = [
    { id: "hierarchy", label: "Hierarchy" },
    { id: "vertical", label: "Vertical" },
    { id: "force", label: "Force" },
    { id: "radial", label: "Radial" },
    { id: "circular", label: "Circular" },
]

export function layout(id, nodes, edges) {
    if (id === "force") {
        return { positions: separate(nodes, force(nodes, edges)), routes: new Map() }
    }
    if (id === "radial") {
        return { positions: separate(nodes, radial(nodes, edges)), routes: new Map() }
    }
    if (id === "circular") {
        return { positions: separate(nodes, circular(nodes, edges)), routes: new Map() }
    }
    if (id === "vertical") {
        const down = layered(nodes.map((node) => ({ ...node, w: node.h, h: node.w })), edges)
        const flip = (point) => ({ x: point.y, y: point.x })
        down.positions = new Map([...down.positions].map(([id, point]) => [id, flip(point)]))
        down.routes = new Map([...down.routes].map(([index, points]) => [index, points.map(flip)]))
        down.vertical = true
        return down
    }
    return layered(nodes, edges)
}

export function layered(nodes, edges) {
    const byId = new Map(nodes.map((node) => [node.id, node]))
    const live = edges.map((edge, index) => ({ ...edge, index })).filter((edge) =>
        byId.has(edge.from) && byId.has(edge.to) && edge.from !== edge.to
    )
    const state = new Map()
    const outs = new Map(nodes.map((node) => [node.id, []]))
    for (const edge of live) {
        outs.get(edge.from).push(edge)
    }
    const visit = (id) => {
        state.set(id, 1)
        for (const edge of outs.get(id)) {
            const seen = state.get(edge.to)
            if (seen === 1) {
                edge.reversed = true
            } else if (!seen) {
                visit(edge.to)
            }
        }
        state.set(id, 2)
    }
    for (const node of nodes) {
        if (!state.get(node.id)) {
            visit(node.id)
        }
    }
    const dag = live.map((edge) => edge.reversed ? { ...edge, from: edge.to, to: edge.from } : edge)
    const layer = new Map(nodes.map((node) => [node.id, 0]))
    const preds = new Map(nodes.map((node) => [node.id, []]))
    const succs = new Map(nodes.map((node) => [node.id, []]))
    for (const edge of dag) {
        preds.get(edge.to).push(edge.from)
        succs.get(edge.from).push(edge.to)
    }
    const indegree = new Map(nodes.map((node) => [node.id, preds.get(node.id).length]))
    const queue = nodes.filter((node) => indegree.get(node.id) === 0).map((node) => node.id)
    while (queue.length) {
        const id = queue.shift()
        for (const next of succs.get(id)) {
            layer.set(next, Math.max(layer.get(next), layer.get(id) + 1))
            indegree.set(next, indegree.get(next) - 1)
            if (indegree.get(next) === 0) {
                queue.push(next)
            }
        }
    }
    for (const node of nodes) {
        const after = succs.get(node.id)
        if (preds.get(node.id).length === 0 && after.length) {
            layer.set(node.id, Math.max(0, Math.min(...after.map((id) => layer.get(id))) - 1))
        }
    }
    const dims = new Map(nodes.map((node) => [node.id, { w: node.w, h: node.h }]))
    const links = []
    const chains = new Map()
    for (const edge of dag) {
        let from = edge.from
        const chain = []
        for (let at = layer.get(edge.from) + 1; at < layer.get(edge.to); at++) {
            const dummy = `\u0000${edge.index}:${at}`
            layer.set(dummy, at)
            dims.set(dummy, { w: 0, h: 4 })
            links.push([from, dummy])
            chain.push(dummy)
            from = dummy
        }
        links.push([from, edge.to])
        chains.set(edge.index, { chain, reversed: !!edge.reversed })
    }
    const count = Math.max(0, ...layer.values()) + 1
    const layers = Array.from({ length: count }, () => [])
    for (const [id, at] of layer) {
        layers[at].push(id)
    }
    const up = new Map([...layer.keys()].map((id) => [id, []]))
    const down = new Map([...layer.keys()].map((id) => [id, []]))
    for (const [from, to] of links) {
        down.get(from).push(to)
        up.get(to).push(from)
    }
    const position = new Map()
    const index = () => layers.forEach((ids) => ids.forEach((id, i) => position.set(id, i)))
    index()
    const crossings = () => {
        let total = 0
        for (let at = 0; at + 1 < count; at++) {
            const pairs = []
            for (const id of layers[at]) {
                for (const to of down.get(id)) {
                    pairs.push([position.get(id), position.get(to)])
                }
            }
            for (let i = 0; i < pairs.length; i++) {
                for (let j = i + 1; j < pairs.length; j++) {
                    if ((pairs[i][0] - pairs[j][0]) * (pairs[i][1] - pairs[j][1]) < 0) {
                        total++
                    }
                }
            }
        }
        return total
    }
    let best = layers.map((ids) => [...ids])
    let fewest = crossings()
    for (let sweep = 0; sweep < 16 && fewest > 0; sweep++) {
        const downward = sweep % 2 === 0
        const order = downward ? [...layers.keys()] : [...layers.keys()].reverse()
        for (const at of order) {
            const near = downward ? up : down
            const centre = (id) => {
                const others = near.get(id)
                return others.length
                    ? others.reduce((sum, other) => sum + position.get(other), 0) / others.length
                    : position.get(id)
            }
            const keyed = layers[at].map((id) => [id, centre(id)])
            keyed.sort((a, b) => a[1] - b[1])
            layers[at] = keyed.map(([id]) => id)
            layers[at].forEach((id, i) => position.set(id, i))
        }
        const now = crossings()
        if (now < fewest) {
            fewest = now
            best = layers.map((ids) => [...ids])
        }
    }
    best.forEach((ids, at) => (layers[at] = ids))
    index()
    const widths = layers.map((ids) => Math.max(0, ...ids.map((id) => dims.get(id).w)))
    const columnX = []
    let x = 0
    for (let at = 0; at < count; at++) {
        columnX.push(x + widths[at] / 2)
        x += widths[at] + LAYER_GAP
    }
    const y = new Map()
    for (const ids of layers) {
        let top = 0
        for (const id of ids) {
            const h = dims.get(id).h
            y.set(id, top + h / 2)
            top += h + NODE_GAP
        }
    }
    const tallest = Math.max(0, ...layers.map((ids) => {
        const last = ids[ids.length - 1]
        return last === undefined ? 0 : y.get(last) + dims.get(last).h / 2
    }))
    for (const ids of layers) {
        const last = ids[ids.length - 1]
        const shift = last === undefined ? 0 : (tallest - (y.get(last) + dims.get(last).h / 2)) / 2
        ids.forEach((id) => y.set(id, y.get(id) + shift))
    }
    for (let pass = 0; pass < 24; pass++) {
        const downward = pass % 2 === 0
        const order = downward ? [...layers.keys()] : [...layers.keys()].reverse()
        for (const at of order) {
            const ids = layers[at]
            const wanted = ids.map((id) => {
                const others = [...up.get(id), ...down.get(id)]
                return others.length ? others.reduce((sum, other) => sum + y.get(other), 0) / others.length : y.get(id)
            })
            placeInOrder(ids, wanted, ids.map((id) => dims.get(id).h)).forEach((value, i) => y.set(ids[i], value))
        }
    }
    const chainOf = new Map()
    for (const { chain } of chains.values()) {
        if (chain.length > 1) {
            chain.forEach((id) => chainOf.set(id, chain))
        }
    }
    const median = (values) => {
        const sorted = [...values].sort((a, b) => a - b)
        return sorted[Math.floor(sorted.length / 2)]
    }
    for (let pass = 0; pass < 8; pass++) {
        const track = new Map()
        for (const chain of new Set(chainOf.values())) {
            track.set(chain, median(chain.map((id) => y.get(id))))
        }
        for (const ids of layers) {
            const wanted = ids.map((id) => chainOf.has(id) ? track.get(chainOf.get(id)) : y.get(id))
            placeInOrder(ids, wanted, ids.map((id) => dims.get(id).h)).forEach((value, i) => y.set(ids[i], value))
        }
    }
    const positions = new Map()
    for (const node of nodes) {
        positions.set(node.id, { x: columnX[layer.get(node.id)], y: y.get(node.id) })
    }
    const routes = new Map()
    for (const [edgeIndex, { chain, reversed }] of chains) {
        const points = chain.map((id) => ({ x: columnX[layer.get(id)], y: y.get(id) }))
        routes.set(edgeIndex, reversed ? points.reverse() : points)
    }
    return { positions, routes }
}

export function placeInOrder(ids, wanted, heights) {
    const offsets = [0]
    for (let i = 1; i < ids.length; i++) {
        offsets.push(offsets[i - 1] + heights[i - 1] / 2 + NODE_GAP + heights[i] / 2)
    }
    const blocks = []
    wanted.forEach((value, i) => {
        blocks.push({ sum: value - offsets[i], n: 1 })
        while (blocks.length > 1) {
            const last = blocks[blocks.length - 1]
            const before = blocks[blocks.length - 2]
            if (before.sum / before.n <= last.sum / last.n) {
                break
            }
            before.sum += last.sum
            before.n += last.n
            blocks.pop()
        }
    })
    const out = []
    for (const block of blocks) {
        for (let k = 0; k < block.n; k++) {
            out.push(block.sum / block.n + offsets[out.length])
        }
    }
    return out
}

function force(nodes, edges) {
    const start = circular(nodes, edges)
    const p = new Map([...start].map(([id, at]) => [id, { x: at.x, y: at.y }]))
    const size = new Map(nodes.map((node) => [node.id, Math.hypot(node.w, node.h) / 2]))
    const ids = nodes.map((node) => node.id)
    const linked = edges.filter((edge) => p.has(edge.from) && p.has(edge.to) && edge.from !== edge.to)
    for (let step = 0, heat = 40; step < 400; step++, heat *= 0.99) {
        const move = new Map(ids.map((id) => [id, { x: 0, y: 0 }]))
        for (let i = 0; i < ids.length; i++) {
            for (let j = i + 1; j < ids.length; j++) {
                const a = p.get(ids[i])
                const b = p.get(ids[j])
                let dx = a.x - b.x
                let dy = a.y - b.y
                const d = Math.hypot(dx, dy) || 0.01
                const room = size.get(ids[i]) + size.get(ids[j]) + 10
                const push = (room * room) / d / 16
                dx /= d
                dy /= d
                move.get(ids[i]).x += dx * push
                move.get(ids[i]).y += dy * push
                move.get(ids[j]).x -= dx * push
                move.get(ids[j]).y -= dy * push
            }
        }
        for (const edge of linked) {
            const a = p.get(edge.from)
            const b = p.get(edge.to)
            const d = Math.hypot(a.x - b.x, a.y - b.y) || 0.01
            const rest = size.get(edge.from) + size.get(edge.to) + 16
            const pull = (d - rest) * 0.2
            const ux = (b.x - a.x) / d
            const uy = (b.y - a.y) / d
            move.get(edge.from).x += ux * pull
            move.get(edge.from).y += uy * pull
            move.get(edge.to).x -= ux * pull
            move.get(edge.to).y -= uy * pull
        }
        for (const id of ids) {
            const m = move.get(id)
            const at = p.get(id)
            m.x -= at.x * 0.06
            m.y -= at.y * 0.06
            const length = Math.hypot(m.x, m.y)
            const scale = length > heat ? heat / length : 1
            at.x += m.x * scale
            at.y += m.y * scale
        }
    }
    return p
}

function radial(nodes, edges) {
    const near = new Map(nodes.map((node) => [node.id, new Set()]))
    for (const edge of edges) {
        if (near.has(edge.from) && near.has(edge.to) && edge.from !== edge.to) {
            near.get(edge.from).add(edge.to)
            near.get(edge.to).add(edge.from)
        }
    }
    const byId = new Map(nodes.map((node) => [node.id, node]))
    const rings = []
    const angle = new Map()
    const seen = new Set()
    const parent = new Map()
    const roots = [...nodes].sort((a, b) => near.get(b.id).size - near.get(a.id).size)
    for (const root of roots) {
        if (seen.has(root.id) || near.get(root.id).size === 0) {
            continue
        }
        seen.add(root.id)
        let ring = [root.id]
        let depth = 0
        while (ring.length) {
            rings[depth] = [...(rings[depth] ?? []), ...ring]
            const next = []
            for (const id of ring) {
                for (const other of near.get(id)) {
                    if (!seen.has(other)) {
                        seen.add(other)
                        parent.set(other, id)
                        next.push(other)
                    }
                }
            }
            ring = next
            depth++
        }
    }
    const lonely = nodes.filter((node) => !seen.has(node.id)).map((node) => node.id)
    if (lonely.length) {
        rings.push(lonely)
    }
    const p = new Map()
    let radius = 0
    rings.forEach((ids, depth) => {
        const span = (id) => (byId.get(id).w + byId.get(id).h) / 2 + 16
        const around = ids.reduce((sum, id) => sum + span(id), 0)
        const thickest = Math.max(...ids.map((id) => byId.get(id).h))
        if (depth === 0 && ids.length === 1) {
            p.set(ids[0], { x: 0, y: 0 })
            angle.set(ids[0], 0)
            radius = Math.max(byId.get(ids[0]).w, byId.get(ids[0]).h) / 2
            return
        }
        radius = Math.max(radius + thickest + 70, around / (2 * Math.PI))
        ids.sort((a, b) => (angle.get(parent.get(a)) ?? 0) - (angle.get(parent.get(b)) ?? 0))
        let walked = 0
        for (const id of ids) {
            const a = ((walked + span(id) / 2) / around) * 2 * Math.PI
            walked += span(id)
            angle.set(id, a)
            p.set(id, { x: radius * Math.cos(a), y: radius * Math.sin(a) })
        }
    })
    return p
}

function circular(nodes, edges) {
    const { positions } = layered(nodes, edges)
    const order = [...nodes].sort((a, b) => {
        const pa = positions.get(a.id)
        const pb = positions.get(b.id)
        return pa.x - pb.x || pa.y - pb.y
    })
    const span = (node) => (node.w + node.h) / 2 + 16
    const around = order.reduce((sum, node) => sum + span(node), 0)
    const radius = Math.max(60, around / (2 * Math.PI))
    const p = new Map()
    let walked = 0
    for (const node of order) {
        const a = ((walked + span(node) / 2) / around) * 2 * Math.PI - Math.PI / 2
        walked += span(node)
        p.set(node.id, { x: radius * Math.cos(a), y: radius * Math.sin(a) })
    }
    return p
}

export function separate(nodes, positions) {
    for (let pass = 0; pass < 400; pass++) {
        let moved = false
        for (let i = 0; i < nodes.length; i++) {
            for (let j = i + 1; j < nodes.length; j++) {
                const a = positions.get(nodes[i].id)
                const b = positions.get(nodes[j].id)
                const ox = (nodes[i].w + nodes[j].w) / 2 + MARGIN - Math.abs(a.x - b.x)
                const oy = (nodes[i].h + nodes[j].h) / 2 + MARGIN - Math.abs(a.y - b.y)
                if (ox <= 0 || oy <= 0) {
                    continue
                }
                moved = true
                if (ox < oy) {
                    const s = (a.x < b.x || (a.x === b.x && i < j) ? -1 : 1) * (ox / 2 + 0.5)
                    a.x += s
                    b.x -= s
                } else {
                    const s = (a.y < b.y || (a.y === b.y && i < j) ? -1 : 1) * (oy / 2 + 0.5)
                    a.y += s
                    b.y -= s
                }
            }
        }
        if (!moved) {
            break
        }
    }
    return positions
}

export function overlaps(nodes, positions) {
    for (let i = 0; i < nodes.length; i++) {
        for (let j = i + 1; j < nodes.length; j++) {
            const a = positions.get(nodes[i].id)
            const b = positions.get(nodes[j].id)
            if (
                Math.abs(a.x - b.x) < (nodes[i].w + nodes[j].w) / 2 &&
                Math.abs(a.y - b.y) < (nodes[i].h + nodes[j].h) / 2
            ) {
                return true
            }
        }
    }
    return false
}
