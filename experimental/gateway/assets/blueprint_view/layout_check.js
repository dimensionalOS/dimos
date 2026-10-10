import { layout, LAYOUTS, overlaps } from "./layout.js"

function crowded(modules, extras) {
    const nodes = []
    const edges = []
    let seed = 7
    const random = () => (seed = (seed * 16807) % 2147483647) / 2147483647
    for (let m = 0; m < modules; m++) {
        nodes.push({ id: `m:${m}`, w: 90 + Math.floor(random() * 120), h: 34 })
    }
    const topics = modules * 2
    for (let t = 0; t < topics; t++) {
        nodes.push({ id: `t:${t}`, w: 80 + Math.floor(random() * 140), h: 36 })
        edges.push({ from: `m:${t % modules}`, to: `t:${t}` })
    }
    for (let m = 0; m < modules; m++) {
        for (let k = 0; k < 3; k++) {
            edges.push({ from: `t:${Math.floor(random() * topics)}`, to: `m:${m}` })
        }
    }
    nodes.push({ id: "t:in", w: 100, h: 36 }, { id: "m:alone", w: 160, h: 34 })
    edges.push({ from: "t:in", to: "m:0" })
    for (let e = 0; e < extras; e++) {
        nodes.push({ id: `t:extra${e}`, w: 70 + Math.floor(random() * 150), h: 36 })
    }
    return { nodes, edges }
}

const results = []
for (const [modules, extras] of [[4, 0], [4, 20], [24, 0], [24, 20], [60, 20]]) {
    const { nodes, edges } = crowded(modules, extras)
    for (const { id } of LAYOUTS) {
        const { positions, vertical } = layout(id, nodes, edges)
        results.push({ modules, extras, layout: id, vertical: !!vertical, overlaps: overlaps(nodes, positions) })
    }
}
console.log(JSON.stringify(results))
