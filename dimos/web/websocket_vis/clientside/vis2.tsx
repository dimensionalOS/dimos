import * as d3 from "npm:d3"
import * as React from "npm:react"
import * as ReactDOMClient from "npm:react-dom/client"
import { Costmap, Drawable, Path, Vector } from "./types.ts"

// ───────────────────────────────────────────────────────────────────────────────
// React component
// ───────────────────────────────────────────────────────────────────────────────
const VisualizerComponent: React.FC<{ state: Record<string, Drawable> }> = ({
    state,
}) => {
    const svgRef = React.useRef<SVGSVGElement>(null)
    const width = 800
    const height = 600

    /** Build a world→pixel transformer from the *first* cost‑map we see. */
    const { worldToPx, pxToWorld } = React.useMemo(() => {
        const ref = Object.values(state).find(
            (d): d is Costmap => d instanceof Costmap,
        )
        if (!ref) return { worldToPx: undefined, pxToWorld: undefined }

        const {
            grid: { shape },
            origin,
            resolution,
        } = ref
        const [rows, cols] = shape

        // Same sizing/centering logic used in visualiseCostmap
        const cell = Math.min(width / cols, height / rows)
        const gridW = cols * cell
        const gridH = rows * cell
        const offsetX = (width - gridW) / 2
        const offsetY = (height - gridH) / 2

        const xScale = d3
            .scaleLinear()
            .domain([origin.coords[0], origin.coords[0] + cols * resolution])
            .range([offsetX, offsetX + gridW])
        const yScale = d3
            .scaleLinear()
            .domain([origin.coords[1], origin.coords[1] + rows * resolution])
            .range([offsetY + gridH, offsetY]) // invert y (world ↑ => svg ↑)

        // World coordinates to pixel coordinates
        const worldToPxFn = (
            x: number,
            y: number,
        ): [number, number] => [xScale(x), yScale(y)]

        // Pixel coordinates to world coordinates (inverse transform)
        const pxToWorldFn = (
            x: number,
            y: number,
        ): [number, number] => [
            xScale.invert(x),
            yScale.invert(y),
        ]

        return {
            worldToPx: worldToPxFn,
            pxToWorld: pxToWorldFn,
        }
    }, [state])

    // ── main draw effect ────────────────────────────────────────────────────────
    const handleClick = React.useCallback((event: MouseEvent) => {
        if (!svgRef.current || !pxToWorld) return

        // Get SVG element position and dimensions
        const svgRect = svgRef.current.getBoundingClientRect()

        // Calculate click position relative to SVG viewport
        const viewportX = event.clientX - svgRect.left
        const viewportY = event.clientY - svgRect.top

        // Convert to SVG coordinate space (accounting for viewBox)
        const svgPoint = new DOMPoint(viewportX, viewportY)
        const transformedPoint = svgPoint.matrixTransform(
            svgRef.current.getScreenCTM()?.inverse(),
        )

        // Convert to world coordinates
        const [worldX, worldY] = pxToWorld(
            transformedPoint.x,
            transformedPoint.y,
        )

        console.log(
            "Click at world coordinates:",
            worldX.toFixed(2),
            worldY.toFixed(2),
        )
    }, [pxToWorld])

    React.useEffect(() => {
        if (!svgRef.current) return
        const svg = d3.select(svgRef.current)
        svg.selectAll("*").remove()

        // 1. maps (bottom layer)
        Object.values(state).forEach((d) => {
            if (d instanceof Costmap) visualiseCostmap(svg, d, width, height)
        })

        // 2. paths (middle layer)
        Object.entries(state).forEach(([key, d]) => {
            if (d instanceof Path) {
                visualisePath(svg, d, key, worldToPx, width, height)
            }
        })

        // 3. vectors (top layer)
        Object.entries(state).forEach(([key, d]) => {
            if (d instanceof Vector) {
                visualiseVector(svg, d, key, worldToPx, width, height)
            }
        })

        // Add click handler
        const svgElement = svgRef.current
        svgElement.addEventListener("click", handleClick)

        return () => {
            svgElement.removeEventListener("click", handleClick)
        }
    }, [state, worldToPx, handleClick])

    return (
        <div
            className="visualizer-container"
            style={{ width: "100%", height: "100%" }}
        >
            <svg
                ref={svgRef}
                width="100%"
                height="100%"
                viewBox={`0 0 ${width} ${height}`}
                preserveAspectRatio="xMidYMid meet"
                style={{ backgroundColor: "#f8f9fa" }}
            />
        </div>
    )
}

// ───────────────────────────────────────────────────────────────────────────────
// Helper: cost‑map
// ───────────────────────────────────────────────────────────────────────────────
function visualiseCostmap(
    svg: d3.Selection<SVGSVGElement, unknown, null, undefined>,
    costmap: Costmap,
    width: number,
    height: number,
): void {
    const { grid, origin, resolution } = costmap
    const [rows, cols] = grid.shape

    const cell = Math.min(width / cols, height / rows)
    const gridW = cols * cell
    const gridH = rows * cell

    const group = svg
        .append("g")
        .attr(
            "transform",
            `translate(${(width - gridW) / 2}, ${(height - gridH) / 2})`,
        )

    // Custom color interpolation function that maps 0 to white and other values to Inferno scale
    const customColorScale = (t: number) => {
        // If value is 0 (or very close to it), return white
        if (t < 0.1) return "#ffffff"
        if (t > 0.9) return "#000000"
        // Otherwise use the Inferno color scale
        return d3.interpolateGreys(t)
    }

    const colour = d3.scaleSequential(customColorScale).domain([
        0,
        100,
    ])

    const fo = group.append("foreignObject").attr("width", gridW).attr(
        "height",
        gridH,
    )
    const canvas = document.createElement("canvas")
    canvas.width = cols
    canvas.height = rows
    Object.assign(canvas.style, {
        width: "100%",
        height: "100%",
        objectFit: "contain",
    })
    fo.append("xhtml:div")
        .style("width", "100%")
        .style("height", "100%")
        .style("display", "flex")
        .style("alignItems", "center")
        .style("justifyContent", "center")
        .node()
        ?.appendChild(canvas)

    const ctx = canvas.getContext("2d")
    if (ctx) {
        const img = ctx.createImageData(cols, rows)
        const data = grid.data // row‑major, (0,0) = world south‑west

        // Flip vertically so world north appears at top of SVG
        for (let i = 0; i < data.length; i++) {
            const row = Math.floor(i / cols)
            const col = i % cols

            // Flip Y coordinate (invert row) to put origin at bottom-left
            const invertedRow = rows - 1 - row
            const srcIdx = invertedRow * cols + col

            const value = data[i] // Get value from original index
            const c = d3.color(colour(value))
            if (!c) continue
            const o = srcIdx * 4 // Write to flipped position
            img.data[o] = c.r ?? 0
            img.data[o + 1] = c.g ?? 0
            img.data[o + 2] = c.b ?? 0
            img.data[o + 3] = 255
        }
        ctx.putImageData(img, 0, 0)
    }

    addCoordinateSystem(group, gridW, gridH, origin, resolution)
}

// ───────────────────────────────────────────────────────────────────────────────
// Helper: coordinate system
// ───────────────────────────────────────────────────────────────────────────────
function addCoordinateSystem(
    group: d3.Selection<SVGGElement, unknown, HTMLElement, any>,
    width: number,
    height: number,
    origin: Vector,
    resolution: number,
): void {
    const minX = origin.coords[0]
    const minY = origin.coords[1]

    const maxX = minX + (width * resolution)
    const maxY = minY + (height * resolution)
    console.log(group, width, origin, maxX)

    const xScale = d3.scaleLinear().domain([
        minX,
        maxX,
    ]).range([0, width])
    const yScale = d3.scaleLinear().domain([
        minY,
        maxY,
    ]).range([height, 0])

    const gridSize = 1.0
    const gridColour = "#555"
    const gridGroup = group.append("g").attr("class", "grid")

    for (
        const x of d3.range(
            Math.ceil(minX / gridSize) * gridSize,
            maxX,
            gridSize,
        )
    ) {
        gridGroup
            .append("line")
            .attr("x1", xScale(x))
            .attr("y1", 0)
            .attr("x2", xScale(x))
            .attr("y2", height)
            .attr("stroke", gridColour)
            .attr("stroke-width", 0.25)
            .attr("opacity", 0.7)
    }
    for (
        const y of d3.range(
            Math.ceil(minY / gridSize) * gridSize,
            maxY,
            gridSize,
        )
    ) {
        gridGroup
            .append("line")
            .attr("x1", 0)
            .attr("y1", yScale(y))
            .attr("x2", width)
            .attr("y2", yScale(y))
            .attr("stroke", gridColour)
            .attr("stroke-width", 0.25)
            .attr("opacity", 0.7)
    }

    const stylise = (
        sel: d3.Selection<SVGGElement, unknown, null, undefined>,
    ) => sel.selectAll("line,path").attr("stroke", "black").attr(
        "stroke-width",
        1,
    )

    group
        .append("g")
        .attr("transform", `translate(0, ${height})`)
        .call(d3.axisBottom(xScale).ticks(5))
        .call(stylise)
    group.append("g").call(d3.axisLeft(yScale).ticks(5)).call(stylise)

    if (minX <= 0 && 0 <= maxX && minY <= 0 && 0 <= maxY) {
        group
            .append("circle")
            .attr("cx", xScale(0))
            .attr("cy", yScale(0))
            .attr("r", 3)
            .attr("fill", "green")
            .attr("opacity", 0.7)
            .append("title")
            .text("World Origin (0,0)")
    }
}

// ───────────────────────────────────────────────────────────────────────────────
// Helper: path
// ───────────────────────────────────────────────────────────────────────────────
function visualisePath(
    svg: d3.Selection<SVGSVGElement, unknown, null, undefined>,
    path: Path,
    label: string,
    wp: ((x: number, y: number) => [number, number]) | undefined,
    width: number,
    height: number,
): void {
    if (path.coords.length < 2) return

    const points = path.coords.map((vector) => {
        return wp
            ? wp(vector.coords[0], vector.coords[1])
            : [width / 2 + vector.coords[0], height / 2 - vector.coords[1]]
    })

    const colour = d3.scaleOrdinal(d3.schemeCategory10)(label)

    // Create a path line
    const line = d3.line()

    svg.append("path")
        .datum(points)
        .attr("fill", "none")
        .attr("stroke", "red")
        .attr("stroke-width", 1)
        .attr("stroke-dasharray", "5,5") // Dashed line
        .attr("d", line)
        .append("title")
        .text(label)

    // Add label near the middle point of the path
    //const midIdx = Math.floor(points.length / 2)
    const [mx, my] = points[points.length - 1]

    // Create a group for the text and background
    const textGroup = svg.append("g")

    // Add text with background
    const text = `${label} (${path.coords.length})`

    // Add text element
    const textElement = textGroup
        .append("text")
        .attr("x", mx + 7)
        .attr("y", my - 7)
        .attr("font-size", "10px")
        .attr("fill", "white")
        .text(text)

    // Add background rect
    const bbox = textElement.node()?.getBBox()
    if (bbox) {
        textGroup
            .insert("rect", "text")
            .attr("x", bbox.x - 1)
            .attr("y", bbox.y - 1)
            .attr("width", bbox.width + 2)
            .attr("height", bbox.height + 2)
            .attr("fill", "black")
            .attr("stroke", "black")
            .attr("opacity", 0.7)
    }
}

// ───────────────────────────────────────────────────────────────────────────────
// Helper: vector
// ───────────────────────────────────────────────────────────────────────────────
function visualiseVector(
    svg: d3.Selection<SVGSVGElement, unknown, null, undefined>,
    vector: Vector,
    label: string,
    wp: ((x: number, y: number) => [number, number]) | undefined,
    width: number,
    height: number,
): void {
    const [cx, cy] = wp
        ? wp(vector.coords[0], vector.coords[1])
        : [width / 2 + vector.coords[0], height / 2 - vector.coords[1]]

    const colour = d3.scaleOrdinal(d3.schemeCategory10)(label)

    svg
        .append("circle")
        .attr("cx", cx)
        .attr("cy", cy)
        .attr("r", 4)
        .attr("fill", colour)

    // Add text with background
    const text = `${label} (${vector.coords[0].toFixed(2)}, ${
        vector.coords[1].toFixed(2)
    })`

    // Create a group for the text and background
    const textGroup = svg.append("g")

    // Add text element
    const textElement = textGroup
        .append("text")
        .attr("x", cx + 7)
        .attr("y", cy - 7)
        .attr("font-size", "10px")
        .attr("fill", "white")
        .text(text)

    // Add background rect
    const bbox = textElement.node()?.getBBox()
    if (bbox) {
        textGroup
            .insert("rect", "text")
            .attr("x", bbox.x - 1)
            .attr("y", bbox.y - 1)
            .attr("width", bbox.width + 2)
            .attr("height", bbox.height + 2)
            .attr("fill", "black")
            .attr("stroke", "black")
            .attr("opacity", 0.7)
    }
}

// ───────────────────────────────────────────────────────────────────────────────
// Wrapper class
// ───────────────────────────────────────────────────────────────────────────────
export class Visualizer {
    private container: HTMLElement | null
    private state: Record<string, Drawable> = {}
    private resizeObserver: ResizeObserver | null = null
    private root: ReactDOMClient.Root
    private onClickCallback: ((worldX: number, worldY: number) => void) | null =
        null

    constructor(selector: string) {
        this.container = document.querySelector(selector)
        if (!this.container) throw new Error(`Container not found: ${selector}`)
        this.root = ReactDOMClient.createRoot(this.container)

        // First paint
        this.render()

        // Keep canvas responsive
        if (window.ResizeObserver) {
            this.resizeObserver = new ResizeObserver(() => this.render())
            this.resizeObserver.observe(this.container)
        }

        // Set up global click handler to capture world coordinates
        document.addEventListener("click", this.handleGlobalClick.bind(this))
    }

    /** Register a callback for when user clicks on the visualization */
    public onWorldClick(
        callback: (worldX: number, worldY: number) => void,
    ): void {
        this.onClickCallback = callback
    }

    /** Handle global click events, filtering for clicks within our SVG */
    private handleGlobalClick(event: MouseEvent): void {
        if (!this.onClickCallback || !this.container) return

        // Check if click was inside our container
        const containerRect = this.container.getBoundingClientRect()
        if (
            event.clientX < containerRect.left ||
            event.clientX > containerRect.right ||
            event.clientY < containerRect.top ||
            event.clientY > containerRect.bottom
        ) {
            return // Click was outside our container
        }

        // Find our SVG element
        const svgElement = this.container.querySelector("svg")
        if (!svgElement) return

        // Calculate click position relative to SVG viewport
        const svgRect = svgElement.getBoundingClientRect()
        const viewportX = event.clientX - svgRect.left
        const viewportY = event.clientY - svgRect.top

        // Convert to SVG coordinate space (accounting for viewBox)
        const svgPoint = new DOMPoint(viewportX, viewportY)
        const transformedPoint = svgPoint.matrixTransform(
            svgElement.getScreenCTM()?.inverse() || new DOMMatrix(),
        )

        // Find a costmap to use for coordinate conversion
        const costmap = Object.values(this.state).find(
            (d): d is Costmap => d instanceof Costmap,
        )

        if (!costmap) return

        const {
            grid: { shape },
            origin,
            resolution,
        } = costmap
        const [rows, cols] = shape
        const width = 800
        const height = 600

        // Calculate scales (same logic as in the component)
        const cell = Math.min(width / cols, height / rows)
        const gridW = cols * cell
        const gridH = rows * cell
        const offsetX = (width - gridW) / 2
        const offsetY = (height - gridH) / 2

        const xScale = d3
            .scaleLinear()
            .domain([offsetX, offsetX + gridW])
            .range([origin.coords[0], origin.coords[0] + cols * resolution])
        const yScale = d3
            .scaleLinear()
            .domain([offsetY + gridH, offsetY])
            .range([origin.coords[1], origin.coords[1] + rows * resolution])

        // Convert to world coordinates
        const worldX = xScale(transformedPoint.x)
        const worldY = yScale(transformedPoint.y)

        // Call the callback with the world coordinates
        this.onClickCallback(worldX, worldY)
    }

    /** Push a new application‑state snapshot to the visualiser */
    public visualizeState(state: Record<string, Drawable>): void {
        this.state = { ...state }
        this.render()
    }

    /** React‑render the component tree */
    private render(): void {
        this.root.render(<VisualizerComponent state={this.state} />)
    }

    /** Tear down listeners and free resources */
    public cleanup(): void {
        if (this.resizeObserver && this.container) {
            this.resizeObserver.unobserve(this.container)
            this.resizeObserver.disconnect()
        }
        document.removeEventListener("click", this.handleGlobalClick.bind(this))
    }
}

// Convenience factory ----------------------------------------------------------
export function createReactVis(selector: string): Visualizer {
    return new Visualizer(selector)
}
