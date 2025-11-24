import * as d3 from "npm:d3"
import { Costmap, Drawable, Grid, Vector } from "./types.ts"

export class CostmapVisualizer {
    private svg: d3.Selection<SVGSVGElement, unknown, HTMLElement, any>
    private canvas: HTMLCanvasElement | null = null
    private width: number
    private height: number
    private colorScale: d3.ScaleSequential<string>
    private cellSize: number = 4 // Default cell size

    constructor(
        selector: string,
        width: number = 800,
        height: number = 600,
    ) {
        this.width = width
        this.height = height

        // Create or select SVG element with responsive dimensions
        this.svg = d3.select(selector)
            .append("svg")
            .attr("width", "100%")
            .attr("height", "100%")
            .attr("viewBox", `0 0 ${width} ${height}`)
            .attr("preserveAspectRatio", "xMidYMid meet")
            .style("background-color", "#f8f9fa")

        this.colorScale = d3.scaleSequential(d3.interpolateGreys)
    }

    public visualize(
        costmap: Costmap,
    ): void {
        const { grid, origin, resolution, origin_theta } = costmap
        const [rows, cols] = grid.shape

        // Adjust cell size based on grid dimensions and container size
        this.cellSize = Math.min(
            this.width / cols,
            this.height / rows,
        )

        // Calculate the required area for the grid
        const gridWidth = cols * this.cellSize
        const gridHeight = rows * this.cellSize

        // Clear previous visualization
        this.svg.selectAll("*").remove()

        // Add transformation group for the entire costmap
        const costmapGroup = this.svg
            .append("g")
            .attr(
                "transform",
                `translate(${(this.width - gridWidth) / 2}, ${
                    (this.height - gridHeight) / 2
                })`,
            )

        // Determine value range for proper coloring
        const minValue = 0
        const maxValue = 100

        this.colorScale.domain([minValue, maxValue])

        // Create a canvas element for fast rendering that fills the container
        const foreignObject = costmapGroup.append("foreignObject")
            .attr("width", gridWidth)
            .attr("height", gridHeight)

        // Add a canvas element inside the foreignObject, ensuring it fills the container
        const canvasDiv = foreignObject.append("xhtml:div")
            .style("width", "100%")
            .style("height", "100%")
            .style("display", "flex")
            .style("align-items", "center")
            .style("justify-content", "center")

        // Create canvas element if not exists
        if (!this.canvas) {
            this.canvas = document.createElement("canvas")
            canvasDiv.node()?.appendChild(this.canvas)
        } else {
            // Reuse existing canvas
            canvasDiv.node()?.appendChild(this.canvas)
        }

        // Set canvas size - physical pixel dimensions for rendering
        this.canvas.width = cols
        this.canvas.height = rows

        // Set canvas display size to fill the available space
        this.canvas.style.width = "100%"
        this.canvas.style.height = "100%"
        this.canvas.style.objectFit = "contain" // Maintains aspect ratio

        // Get canvas context and render the grid
        const ctx = this.canvas.getContext("2d")
        if (ctx) {
            // Create ImageData directly from the grid data
            const imageData = ctx.createImageData(cols, rows)
            const typedArray = grid.data

            // Fill the image data with colors based on the grid values
            for (let i = 0; i < typedArray.length; i++) {
                const value = typedArray[i]
                // Get color from scale
                const color = d3.color(this.colorScale(value))
                if (color) {
                    const idx = i * 4
                    imageData.data[idx] = color.r || 0 // Red
                    imageData.data[idx + 1] = color.g || 0 // Green
                    imageData.data[idx + 2] = color.b || 0 // Blue
                    imageData.data[idx + 3] = 255 // Alpha (fully opaque)
                }
            }

            // Put the image data on the canvas
            ctx.putImageData(imageData, 0, 0)
        }

        // Add coordinates/scale
        this.addCoordinateSystem(
            costmapGroup,
            gridWidth,
            gridHeight,
            origin,
            resolution,
        )
    }

    private addCoordinateSystem(
        group: d3.Selection<SVGGElement, unknown, HTMLElement, any>,
        width: number,
        height: number,
        origin: Vector,
        resolution: number,
    ): void {
        // Add axes at the bottom and left edge
        const xScale = d3.scaleLinear()
            .domain([origin.coords[0], origin.coords[0] + width * resolution])
            .range([0, width])

        const yScale = d3.scaleLinear()
            .domain([origin.coords[1], origin.coords[1] + height * resolution])
            .range([height, 0])

        // Add x-axis at the bottom
        const xAxis = d3.axisBottom(xScale).ticks(5)
        group.append("g")
            .attr("transform", `translate(0, ${height})`)
            .call(xAxis)
            .attr("class", "axis")

        // Add y-axis at the left
        const yAxis = d3.axisLeft(yScale).ticks(5)
        group.append("g")
            .call(yAxis)
            .attr("class", "axis")
    }

    /**
     * @deprecated Use visualize with interpolator parameter directly
     */
    public setColorScale(interpolator: (t: number) => string): void {
        console.warn(
            "setColorScale is deprecated, pass the interpolator directly to visualize",
        )
    }

    // Method to add a legend for the costmap values
    public addLegend(minValue: number, maxValue: number): void {
        // Create a gradient definition
        const defs = this.svg.append("defs")
        const gradient = defs.append("linearGradient")
            .attr("id", "costmap-gradient")
            .attr("x1", "0%")
            .attr("y1", "0%")
            .attr("x2", "100%")
            .attr("y2", "0%")

        // Add color stops
        const steps = 10
        for (let i = 0; i <= steps; i++) {
            const t = i / steps
            gradient.append("stop")
                .attr("offset", `${t * 100}%`)
                .attr(
                    "stop-color",
                    this.colorScale(minValue + t * (maxValue - minValue)),
                )
        }

        // Add a rectangle with the gradient
        const legendWidth = 200
        const legendHeight = 20

        const legend = this.svg.append("g")
            .attr("class", "legend")
            .attr(
                "transform",
                `translate(${this.width - legendWidth - 20}, 20)`,
            )

        legend.append("rect")
            .attr("width", legendWidth)
            .attr("height", legendHeight)
            .style("fill", "url(#costmap-gradient)")

        // Add labels
        const legendScale = d3.scaleLinear()
            .domain([minValue, maxValue])
            .range([0, legendWidth])

        const legendAxis = d3.axisBottom(legendScale).ticks(5)

        legend.append("g")
            .attr("transform", `translate(0, ${legendHeight})`)
            .call(legendAxis)
    }
}

// Helper function to create and hook up visualization
export function createCostmapVis(
    selector: string,
    width: number = 800,
    height: number = 600,
): CostmapVisualizer {
    return new CostmapVisualizer(selector, width, height)
}

// Extension to visualize multiple drawables
export class RobotStateVisualizer {
    private costmapVis: CostmapVisualizer
    private svg: d3.Selection<SVGSVGElement, unknown, HTMLElement, any>
    private containerSelector: string
    private resizeObserver: ResizeObserver | null = null

    constructor(
        selector: string,
        width: number = 800,
        height: number = 600,
    ) {
        this.containerSelector = selector

        // Create base SVG with responsive sizing
        this.svg = d3.select(selector)
            .append("svg")
            .attr("width", "100%")
            .attr("height", "100%")
            .attr("viewBox", `0 0 ${width} ${height}`)
            .attr("preserveAspectRatio", "xMidYMid meet")
            .style("background-color", "#f8f9fa")

        // Create costmap visualizer that will render to the same SVG
        this.costmapVis = new CostmapVisualizer(selector, width, height)

        // Set up resize observer to update when container size changes
        const container = document.querySelector(selector)
        if (container && window.ResizeObserver) {
            this.resizeObserver = new ResizeObserver((entries) => {
                for (const entry of entries) {
                    const { width, height } = entry.contentRect
                    if (width > 0 && height > 0) {
                        this.updateSize(width, height)
                    }
                }
            })
            this.resizeObserver.observe(container)
        }
    }

    private updateSize(width: number, height: number): void {
        // Update viewBox to maintain aspect ratio
        this.svg.attr("viewBox", `0 0 ${width} ${height}`)
    }

    public visualizeState(
        state: { [key: string]: Drawable },
    ): void {
        // Clear previous visualization
        this.svg.selectAll("*").remove()

        // Visualize each drawable based on its type
        for (const [key, drawable] of Object.entries(state)) {
            if (drawable instanceof Costmap) {
                this.costmapVis.visualize(drawable)
            } else if (drawable instanceof Vector) {
                this.visualizeVector(drawable, key)
            }
        }
    }

    private visualizeVector(vector: Vector, label: string): void {
        // Implement vector visualization (arrows, points, etc.)
        // This is a simple implementation showing vectors as points
        const [x, y] = vector.coords

        console.log("VIS VECTOR", vector)
        this.svg.append("circle")
            .attr("cx", 200)
            .attr("cy", 200)
            .attr("r", 20)
            .attr("fill", "red")
            .append("title")
            .text(`${label}: (${x}, ${y})`)
    }
}
