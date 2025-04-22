import * as d3 from "npm:d3"
import * as React from "npm:react"
import * as ReactDOM from "npm:react-dom"
import * as ReactDOMClient from "npm:react-dom/client"
import { Costmap, Drawable, Grid, Vector } from "./types.ts"

// React component for visualization
const VisualizerComponent: React.FC<{
    state: { [key: string]: Drawable }
}> = ({ state }) => {
    const svgRef = React.useRef<SVGSVGElement>(null)
    const width = 800
    const height = 600

    // We'll use the first costmap's coordinates as the "world coordinates"
    
    React.useEffect(() => {
        if (!svgRef.current) return

        // Clear previous visualization
        const svg = d3.select(svgRef.current)
        svg.selectAll("*").remove()

        // Find a costmap to use as reference for coordinate system
        let referenceCostmap: Costmap | undefined = undefined
        for (const drawable of Object.values(state)) {
            if (drawable instanceof Costmap) {
                referenceCostmap = drawable
                break
            }
        }

        // First render all costmaps
        for (const [key, drawable] of Object.entries(state)) {
            if (drawable instanceof Costmap) {
                visualizeCostmap(svg, drawable, width, height)
            }
        }

        // Then render all vectors, using the reference costmap if available
        for (const [key, drawable] of Object.entries(state)) {
            if (drawable instanceof Vector) {
                visualizeVector(svg, drawable, key, width, height, referenceCostmap)
            }
        }
    }, [state])

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

// Helper function to visualize a Costmap
function visualizeCostmap(
    svg: d3.Selection<SVGSVGElement, unknown, null, undefined>,
    costmap: Costmap,
    width: number,
    height: number,
): void {
    const { grid, origin, resolution, origin_theta } = costmap
    const [rows, cols] = grid.shape

    // Adjust cell size based on grid dimensions and container size
    const cellSize = Math.min(
        width / cols,
        height / rows,
    )

    // Calculate the required area for the grid
    const gridWidth = cols * cellSize
    const gridHeight = rows * cellSize

    // Add transformation group for the entire costmap
    const costmapGroup = svg
        .append("g")
        .attr(
            "transform",
            `translate(${(width - gridWidth) / 2}, ${
                (height - gridHeight) / 2
            })`,
        )

    // Determine value range for proper coloring
    const minValue = 0
    const maxValue = 100
    const colorScale = d3.scaleSequential(d3.interpolateGreys)
        .domain([minValue, maxValue])

    // Create a canvas element for rendering
    const foreignObject = costmapGroup.append("foreignObject")
        .attr("width", gridWidth)
        .attr("height", gridHeight)

    // Create a canvas element inside the foreignObject
    const canvasEl = document.createElement("canvas")
    canvasEl.width = cols
    canvasEl.height = rows
    canvasEl.style.width = "100%"
    canvasEl.style.height = "100%"
    canvasEl.style.objectFit = "contain"

    // Append the canvas to the foreign object
    const canvasDiv = foreignObject.append("xhtml:div")
        .style("width", "100%")
        .style("height", "100%")
        .style("display", "flex")
        .style("align-items", "center")
        .style("justify-content", "center")
        .node()

    if (canvasDiv) {
        canvasDiv.appendChild(canvasEl)
    }

    // Get canvas context and render the grid
    const ctx = canvasEl.getContext("2d")
    if (ctx) {
        // Create ImageData from the grid data
        const imageData = ctx.createImageData(cols, rows)
        const typedArray = grid.data

        // Fill the image data with colors based on the grid values
        for (let i = 0; i < typedArray.length; i++) {
            const value = typedArray[i]
            // Get color from scale
            const color = d3.color(colorScale(value))
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

    // Add coordinate system
    addCoordinateSystem(
        costmapGroup,
        gridWidth,
        gridHeight,
        origin,
        resolution,
    )
    
    // Add debug info text
    svg.append("text")
        .attr("x", 10)
        .attr("y", 15)
        .attr("font-size", "11px")
        .attr("fill", "black")
        .attr("font-weight", "bold")
        .attr("text-anchor", "start")
        .text("World Coordinate System")
        
    svg.append("text")
        .attr("x", 10)
        .attr("y", 30)
        .attr("font-size", "10px")
        .attr("fill", "red")
        .attr("font-weight", "bold")
        .attr("text-anchor", "start")
        .text(`Costmap Origin: (${origin.coords[0].toFixed(2)},${origin.coords[1].toFixed(2)}), Resolution: ${resolution.toFixed(4)}`)
}

// Helper function to visualize a Vector
function visualizeVector(
    svg: d3.Selection<SVGSVGElement, unknown, null, undefined>,
    vector: Vector,
    label: string,
    width: number,
    height: number,
    referenceMap?: Costmap, // Reference costmap for coordinate alignment
): void {
    // Get original vector coordinates
    const worldX = vector.coords[0];
    const worldY = vector.coords[1];
    
    // Initialize coordinates in the chosen coordinate system
    let x = worldX;
    let y = worldY;
    
    // If we have a reference costmap, transform to its coordinate system
    if (referenceMap) {
        // To transform from world to costmap coordinates:
        // Subtract the costmap origin from the world coordinates
        x = worldX - referenceMap.origin.coords[0];
        y = worldY - referenceMap.origin.coords[1];
    }

    // If we have a reference costmap, use its coordinate system
    if (referenceMap) {
        const { origin, resolution } = referenceMap
        const rows = referenceMap.grid.shape[0]
        const cols = referenceMap.grid.shape[1]
        
        // Calculate the world extents of the costmap
        const worldMinX = origin.coords[0]
        const worldMinY = origin.coords[1]
        const worldMaxX = worldMinX + cols * resolution
        const worldMaxY = worldMinY + rows * resolution
        
        // Calculate SVG dimensions for the costmap visualization
        const cellSize = Math.min(width / cols, height / rows)
        const svgGridWidth = cols * cellSize
        const svgGridHeight = rows * cellSize
        
        // Position of the costmap group within the SVG
        const translateX = (width - svgGridWidth) / 2
        const translateY = (height - svgGridHeight) / 2
        
        // Create scales that map world coordinates to SVG pixels
        // The issue is that we need to treat the vector coordinates as relative to the 
        // costmap's coordinate system, not the world coordinate system.
        const xScale = d3.scaleLinear()
            .domain([worldMinX, worldMaxX])
            .range([0, svgGridWidth])
        
        const yScale = d3.scaleLinear()
            .domain([worldMinY, worldMaxY])
            .range([svgGridHeight, 0]) // Inverted y-axis for SVG
        
        // If the vector is (0,0), it should be exactly at the costmap's origin
        // Map vector coordinates to SVG coordinates
        const svgX = xScale(x)
        const svgY = yScale(y)
        
        // Mark costmap origin point
        const originX = xScale(worldMinX)
        const originY = yScale(worldMinY)
        
        // Also calculate where world (0,0) is in the SVG for reference
        const worldOriginX = xScale(0)
        const worldOriginY = yScale(0)
        
        // Pick color based on the label
        const color = d3.scaleOrdinal(d3.schemeCategory10)(label)
        
        // Draw costmap origin marker (bottom-left corner of the costmap)
        svg.append("circle")
            .attr("cx", translateX + originX)
            .attr("cy", translateY + originY)
            .attr("r", 5)
            .attr("fill", "red")
            .attr("stroke", "white")
            .attr("stroke-width", 1.5)
            .attr("opacity", 1.0)
            .append("title")
            .text(`Costmap Origin: (${worldMinX.toFixed(2)},${worldMinY.toFixed(2)})`)
            
        // Add a text label for the costmap origin
        svg.append("text")
            .attr("x", translateX + originX + 8)
            .attr("y", translateY + originY - 8)
            .attr("font-size", "10px")
            .attr("fill", "red")
            .attr("font-weight", "bold")
            .attr("text-anchor", "start")
            .text(`(${worldMinX.toFixed(1)},${worldMinY.toFixed(1)})`)
        
        // Draw the world origin (0,0) if it's visible in the map
        if (worldMinX <= 0 && 0 <= worldMaxX && worldMinY <= 0 && 0 <= worldMaxY) {
            svg.append("circle")
                .attr("cx", translateX + worldOriginX)
                .attr("cy", translateY + worldOriginY)
                .attr("r", 4)
                .attr("fill", "blue")
                .attr("stroke", "white")
                .attr("stroke-width", 1)
                .attr("opacity", 0.9)
                .append("title")
                .text("World Origin (0,0)")
                
            // Draw a line from world origin (0,0) to the vector point
            svg.append("line")
                .attr("x1", translateX + worldOriginX)
                .attr("y1", translateY + worldOriginY)
                .attr("x2", translateX + svgX)
                .attr("y2", translateY + svgY)
                .attr("stroke", color)
                .attr("stroke-width", 1)
                .attr("stroke-dasharray", "3,3")
                .attr("opacity", 0.6)
        }
        
        // Always draw a line from costmap origin (0,0) to the vector
        // since we're using costmap coordinates
        svg.append("line")
            .attr("x1", translateX + originX)
            .attr("y1", translateY + originY)
            .attr("x2", translateX + svgX)
            .attr("y2", translateY + svgY)
            .attr("stroke", "red")
            .attr("stroke-width", 1)
            .attr("stroke-dasharray", "3,3")
            .attr("opacity", 0.6)
            
        // Add the vector point
        svg.append("circle")
            .attr("cx", translateX + svgX)
            .attr("cy", translateY + svgY)
            .attr("r", 5)
            .attr("fill", color)
            .attr("stroke", "white")
            .attr("stroke-width", 1.5)
            .append("title")
            .text(`${label}: (${x.toFixed(2)}, ${y.toFixed(2)})`)

        // Add a text label
        svg.append("text")
            .attr("x", translateX + svgX + 7)
            .attr("y", translateY + svgY - 7)
            .attr("font-size", "10px")
            .attr("fill", color)
            .attr("text-anchor", "start")
            .text(label)
        
        // Add debug information text 
        svg.append("text")
            .attr("x", 10)
            .attr("y", height - 40)
            .attr("font-size", "11px")
            .attr("fill", "black")
            .attr("font-weight", "bold")
            .attr("text-anchor", "start")
            .text("Costmap Coordinate System")
        
        svg.append("text")
            .attr("x", 10)
            .attr("y", height - 25)
            .attr("font-size", "10px")
            .attr("fill", "red")
            .attr("font-weight", "bold")
            .attr("text-anchor", "start")
            .text(`Costmap Origin: (${worldMinX.toFixed(2)},${worldMinY.toFixed(2)}) in world coords [RED DOT]`)
            
        svg.append("text")
            .attr("x", 10)
            .attr("y", height - 10)
            .attr("font-size", "10px")
            .attr("fill", color)
            .attr("font-weight", "bold")
            .attr("text-anchor", "start")
            .text(`Vector ${label}: (${x.toFixed(2)},${y.toFixed(2)}) in costmap coords [${color} DOT]`)
        
        // Draw map bounds for debugging
        svg.append("rect")
            .attr("x", translateX)
            .attr("y", translateY)
            .attr("width", svgGridWidth)
            .attr("height", svgGridHeight)
            .attr("fill", "none")
            .attr("stroke", "blue")
            .attr("stroke-width", 0.5)
            .attr("stroke-dasharray", "2,2")
            .attr("opacity", 0.6)
        
        // Add a costmap-aligned grid
        // Create grid at intervals appropriate for the map resolution
        const gridStep = Math.max(1, Math.ceil(resolution * 5))
        
        // Calculate costmap coordinates (starting from 0,0 at costmap origin)
        const costmapWidth = cols * resolution
        const costmapHeight = rows * resolution
        
        // Draw vertical grid lines at regular intervals
        for (let cx = 0; cx <= costmapWidth; cx += gridStep) {
            const gx = translateX + xScale(worldMinX + cx)
            svg.append("line")
                .attr("x1", gx)
                .attr("y1", translateY)
                .attr("x2", gx)
                .attr("y2", translateY + svgGridHeight)
                .attr("stroke", "#ddd")
                .attr("stroke-width", 0.5)
                .attr("opacity", 0.4)
                
            // Removed the tick labels
        }
        
        // Draw horizontal grid lines at regular intervals
        for (let cy = 0; cy <= costmapHeight; cy += gridStep) {
            const gy = translateY + yScale(worldMinY + cy)
            svg.append("line")
                .attr("x1", translateX)
                .attr("y1", gy)
                .attr("x2", translateX + svgGridWidth)
                .attr("y2", gy)
                .attr("stroke", "#ddd")
                .attr("stroke-width", 0.5)
                .attr("opacity", 0.4)
                
            // Removed the tick labels
        }
            
        // We've removed the duplicate axes
            
    } else {
        // Fallback to original behavior if no reference costmap
        const color = d3.scaleOrdinal(d3.schemeCategory10)(label)
        
        svg.append("circle")
            .attr("cx", width / 2 + x)
            .attr("cy", height / 2 - y) // Invert y-axis
            .attr("r", 5)
            .attr("fill", color)
            .attr("stroke", "white")
            .attr("stroke-width", 1.5)
            .append("title")
            .text(`${label}: (${x.toFixed(2)}, ${y.toFixed(2)})`)
            
        // Add a small text label
        svg.append("text")
            .attr("x", width / 2 + x + 7)
            .attr("y", height / 2 - y - 7)
            .attr("font-size", "10px")
            .attr("fill", color)
            .attr("text-anchor", "start")
            .text(label)
    }
}

// Helper function to add coordinate system
function addCoordinateSystem(
    group: d3.Selection<SVGGElement, unknown, HTMLElement, any>,
    width: number,
    height: number,
    origin: Vector,
    resolution: number,
): void {
    // Get the world coordinate extents
    const worldMinX = origin.coords[0] 
    const worldMinY = origin.coords[1]
    const worldMaxX = worldMinX + width * resolution
    const worldMaxY = worldMinY + height * resolution
    
    // Create scales that map world coordinates to SVG pixels
    const xScale = d3.scaleLinear()
        .domain([worldMinX, worldMaxX])
        .range([0, width])

    const yScale = d3.scaleLinear()
        .domain([worldMinY, worldMaxY])
        .range([height, 0])

    // Add x-axis at the bottom with RED styling
    const xAxis = d3.axisBottom(xScale).ticks(5)
    const xAxisGroup = group.append("g")
        .attr("transform", `translate(0, ${height})`)
        .call(xAxis)
        .attr("class", "axis")
        
    // Style the costmap axes in RED
    xAxisGroup.selectAll("line").attr("stroke", "red").attr("stroke-width", 1.5)
    xAxisGroup.selectAll("path").attr("stroke", "red").attr("stroke-width", 1.5)
    xAxisGroup.selectAll("text").attr("fill", "red").attr("font-weight", "bold")

    // Add y-axis at the left with RED styling
    const yAxis = d3.axisLeft(yScale).ticks(5)
    const yAxisGroup = group.append("g")
        .call(yAxis)
        .attr("class", "axis")
        
    // Style the costmap axes in RED
    yAxisGroup.selectAll("line").attr("stroke", "red").attr("stroke-width", 1.5)
    yAxisGroup.selectAll("path").attr("stroke", "red").attr("stroke-width", 1.5)
    yAxisGroup.selectAll("text").attr("fill", "red").attr("font-weight", "bold")
    
    // Add a label to clarify these are costmap coordinates
    group.append("text")
        .attr("x", width)
        .attr("y", height - 5)
        .attr("text-anchor", "end")
        .attr("font-size", "10px")
        .attr("fill", "red")
        .attr("font-weight", "bold")
        .text("Costmap Coordinates")
    
    // Add origin point marker if it falls within the visible area
    if (worldMinX <= 0 && 0 <= worldMaxX && worldMinY <= 0 && 0 <= worldMaxY) {
        const originX = xScale(0)
        const originY = yScale(0)
        
        group.append("circle")
            .attr("cx", originX)
            .attr("cy", originY)
            .attr("r", 4)
            .attr("fill", "blue")
            .attr("opacity", 0.7)
            .append("title")
            .text("World Origin (0,0)")
    }
}

// Main class to handle visualization
export class Visualizer {
    private container: HTMLElement | null = null
    private state: { [key: string]: Drawable } = {}
    private resizeObserver: ResizeObserver | null = null

    constructor(selector: string) {
        this.container = document.querySelector(selector)

        if (!this.container) {
            console.error(`Container not found: ${selector}`)
            return
        }

        // Initial render
        this.render()

        // Set up resize observer
        if (window.ResizeObserver) {
            this.resizeObserver = new ResizeObserver(() => {
                this.render()
            })
            this.resizeObserver.observe(this.container)
        }
    }

    public visualizeState(state: { [key: string]: Drawable }): void {
        this.state = state
        this.render()
    }

    private render(): void {
        if (!this.container) return

        // Use React 18's createRoot API
        const root = ReactDOMClient.createRoot(this.container)
        root.render(
            React.createElement(VisualizerComponent, { state: this.state }),
        )
    }

    public cleanup(): void {
        if (this.resizeObserver && this.container) {
            this.resizeObserver.unobserve(this.container)
            this.resizeObserver.disconnect()
        }
    }
}

// Helper function to create and hook up visualization
export function createReactVis(selector: string): Visualizer {
    return new Visualizer(selector)
}
