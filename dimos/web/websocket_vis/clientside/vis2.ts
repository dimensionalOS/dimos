import * as d3 from "npm:d3"
import { Costmap, Drawable, Vector } from "./types.ts"

export class Visualizer {
    private svg: d3.Selection<SVGSVGElement, unknown, null, undefined>
    private vis: {
        [key: string]: d3.Selection<SVGSVGElement, unknown, null, undefined>
    } = {}
    constructor(selector: string) {
        this.svg = d3.select(selector)
            .append("svg")
            .attr("width", "100%")
            .attr("height", "100%")
    }

    visualizeDrawable(drawable: Drawable) {
        if (drawable instanceof Costmap) {
            return this.visualizeCostmap(drawable)
        } else if (drawable instanceof Vector) {
            return this.visualizeVector(drawable)
        }
    }

    visualizeVector(vector: Vector) {
    }

    visualizeCostmap(costmap: Costmap) {
    }
}
