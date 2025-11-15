import * as d3 from 'd3';

interface FileDefinition {
  defines: string[];
  uses: [string, string][];
}

interface ImportStructure {
  [key: string]: FileDefinition;
}

interface NodeData {
  id: string;
  fileName: string;
  defines: string[];
  radius: number;
  x?: number;
  y?: number;
  fx?: number | null;
  fy?: number | null;
}

interface LinkData {
  source: string | NodeData;
  target: string | NodeData;
  definition: string;
}

interface DefinitionDot {
  name: string;
  fileId: string;
  x: number;
  y: number;
}

class DependencyGraph {
  private width: number = window.innerWidth;
  private height: number = window.innerHeight;
  private svg: d3.Selection<SVGSVGElement, unknown, HTMLElement, any>;
  private g: d3.Selection<SVGGElement, unknown, HTMLElement, any>;
  private simulation: d3.Simulation<NodeData, LinkData>;
  private tooltip: d3.Selection<HTMLDivElement, unknown, HTMLElement, any>;

  constructor(container: string) {
    // Create tooltip
    this.tooltip = d3.select('body').append('div')
      .attr('class', 'tooltip')
      .style('position', 'absolute')
      .style('padding', '10px')
      .style('background', 'rgba(0, 0, 0, 0.8)')
      .style('color', 'white')
      .style('border-radius', '5px')
      .style('pointer-events', 'none')
      .style('opacity', 0)
      .style('font-family', 'monospace')
      .style('font-size', '12px')
      .style('z-index', '1000');

    // Create SVG
    this.svg = d3.select(container)
      .append('svg')
      .attr('width', this.width)
      .attr('height', this.height)
      .style('background-color', '#f5f5f5');

    // Add zoom behavior
    const zoom = d3.zoom<SVGSVGElement, unknown>()
      .scaleExtent([0.1, 4])
      .on('zoom', (event) => {
        this.g.attr('transform', event.transform.toString());
      });

    this.svg.call(zoom);

    // Create main group for transformations
    this.g = this.svg.append('g');

    // Initialize simulation with optimized parameters for large graphs
    this.simulation = d3.forceSimulation<NodeData, LinkData>()
      .force('link', d3.forceLink<NodeData, LinkData>()
        .id((d: any) => d.id)
        .distance(150)
        .strength(0.5))
      .force('charge', d3.forceManyBody()
        .strength(-500)
        .distanceMax(500))
      .force('center', d3.forceCenter(this.width / 2, this.height / 2))
      .force('collision', d3.forceCollide<NodeData>()
        .radius((d) => d.radius + 30)
        .strength(0.8))
      .force('x', d3.forceX(this.width / 2).strength(0.05))
      .force('y', d3.forceY(this.height / 2).strength(0.05));
  }

  async loadData() {
    try {
      const response = await fetch('/import_structure.json');
      const data: ImportStructure = await response.json();
      this.processAndRender(data);
      this.updateStats(data);
    } catch (error) {
      console.error('Error loading data:', error);
      // Show error in stats panel
      const statsContent = document.getElementById('stats-content');
      if (statsContent) {
        statsContent.innerHTML = `<span style="color: red;">Error loading data!</span>`;
      }
    }
  }

  private updateStats(data: ImportStructure) {
    const totalFiles = Object.keys(data).length;
    const totalDefs = Object.values(data).reduce((acc, file) => acc + file.defines.length, 0);
    const totalLinks = Object.values(data).reduce((acc, file) => acc + file.uses.length, 0);

    const statsContent = document.getElementById('stats-content');
    if (statsContent) {
      statsContent.innerHTML = `
        <strong>Files:</strong> ${totalFiles}<br>
        <strong>Definitions:</strong> ${totalDefs}<br>
        <strong>Import Links:</strong> ${totalLinks}<br>
        <br>
        <small>Zoom: scroll<br>Pan: drag background<br>Move: drag nodes</small>
      `;
    }
  }

  private processAndRender(data: ImportStructure) {
    // Create nodes from files
    const nodes: NodeData[] = Object.entries(data).map(([filePath, info]) => {
      const radius = Math.max(20, Math.min(80, 15 + info.defines.length * 5));
      return {
        id: filePath,
        fileName: filePath.split('/').pop() || filePath,
        defines: info.defines,
        radius: radius
      };
    });

    // Create links from uses
    const links: LinkData[] = [];
    Object.entries(data).forEach(([filePath, info]) => {
      info.uses.forEach(([targetFile, definition]) => {
        links.push({
          source: filePath,
          target: targetFile,
          definition: definition
        });
      });
    });

    // Create link elements
    const linkGroup = this.g.append('g').attr('class', 'links');
    const linkElements = linkGroup.selectAll('line')
      .data(links)
      .enter().append('line')
      .style('stroke', '#00ff00')
      .style('stroke-opacity', 0.6)
      .style('stroke-width', 1);

    // Create node groups
    const nodeGroup = this.g.append('g').attr('class', 'nodes');
    const nodeElements = nodeGroup.selectAll('g')
      .data(nodes)
      .enter().append('g')
      .attr('class', 'node')
      .call(this.createDragBehavior());

    // Add circles for files
    nodeElements.append('circle')
      .attr('r', (d) => d.radius)
      .style('fill', '#404040')
      .style('stroke', '#202020')
      .style('stroke-width', 2)
      .style('fill-opacity', 0.7);

    // Add file name labels
    nodeElements.append('text')
      .text((d) => d.fileName)
      .attr('y', (d) => d.radius + 15)
      .style('text-anchor', 'middle')
      .style('font-size', '10px')
      .style('font-family', 'monospace')
      .style('fill', '#202020');

    // Add definition dots
    const defGroup = nodeElements.append('g').attr('class', 'definitions');

    nodeElements.each((nodeData, nodeIndex, nodeList) => {
      const defContainer = d3.select(nodeList[nodeIndex]).select('.definitions');
      const numDefs = nodeData.defines.length;

      nodeData.defines.forEach((def, i) => {
        // Calculate position for dots in a circular pattern inside the file circle
        const angle = numDefs === 1 ? 0 : (i * 2 * Math.PI) / numDefs;
        const dotRadius = Math.min(nodeData.radius - 10, nodeData.radius * 0.7);
        const x = numDefs === 1 ? 0 : Math.cos(angle) * dotRadius;
        const y = numDefs === 1 ? 0 : Math.sin(angle) * dotRadius;

        const dotGroup = defContainer.append('g')
          .attr('transform', `translate(${x}, ${y})`);

        // Add the dot
        dotGroup.append('circle')
          .attr('r', 3)
          .style('fill', '#808080')
          .style('stroke', '#606060')
          .style('stroke-width', 1)
          .on('mouseover', (event) => {
            this.tooltip
              .style('opacity', 1)
              .html(def)
              .style('left', (event.pageX + 10) + 'px')
              .style('top', (event.pageY - 10) + 'px');
          })
          .on('mouseout', () => {
            this.tooltip.style('opacity', 0);
          });

        // Add truncated label
        const truncatedName = def.length > 6 ? def.substring(0, 4) + '..' : def;
        dotGroup.append('text')
          .text(truncatedName)
          .attr('x', 6)
          .attr('y', 3)
          .style('font-size', '8px')
          .style('font-family', 'monospace')
          .style('fill', '#303030')
          .style('pointer-events', 'none');
      });
    });

    // Update simulation
    this.simulation
      .nodes(nodes)
      .on('tick', () => {
        // Update link positions
        linkElements
          .attr('x1', (d: any) => d.source.x)
          .attr('y1', (d: any) => d.source.y)
          .attr('x2', (d: any) => d.target.x)
          .attr('y2', (d: any) => d.target.y);

        // Update node positions
        nodeElements
          .attr('transform', (d) => `translate(${d.x},${d.y})`);
      });

    (this.simulation.force('link') as d3.ForceLink<NodeData, LinkData>).links(links);
  }

  private createDragBehavior() {
    const dragstarted = (event: any, d: NodeData) => {
      if (!event.active) this.simulation.alphaTarget(0.3).restart();
      d.fx = d.x;
      d.fy = d.y;
    };

    const dragged = (event: any, d: NodeData) => {
      d.fx = event.x;
      d.fy = event.y;
    };

    const dragended = (event: any, d: NodeData) => {
      if (!event.active) this.simulation.alphaTarget(0);
      d.fx = null;
      d.fy = null;
    };

    return d3.drag<SVGGElement, NodeData>()
      .on('start', dragstarted)
      .on('drag', dragged)
      .on('end', dragended);
  }
}

// Initialize the visualization when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
  const graph = new DependencyGraph('#graph-container');
  graph.loadData();
});