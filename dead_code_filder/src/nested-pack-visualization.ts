import * as d3 from 'd3';

interface FileDefinition {
  defines: string[];
  uses: [string, string][];
}

interface ImportStructure {
  [key: string]: FileDefinition;
}

interface HierarchyNode {
  name: string;
  path: string;
  type: 'file' | 'directory';
  children?: HierarchyNode[];
  definitions?: string[];
  value?: number;
  x?: number;
  y?: number;
  r?: number;
  parent?: HierarchyNode;
  depth?: number;
}

interface LinkData {
  source: string;
  target: string;
  definition: string;
}

class NestedPackVisualization {
  private width: number = window.innerWidth;
  private height: number = window.innerHeight;
  private svg: d3.Selection<SVGSVGElement, unknown, HTMLElement, any>;
  private g: d3.Selection<SVGGElement, unknown, HTMLElement, any>;
  private tooltip: d3.Selection<HTMLDivElement, unknown, HTMLElement, any>;
  private importData: ImportStructure = {};
  private links: LinkData[] = [];
  private nodeMap: Map<string, any> = new Map();

  // ============================================================================
  // ADJUST THIS VALUE TO CONTROL SPACING
  // 0.9 = tight packing (10% padding)
  // 0.8 = moderate packing (20% padding) - default
  // 0.7 = loose packing (30% padding)
  // 0.5 = very loose (50% padding)
  // This controls how much of each circle is used for children
  // ============================================================================
  private PACKING_DENSITY: number = 0.8;

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

    // Clear any existing SVG
    d3.select(container).selectAll('svg').remove();

    // Create SVG
    this.svg = d3.select(container)
      .append('svg')
      .attr('width', this.width)
      .attr('height', this.height)
      .style('background-color', '#f5f5f5');

    // Add zoom behavior
    const zoom = d3.zoom<SVGSVGElement, unknown>()
      .scaleExtent([0.1, 10])  // Allow more zoom for deep nesting
      .on('zoom', (event) => {
        this.g.attr('transform', event.transform.toString());
      });

    this.svg.call(zoom);

    // Create main group for transformations
    this.g = this.svg.append('g');
  }

  async loadData() {
    try {
      const response = await fetch('/import_structure.json');
      const data: ImportStructure = await response.json();
      this.importData = data;
      this.processAndRender(data);
      this.updateStats(data);
    } catch (error) {
      console.error('Error loading data:', error);
      const statsContent = document.getElementById('stats-content');
      if (statsContent) {
        statsContent.innerHTML = `<span style="color: red;">Error loading data!</span>`;
      }
    }
  }

  private buildHierarchy(data: ImportStructure): HierarchyNode {
    const root: HierarchyNode = {
      name: 'root',
      path: '',
      type: 'directory',
      children: []
    };

    // Build tree structure from file paths
    Object.entries(data).forEach(([filePath, info]) => {
      const parts = filePath.split('/');
      let currentNode = root;

      parts.forEach((part, index) => {
        const isFile = index === parts.length - 1 && part.endsWith('.py');
        const currentPath = parts.slice(0, index + 1).join('/');

        if (isFile) {
          // Add file node
          const fileNode: HierarchyNode = {
            name: part,
            path: currentPath,
            type: 'file',
            definitions: info.defines,
            value: 1 + info.defines.length * 0.2  // File size based on definitions
          };
          if (!currentNode.children) currentNode.children = [];
          currentNode.children.push(fileNode);
        } else {
          // Find or create directory node
          let dirNode = currentNode.children?.find(child => child.name === part && child.type === 'directory');
          if (!dirNode) {
            dirNode = {
              name: part,
              path: currentPath,
              type: 'directory',
              children: []
            };
            if (!currentNode.children) currentNode.children = [];
            currentNode.children.push(dirNode);
          }
          currentNode = dirNode;
        }
      });
    });

    // Extract links
    Object.entries(data).forEach(([filePath, info]) => {
      info.uses.forEach(([targetFile, definition]) => {
        this.links.push({
          source: filePath,
          target: targetFile,
          definition: definition
        });
      });
    });

    return root;
  }

  private processAndRender(data: ImportStructure) {
    const hierarchyRoot = this.buildHierarchy(data);

    // Create d3 hierarchy
    const root = d3.hierarchy(hierarchyRoot)
      .sum(d => d.value || 0)
      .sort((a, b) => (b.value || 0) - (a.value || 0));

    // Create recursive pack layout
    const pack = d3.pack()
      .size([this.width - 100, this.height - 100])
      .padding(3);

    // Apply pack layout
    const packedRoot = pack(root) as d3.HierarchyCircularNode<HierarchyNode>;

    // Create groups for rendering
    const linkGroup = this.g.append('g').attr('class', 'links');
    const nodeGroup = this.g.append('g').attr('class', 'nodes');

    // Draw links first (behind everything)
    const linkElements = linkGroup.selectAll('line')
      .data(this.links)
      .enter().append('line')
      .style('stroke', '#00ff00')
      .style('stroke-opacity', 0.2)
      .style('stroke-width', 0.5);

    // Store nodes in map for link drawing
    packedRoot.descendants().forEach(d => {
      if (d.data.path) {
        this.nodeMap.set(d.data.path, d);
      }
    });

    // Create node elements
    const nodes = nodeGroup.selectAll('g')
      .data(packedRoot.descendants())
      .enter().append('g')
      .attr('transform', d => `translate(${d.x + 50},${d.y + 50})`);

    // Draw circles
    nodes.append('circle')
      .attr('r', d => d.r)
      .attr('fill', d => {
        if (d.data.type === 'file') {
          return '#505050';
        } else {
          return 'none';
        }
      })
      .attr('stroke', d => {
        if (d.data.type === 'directory') {
          const colors = ['#202020', '#404040', '#606060', '#808080', '#a0a0a0'];
          return colors[Math.min(d.depth, colors.length - 1)];
        }
        return '#303030';
      })
      .attr('stroke-width', d => {
        if (d.data.type === 'directory') {
          return Math.max(0.5, 3 - d.depth * 0.5);
        }
        return 0.5;
      })
      .attr('stroke-dasharray', d => {
        if (d.data.type === 'directory' && d.depth > 0) {
          return '3,2';
        }
        return 'none';
      })
      .attr('fill-opacity', d => {
        if (d.data.type === 'file') {
          return 0.7;
        }
        return 0.02;
      })
      .on('mouseover', (event, d) => {
        if (d.data.type === 'file') {
          // Highlight connections
          linkElements
            .style('stroke-opacity', (l: LinkData) =>
              l.source === d.data.path || l.target === d.data.path ? 0.6 : 0.05
            )
            .style('stroke-width', (l: LinkData) =>
              l.source === d.data.path || l.target === d.data.path ? 1.5 : 0.5
            );

          // Show tooltip with file info
          const defCount = d.data.definitions?.length || 0;
          this.tooltip
            .style('opacity', 1)
            .html(`<strong>${d.data.name}</strong><br>
                   Definitions: ${defCount}<br>
                   Path: ${d.data.path}`)
            .style('left', (event.pageX + 10) + 'px')
            .style('top', (event.pageY - 10) + 'px');
        }
      })
      .on('mouseout', () => {
        linkElements
          .style('stroke-opacity', 0.2)
          .style('stroke-width', 0.5);
        this.tooltip.style('opacity', 0);
      });

    // Add labels for directories and large files
    nodes.append('text')
      .text(d => {
        // Only show labels for directories and files with sufficient radius
        if (d.data.type === 'directory' || d.r > 10) {
          const name = d.data.name;
          if (d.r < 20) {
            return name.length > 5 ? name.substring(0, 4) + '.' : name;
          } else if (d.r < 40) {
            return name.length > 10 ? name.substring(0, 8) + '..' : name;
          }
          return name.length > 20 ? name.substring(0, 18) + '..' : name;
        }
        return '';
      })
      .attr('dy', d => d.data.type === 'directory' ? -d.r - 3 : 3)
      .style('text-anchor', 'middle')
      .style('font-size', d => {
        if (d.depth === 0) return '14px';
        if (d.depth === 1) return '12px';
        if (d.depth === 2) return '10px';
        if (d.r > 20) return '9px';
        if (d.r > 10) return '7px';
        return '6px';
      })
      .style('font-family', 'monospace')
      .style('font-weight', d => d.depth <= 1 ? 'bold' : 'normal')
      .style('fill', d => d.data.type === 'directory' ? '#202020' : '#606060')
      .style('pointer-events', 'none');

    // Add definition dots for larger file circles
    nodes.each(function(d) {
      if (d.data.type === 'file' && d.data.definitions && d.r > 5) {
        const element = d3.select(this);
        const numDefs = d.data.definitions.length;

        // Only show dots if circle is large enough and not too many definitions
        if (d.r > 8 && numDefs <= 8 && numDefs > 0) {
          d.data.definitions.forEach((def, i) => {
            const angle = numDefs === 1 ? 0 : (i * 2 * Math.PI) / numDefs;
            const dotRadius = d.r * 0.5;
            const x = numDefs === 1 ? 0 : Math.cos(angle) * dotRadius;
            const y = numDefs === 1 ? 0 : Math.sin(angle) * dotRadius;

            element.append('circle')
              .attr('cx', x)
              .attr('cy', y)
              .attr('r', Math.min(2, d.r * 0.1))
              .style('fill', '#a0a0a0')
              .style('stroke', '#707070')
              .style('stroke-width', 0.3);
          });
        }
      }
    });

    // Update link positions
    linkElements
      .attr('x1', (d: any) => {
        const source = this.nodeMap.get(d.source);
        return source ? source.x + 50 : 0;
      })
      .attr('y1', (d: any) => {
        const source = this.nodeMap.get(d.source);
        return source ? source.y + 50 : 0;
      })
      .attr('x2', (d: any) => {
        const target = this.nodeMap.get(d.target);
        return target ? target.x + 50 : 0;
      })
      .attr('y2', (d: any) => {
        const target = this.nodeMap.get(d.target);
        return target ? target.y + 50 : 0;
      });

    // Auto-zoom to fit content
    const bounds = nodeGroup.node()?.getBBox();
    if (bounds) {
      const fullWidth = bounds.width + 100;
      const fullHeight = bounds.height + 100;
      const midX = bounds.x + bounds.width / 2;
      const midY = bounds.y + bounds.height / 2;
      const scale = 0.9 / Math.max(fullWidth / this.width, fullHeight / this.height);
      const translate = [this.width / 2 - scale * midX, this.height / 2 - scale * midY];

      this.svg.call(
        d3.zoom<SVGSVGElement, unknown>().transform as any,
        d3.zoomIdentity.translate(translate[0], translate[1]).scale(scale)
      );
    }
  }

  private updateStats(data: ImportStructure) {
    const totalFiles = Object.keys(data).length;
    const totalDefs = Object.values(data).reduce((acc, file) => acc + file.defines.length, 0);
    const totalLinks = Object.values(data).reduce((acc, file) => acc + file.uses.length, 0);

    // Count directories
    const directories = new Set<string>();
    Object.keys(data).forEach(path => {
      const parts = path.split('/');
      for (let i = 1; i < parts.length; i++) {
        directories.add(parts.slice(0, i).join('/'));
      }
    });

    const statsContent = document.getElementById('stats-content');
    if (statsContent) {
      statsContent.innerHTML = `
        <strong>Directories:</strong> ${directories.size}<br>
        <strong>Files:</strong> ${totalFiles}<br>
        <strong>Definitions:</strong> ${totalDefs}<br>
        <strong>Import Links:</strong> ${totalLinks}<br>
        <br>
        <small>Zoom in to see nested structures<br>
        Hover over files for details</small>
      `;
    }
  }
}

// Initialize the visualization when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
  const graph = new NestedPackVisualization('#graph-container');
  graph.loadData();
});