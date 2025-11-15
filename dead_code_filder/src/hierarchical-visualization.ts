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
  size?: number;
  x?: number;
  y?: number;
  r?: number;
  vx?: number;
  vy?: number;
  fx?: number | null;
  fy?: number | null;
  parent?: HierarchyNode;
  depth?: number;
}

interface LinkData {
  source: string;
  target: string;
  definition: string;
}

class HierarchicalDependencyGraph {
  private width: number = window.innerWidth;
  private height: number = window.innerHeight;
  private svg: d3.Selection<SVGSVGElement, unknown, HTMLElement, any>;
  private g: d3.Selection<SVGGElement, unknown, HTMLElement, any>;
  private simulation: d3.Simulation<HierarchyNode, undefined>;
  private tooltip: d3.Selection<HTMLDivElement, unknown, HTMLElement, any>;
  private importData: ImportStructure = {};
  private links: LinkData[] = [];
  private nodeMap: Map<string, HierarchyNode> = new Map();

  // ============================================================================
  // ADJUST THIS VALUE TO CONTROL SPACING
  // 1.0 = default spacing
  // 1.5 = 50% more spacing
  // 2.0 = double spacing (current setting - good for lower level directories)
  // 2.5 = even more spacious
  // 0.5 = half spacing (more compact)
  // This affects padding between all directories and files at all levels
  // ============================================================================
  private SPACING_MULTIPLIER: number = 2.0;

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
      .scaleExtent([0.1, 4])
      .on('zoom', (event) => {
        this.g.attr('transform', event.transform.toString());
      });

    this.svg.call(zoom);

    // Create main group for transformations
    this.g = this.svg.append('g');

    // Initialize simulation with gentler forces
    this.simulation = d3.forceSimulation<HierarchyNode>()
      .force('charge', d3.forceManyBody().strength(-30).distanceMax(100))
      .force('center', d3.forceCenter(this.width / 2, this.height / 2))
      .alphaDecay(0.02);  // Slower decay for better settling
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
      name: 'dimos',
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
            size: 1 + info.defines.length * 0.5
          };
          if (!currentNode.children) currentNode.children = [];
          currentNode.children.push(fileNode);
          this.nodeMap.set(currentPath, fileNode);
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
            this.nodeMap.set(currentPath, dirNode);
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
      .sum((d: any) => d.size || 1)
      .sort((a, b) => (b.value || 0) - (a.value || 0));

    // Use pack layout for initial positioning with more spacing
    const pack = d3.pack<HierarchyNode>()
      .size([this.width - 50, this.height - 50])
      .padding((d) => {
        // Base padding values multiplied by spacing multiplier
        if (d.depth === 0) return 80 * this.SPACING_MULTIPLIER;  // Space around root
        if (d.depth === 1) return 50 * this.SPACING_MULTIPLIER;  // Space between top-level directories
        if (d.depth === 2) return 25 * this.SPACING_MULTIPLIER;  // Space between subdirectories
        return 15 * this.SPACING_MULTIPLIER;  // Space between files
      });

    const packedRoot = pack(root) as any;

    // Store hierarchy information in nodes
    const nodes: HierarchyNode[] = [];
    packedRoot.each((d: any) => {
      const node = d.data;
      node.x = d.x + 25;  // Offset from edge
      node.y = d.y + 25;
      node.r = d.r;
      node.depth = d.depth;
      node.parent = d.parent?.data;
      nodes.push(node);
    });

    // Create groups for each level
    const linkGroup = this.g.append('g').attr('class', 'links');
    const directoryGroup = this.g.append('g').attr('class', 'directories');
    const fileGroup = this.g.append('g').attr('class', 'files');

    // Draw links first (behind everything)
    const linkElements = linkGroup.selectAll('line')
      .data(this.links)
      .enter().append('line')
      .style('stroke', '#00ff00')
      .style('stroke-opacity', 0.3)
      .style('stroke-width', 1);

    // Draw directories (static, not part of simulation)
    const directories = nodes.filter(d => d.type === 'directory');
    const directoryElements = directoryGroup.selectAll<SVGGElement, HierarchyNode>('.directory')
      .data(directories)
      .enter().append('g')
      .attr('class', 'directory')
      .attr('transform', d => `translate(${d.x},${d.y})`);

    directoryElements.append('circle')
      .attr('r', d => d.r || 20)
      .style('fill', 'none')
      .style('stroke', d => {
        const colors = ['#202020', '#404040', '#606060', '#808080'];
        return colors[Math.min(d.depth || 0, colors.length - 1)];
      })
      .style('stroke-width', d => Math.max(1.5, 4 - (d.depth || 0)))
      .style('stroke-dasharray', d => d.depth === 0 ? 'none' : '5,5')
      .style('fill-opacity', 0.05);

    directoryElements.append('text')
      .text(d => d.name)
      .attr('y', d => -(d.r || 20) - 5)
      .style('text-anchor', 'middle')
      .style('font-size', d => Math.max(10, 14 - (d.depth || 0) * 2) + 'px')
      .style('font-family', 'monospace')
      .style('font-weight', d => d.depth === 1 ? 'bold' : 'normal')
      .style('fill', '#202020');

    // Draw files (part of simulation)
    const files = nodes.filter(d => d.type === 'file');
    const fileElements = fileGroup.selectAll<SVGGElement, HierarchyNode>('.file')
      .data(files)
      .enter().append('g')
      .attr('class', 'file')
      .call(this.createDragBehavior());

    fileElements.append('circle')
      .attr('r', d => Math.max(8, Math.min(25, 8 + (d.definitions?.length || 0) * 1.5)))
      .style('fill', '#505050')
      .style('stroke', '#303030')
      .style('stroke-width', 1.5)
      .style('fill-opacity', 0.8)
      .on('mouseover', (event, d) => {
        linkElements
          .style('stroke-opacity', (l: LinkData) =>
            l.source === d.path || l.target === d.path ? 0.8 : 0.1
          )
          .style('stroke-width', (l: LinkData) =>
            l.source === d.path || l.target === d.path ? 2 : 1
          );
      })
      .on('mouseout', () => {
        linkElements
          .style('stroke-opacity', 0.3)
          .style('stroke-width', 1);
      });

    // Add file name labels
    fileElements.append('text')
      .text(d => {
        const name = d.name;
        return name.length > 12 ? name.substring(0, 10) + '..' : name;
      })
      .attr('y', d => Math.max(8, Math.min(25, 8 + (d.definitions?.length || 0) * 1.5)) + 10)
      .style('text-anchor', 'middle')
      .style('font-size', '8px')
      .style('font-family', 'monospace')
      .style('fill', '#202020');

    // Add definition dots for files
    fileElements.each((nodeData, nodeIndex, nodeList) => {
      if (!nodeData.definitions || nodeData.definitions.length === 0) return;

      const element = d3.select(nodeList[nodeIndex]);
      const numDefs = nodeData.definitions.length;
      const radius = Math.max(8, Math.min(25, 8 + numDefs * 1.5));

      // Only show dots for files with few definitions
      if (numDefs <= 8) {
        nodeData.definitions.forEach((def, i) => {
          const angle = numDefs === 1 ? 0 : (i * 2 * Math.PI) / numDefs;
          const dotRadius = Math.min(radius - 4, radius * 0.6);
          const x = numDefs === 1 ? 0 : Math.cos(angle) * dotRadius;
          const y = numDefs === 1 ? 0 : Math.sin(angle) * dotRadius;

          const dotGroup = element.append('g')
            .attr('transform', `translate(${x}, ${y})`);

          dotGroup.append('circle')
            .attr('r', 2)
            .style('fill', '#a0a0a0')
            .style('stroke', '#707070')
            .style('stroke-width', 0.5)
            .on('mouseover', (event) => {
              this.tooltip
                .style('opacity', 1)
                .html(`${def}<br><small>${nodeData.path}</small>`)
                .style('left', (event.pageX + 10) + 'px')
                .style('top', (event.pageY - 10) + 'px');
            })
            .on('mouseout', () => {
              this.tooltip.style('opacity', 0);
            });

          if (numDefs <= 4) {
            const truncatedName = def.length > 4 ? def.substring(0, 3) + '.' : def;
            dotGroup.append('text')
              .text(truncatedName)
              .attr('x', 3)
              .attr('y', 2)
              .style('font-size', '6px')
              .style('font-family', 'monospace')
              .style('fill', '#404040')
              .style('pointer-events', 'none');
          }
        });
      }
    });

    // Set up force simulation for files only
    this.simulation
      .nodes(files)
      .force('collision', d3.forceCollide<HierarchyNode>()
        .radius(d => (Math.max(8, Math.min(25, 8 + (d.definitions?.length || 0) * 1.5))) + (8 * this.SPACING_MULTIPLIER))  // Space between files
        .strength(0.7))
      .on('tick', () => {
        // Constrain files to their parent directories
        files.forEach(node => {
          if (node.parent && node.parent.x !== undefined && node.parent.y !== undefined && node.parent.r !== undefined) {
            const parentX = node.parent.x;
            const parentY = node.parent.y;
            const parentR = node.parent.r;

            // Calculate distance from parent center
            const dx = (node.x || 0) - parentX;
            const dy = (node.y || 0) - parentY;
            const distance = Math.sqrt(dx * dx + dy * dy);

            // If outside parent boundary, constrain it
            const nodeRadius = Math.max(8, Math.min(25, 8 + (node.definitions?.length || 0) * 1.5));
            const maxDistance = parentR - nodeRadius - (12 * this.SPACING_MULTIPLIER); // Padding from boundary

            if (distance > maxDistance && distance > 0) {
              const ratio = maxDistance / distance;
              node.x = parentX + dx * ratio;
              node.y = parentY + dy * ratio;
            }
          }
        });

        // Update file positions
        fileElements.attr('transform', d => `translate(${d.x},${d.y})`);

        // Update links
        linkElements
          .attr('x1', (d: any) => {
            const source = this.nodeMap.get(d.source);
            return source?.x || 0;
          })
          .attr('y1', (d: any) => {
            const source = this.nodeMap.get(d.source);
            return source?.y || 0;
          })
          .attr('x2', (d: any) => {
            const target = this.nodeMap.get(d.target);
            return target?.x || 0;
          })
          .attr('y2', (d: any) => {
            const target = this.nodeMap.get(d.target);
            return target?.y || 0;
          });
      });

    // Run simulation to settle positions with good spacing
    this.simulation.alpha(0.8).restart();
  }

  private createDragBehavior() {
    const dragstarted = (event: any, d: HierarchyNode) => {
      if (!event.active) this.simulation.alphaTarget(0.3).restart();
      d.fx = d.x;
      d.fy = d.y;
    };

    const dragged = (event: any, d: HierarchyNode) => {
      d.fx = event.x;
      d.fy = event.y;
    };

    const dragended = (event: any, d: HierarchyNode) => {
      if (!event.active) this.simulation.alphaTarget(0);
      d.fx = null;
      d.fy = null;
    };

    return d3.drag<SVGGElement, HierarchyNode>()
      .on('start', dragstarted)
      .on('drag', dragged)
      .on('end', dragended);
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
        <small>Files are contained within directories<br>
        Hover over files to see connections</small>
      `;
    }
  }
}

// Initialize the visualization when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
  const graph = new HierarchicalDependencyGraph('#graph-container');
  graph.loadData();
});
