#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Real-time graph database visualization server.

Usage:
    python -m dimos.perception.experimental.temporal_memory.graph_viz_server <db_path>

Then open http://localhost:8080 in your browser.
"""

from pathlib import Path
import sys
from threading import Lock
import time
from typing import Any

from flask import Flask, jsonify, render_template_string

from dimos.perception.experimental.temporal_memory.entity_graph_db import EntityGraphDB

app = Flask(__name__)

_db: EntityGraphDB | None = None
_db_path: Path | None = None
_output_dir: Path | None = None
_db_lock = Lock()
_last_update = 0.0

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Temporal Memory Graph Visualization</title>
    <script type="text/javascript" src="https://unpkg.com/vis-network/standalone/umd/vis-network.min.js"></script>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: 'Gill Sans', 'Gill Sans MT', Calibri, 'Trebuchet MS', sans-serif;
            background: #ffffff;
            color: #24292f;
            font-size: 13px;
        }
        #header {
            background: #f6f8fa;
            color: #24292f;
            padding: 8px 12px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 1px solid #d0d7de;
        }
        #header h2 {
            font-size: 14px;
            font-weight: 600;
        }
        #stats {
            display: flex;
            gap: 10px;
            font-size: 12px;
        }
        #main-container {
            display: flex;
            height: calc(100vh - 35px);
        }
        #mynetwork {
            flex: 1;
            background: #ffffff;
        }
        #sidebar {
            width: 240px;
            background: #f6f8fa;
            color: #24292f;
            padding: 10px;
            overflow-y: auto;
            border-left: 1px solid #d0d7de;
        }
        .stat {
            background: #ffffff;
            padding: 4px 8px;
            border-radius: 3px;
            border: 1px solid #d0d7de;
            font-size: 11px;
        }
        .toggle-group {
            margin-bottom: 12px;
            padding-bottom: 10px;
            border-bottom: 1px solid #d0d7de;
        }
        .toggle-group:last-child {
            border-bottom: none;
        }
        .toggle-group h3 {
            margin-bottom: 6px;
            font-size: 11px;
            color: #656d76;
            font-weight: 600;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        .toggle-btn {
            display: block;
            width: 100%;
            padding: 6px 8px;
            margin-bottom: 4px;
            background: #ffffff;
            border: 1px solid #d0d7de;
            color: #24292f;
            cursor: pointer;
            border-radius: 4px;
            transition: all 0.15s;
            font-size: 12px;
            font-weight: 500;
        }
        .toggle-btn:hover {
            background: #f3f4f6;
            border-color: #0969da;
        }
        .toggle-btn.active {
            background: #0969da;
            border-color: #0969da;
            color: #ffffff;
        }
        .legend-item {
            display: flex;
            align-items: center;
            margin-bottom: 6px;
            padding: 4px;
            border-radius: 3px;
        }
        .legend-color {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            margin-right: 8px;
            border: 1px solid #30363d;
        }
        .legend-label {
            font-size: 12px;
        }
        #sidebar::-webkit-scrollbar {
            width: 6px;
        }
        #sidebar::-webkit-scrollbar-track {
            background: #f6f8fa;
        }
        #sidebar::-webkit-scrollbar-thumb {
            background: #d0d7de;
            border-radius: 3px;
        }
        #sidebar::-webkit-scrollbar-thumb:hover {
            background: #656d76;
        }
        /* Vis.js navigation buttons styling - light mode */
        .vis-navigation {
            background: rgba(255, 255, 255, 0.95) !important;
            border: 1px solid #d0d7de !important;
            border-radius: 4px !important;
            box-shadow: 0 1px 3px rgba(0,0,0,0.1) !important;
        }
        .vis-button {
            background: rgba(255, 255, 255, 0.95) !important;
            border: 1px solid #d0d7de !important;
            color: #24292f !important;
        }
        .vis-button:hover {
            background: rgba(246, 248, 250, 0.95) !important;
            border-color: #0969da !important;
        }
        .vis-button:active {
            background: rgba(9, 105, 218, 0.1) !important;
        }
        .vis-button svg,
        .vis-button path,
        .vis-button line,
        .vis-button circle,
        .vis-button polygon {
            fill: #24292f !important;
            stroke: #24292f !important;
        }
        .vis-button:hover svg,
        .vis-button:hover path,
        .vis-button:hover line,
        .vis-button:hover circle,
        .vis-button:hover polygon {
            fill: #0969da !important;
            stroke: #0969da !important;
        }
    </style>
</head>
<body>
    <div id="header">
        <h2 style="margin: 0;">Temporal Memory Graph DB</h2>
        <div id="stats">
            <div class="stat">Entities: <span id="entity-count">0</span></div>
            <div class="stat">Relations: <span id="relation-count">0</span></div>
            <div class="stat">Distances: <span id="distance-count">0</span></div>
            <div class="stat">Last Update: <span id="last-update">Never</span></div>
            <div class="stat" id="waiting-msg" style="display: none; background: #fff4e6; border-color: #d4a574; color: #8b6914;">⏳ Waiting for database...</div>
        </div>
    </div>
    <div id="main-container">
        <div id="mynetwork"></div>
        <div id="sidebar">
            <div class="toggle-group">
                <h3>Display Options</h3>
                <button class="toggle-btn active" id="toggle-relations">Show Relations</button>
                <button class="toggle-btn active" id="toggle-distances">Show Distances</button>
                <button class="toggle-btn active" id="toggle-time">Show Time Info</button>
            </div>
            <div class="toggle-group">
                <h3>Entity Types</h3>
                <div class="legend-item">
                    <div class="legend-color" style="background: #ff6b6b;"></div>
                    <div class="legend-label">Person</div>
                </div>
                <div class="legend-item">
                    <div class="legend-color" style="background: #4ecdc4;"></div>
                    <div class="legend-label">Object</div>
                </div>
                <div class="legend-item">
                    <div class="legend-color" style="background: #95e1d3;"></div>
                    <div class="legend-label">Location</div>
                </div>
                <div class="legend-item">
                    <div class="legend-color" style="background: #8b949e;"></div>
                    <div class="legend-label">Unknown</div>
                </div>
            </div>
            <div class="toggle-group">
                <h3>Edge Types</h3>
                <div class="legend-item">
                    <div style="width: 20px; height: 2px; background: #7f8c8d; margin-right: 8px;"></div>
                    <div class="legend-label">Relations</div>
                </div>
                <div class="legend-item">
                    <div style="width: 20px; height: 2px; border-top: 2px dashed #27ae60; margin-right: 8px;"></div>
                    <div class="legend-label">Distances</div>
                </div>
            </div>
        </div>
    </div>

    <script>
        let network, nodes, edges;
        let nodePositions = {};
        let viewPosition = null;
        let viewScale = 1.0;
        let showRelations = true;
        let showDistances = true;
        let showTimeInfo = true;
        let physicsEnabled = true;
        const container = document.getElementById('mynetwork');

        const options = {
            nodes: {
                shape: 'dot',
                size: 30,
                font: {
                    size: 16,
                    color: '#24292f',
                    face: 'Gill Sans, Gill Sans MT, Calibri, Trebuchet MS, sans-serif',
                    bold: '600'
                },
                borderWidth: 3,
                borderWidthSelected: 4,
                shadow: {
                    enabled: true,
                    color: 'rgba(0,0,0,0.1)',
                    size: 8,
                    x: 0,
                    y: 2
                },
            },
            edges: {
                width: 2,
                arrows: {
                    to: {
                        enabled: true,
                        scaleFactor: 0.7
                    }
                },
                smooth: {
                    type: 'continuous',
                    roundness: 0.5
                },
                font: {
                    size: 11,
                    color: '#24292f',
                    background: 'rgba(255,255,255,0.8)',
                    strokeWidth: 1,
                    strokeColor: '#ffffff'
                },
                shadow: false
            },
            physics: {
                enabled: true,
                stabilization: {
                    iterations: 250,
                    fit: true
                },
                barnesHut: {
                    gravitationalConstant: -2000,
                    springLength: 200,
                    springConstant: 0.04,
                    avoidOverlap: 0.5
                },
            },
            interaction: {
                hover: true,
                tooltipDelay: 100,
                dragNodes: true,
                dragView: true,
                zoomView: true,
                navigationButtons: true,
                keyboard: true
            },
        };

        function formatTime(seconds) {
            if (!seconds) return 'N/A';
            const mins = Math.floor(seconds / 60);
            const secs = Math.floor(seconds % 60);
            return `${mins}:${secs.toString().padStart(2, '0')}`;
        }

        function updateGraph(data) {
            // Show/hide waiting message
            const waitingMsg = document.getElementById('waiting-msg');
            if (data.waiting) {
                if (waitingMsg) waitingMsg.style.display = 'block';
                // Don't update graph when waiting
                return;
            } else {
                if (waitingMsg) waitingMsg.style.display = 'none';
            }

            const nodeMap = new Map();
            const edgeList = [];
            const edgeSet = new Set();

            // Save current positions AND view position before updating
            if (network) {
                const positions = network.getPositions();
                Object.keys(positions).forEach(id => {
                    nodePositions[id] = positions[id];
                });
                // Save view position and scale
                viewPosition = network.getViewPosition();
                viewScale = network.getScale();
            }

            // Add entities as nodes
            data.entities.forEach(e => {
                // Clean label - just show descriptor, keep it short
                const descriptor = (e.descriptor || 'unknown').trim();
                const shortDescriptor = descriptor.length > 20 ? descriptor.substring(0, 18) + '...' : descriptor;

                // Build detailed tooltip with all info (using newlines instead of <br> for vis.js)
                let tooltip = `${e.entity_type.toUpperCase()}: ${descriptor}`;
                tooltip += `\\nID: ${e.entity_id}`;
                if (showTimeInfo && (e.first_seen_ts || e.last_seen_ts)) {
                    tooltip += `\\nFirst seen: ${formatTime(e.first_seen_ts)}`;
                    tooltip += `\\nLast seen: ${formatTime(e.last_seen_ts)}`;
                    const duration = (e.last_seen_ts || 0) - (e.first_seen_ts || 0);
                    if (duration > 0) {
                        tooltip += `\\nDuration: ${formatTime(duration)}`;
                    }
                }

                const node = {
                    id: e.entity_id,
                    label: shortDescriptor,
                    title: tooltip,
                    color: getColorForType(e.entity_type),
                };

                // Restore position if available
                if (nodePositions[e.entity_id]) {
                    node.x = nodePositions[e.entity_id].x;
                    node.y = nodePositions[e.entity_id].y;
                    node.fixed = { x: false, y: false };
                }

                nodeMap.set(e.entity_id, node);
            });

            // Add relations as edges
            if (showRelations) {
                data.relations.forEach(r => {
                    const key = `${r.subject_id}-${r.relation_type}-${r.object_id}`;
                    if (!edgeSet.has(key) && nodeMap.has(r.subject_id) && nodeMap.has(r.object_id)) {
                        edgeSet.add(key);
                        const relationType = r.relation_type || 'related';
                        edgeList.push({
                            id: key,
                            from: r.subject_id,
                            to: r.object_id,
                            label: relationType.length > 15 ? relationType.substring(0, 13) + '...' : relationType,
                            title: `${relationType}\\nConfidence: ${(r.confidence || 1.0).toFixed(2)}`,
                            color: { color: '#656d76', opacity: 0.8 },
                            hidden: !showRelations,
                        });
                    }
                });
            }

            // Add distances as edges (dashed)
            if (showDistances) {
                data.distances.forEach(d => {
                    const key = `dist-${d.entity_a_id}-${d.entity_b_id}`;
                    if (!edgeSet.has(key) && nodeMap.has(d.entity_a_id) && nodeMap.has(d.entity_b_id)) {
                        edgeSet.add(key);
                        const distLabel = d.distance_meters ? `${d.distance_meters.toFixed(1)}m` : (d.distance_category || '?');
                        edgeList.push({
                            id: key,
                            from: d.entity_a_id,
                            to: d.entity_b_id,
                            label: distLabel,
                            title: `Distance\\n${d.distance_category || 'unknown'}${d.distance_meters ? '\\n' + d.distance_meters.toFixed(2) + ' meters' : ''}`,
                            dashes: [5, 5],
                            color: { color: '#1a7f37', opacity: 0.7 },
                            hidden: !showDistances,
                            arrows: { to: { enabled: false } }, // No arrows for distance edges
                        });
                    }
                });
            }

            nodes = new vis.DataSet(Array.from(nodeMap.values()));
            edges = new vis.DataSet(edgeList);

            const graphData = { nodes, edges };
            if (network) {
                // Save current view before setData (which might reset it)
                const savedView = { position: viewPosition, scale: viewScale };

                // Update data - keep physics enabled for smooth interactions
                network.setData(graphData);

                // Restore view position immediately without animation to prevent flashing
                if (savedView.position && savedView.scale) {
                    // Use setTimeout with 0 delay to ensure it runs after setData completes
                    setTimeout(() => {
                        network.moveTo({
                            position: savedView.position,
                            scale: savedView.scale,
                            animation: false
                        });
                    }, 0);
                }
            } else {
                network = new vis.Network(container, graphData, options);
                // Keep physics enabled for better node interactions
                network.once('stabilizationIterationsDone', () => {
                    // Physics stays enabled for dragging and smooth layout
                });
            }

            // Update stats
            document.getElementById('entity-count').textContent = data.stats.entities || 0;
            document.getElementById('relation-count').textContent = data.stats.relations || 0;
            document.getElementById('distance-count').textContent = data.stats.distances || 0;
            document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
        }

        function getColorForType(type) {
            const colors = {
                person: {
                    background: '#ff6b6b',
                    border: '#d1242f',
                    highlight: { background: '#ff8787', border: '#ff6b6b' },
                    hover: { background: '#ff8787', border: '#ff6b6b' }
                },
                object: {
                    background: '#4ecdc4',
                    border: '#0969da',
                    highlight: { background: '#6eddd6', border: '#4ecdc4' },
                    hover: { background: '#6eddd6', border: '#4ecdc4' }
                },
                location: {
                    background: '#95e1d3',
                    border: '#1a7f37',
                    highlight: { background: '#adeee3', border: '#95e1d3' },
                    hover: { background: '#adeee3', border: '#95e1d3' }
                },
                unknown: {
                    background: '#d0d7de',
                    border: '#656d76',
                    highlight: { background: '#e1e4e8', border: '#d0d7de' },
                    hover: { background: '#e1e4e8', border: '#d0d7de' }
                }
            };
            return colors[type?.toLowerCase()] || colors.unknown;
        }

        // Toggle buttons
        document.getElementById('toggle-relations').addEventListener('click', function() {
            showRelations = !showRelations;
            this.classList.toggle('active');
            if (network && edges) {
                const updates = edges.get().map(edge => {
                    if (edge.id && !edge.id.startsWith('dist-')) {
                        return { id: edge.id, hidden: !showRelations };
                    }
                    return null;
                }).filter(x => x !== null);
                if (updates.length > 0) {
                    edges.update(updates);
                }
            }
        });

        document.getElementById('toggle-distances').addEventListener('click', function() {
            showDistances = !showDistances;
            this.classList.toggle('active');
            if (network && edges) {
                const updates = edges.get().map(edge => {
                    if (edge.id && edge.id.startsWith('dist-')) {
                        return { id: edge.id, hidden: !showDistances };
                    }
                    return null;
                }).filter(x => x !== null);
                if (updates.length > 0) {
                    edges.update(updates);
                }
            }
        });

        document.getElementById('toggle-time').addEventListener('click', function() {
            showTimeInfo = !showTimeInfo;
            this.classList.toggle('active');
            // Force graph update to refresh labels
            poll();
        });

        // Poll for updates every 1 second (reduced from 500ms for smoother experience)
        async function poll() {
            try {
                const res = await fetch('/api/graph');
                const data = await res.json();
                updateGraph(data);
            } catch (e) {
                console.error('Poll error:', e);
            }
        }

        nodes = new vis.DataSet([]);
        edges = new vis.DataSet([]);
        poll();
        setInterval(poll, 1000);
    </script>
</body>
</html>
"""


@app.route("/")
def index() -> str:
    return render_template_string(HTML_TEMPLATE)


def _try_init_db() -> bool:
    """Try to initialize the database if the file exists."""
    global _db, _db_path

    with _db_lock:
        if _db is not None:
            return True

        if _db_path is None or not _db_path.exists():
            return False

        try:
            _db = EntityGraphDB(db_path=_db_path)
            return True
        except Exception as e:
            print(f"Warning: Failed to initialize database: {e}")
            return False


@app.route("/api/graph")
def get_graph() -> Any:
    """Get current graph state."""
    global _last_update

    # Try to initialize DB if not already initialized
    if not _try_init_db():
        # Return empty data when waiting for DB
        return jsonify(
            {
                "stats": {"entities": 0, "relations": 0, "distances": 0},
                "entities": [],
                "relations": [],
                "distances": [],
                "waiting": True,
            }
        )

    with _db_lock:
        if not _db:
            return jsonify(
                {
                    "stats": {"entities": 0, "relations": 0, "distances": 0},
                    "entities": [],
                    "relations": [],
                    "distances": [],
                    "waiting": True,
                }
            )

        stats = _db.get_stats()
        entities = _db.get_all_entities()
        recent_relations = _db.get_recent_relations(limit=100)

        # Get all distances (latest per pair)
        distances = []
        entity_ids = [e["entity_id"] for e in entities]
        for i, e1 in enumerate(entity_ids):
            for e2 in entity_ids[i + 1 :]:
                dist = _db.get_distance(e1, e2)
                if dist:
                    distances.append(dist)

        _last_update = time.time()

        return jsonify(
            {
                "stats": stats,
                "entities": entities,
                "relations": recent_relations,
                "distances": distances,
                "waiting": False,
            }
        )


def main() -> None:
    """Run the visualization server."""
    global _db_path, _output_dir

    if len(sys.argv) < 2:
        print(
            "Usage: python -m dimos.perception.experimental.temporal_memory.graph_viz_server <db_path>"
        )
        print(
            "Example: python -m dimos.perception.experimental.temporal_memory.graph_viz_server assets/temporal_memory/entity_graph.db"
        )
        sys.exit(1)

    db_path = Path(sys.argv[1])
    _db_path = db_path
    # Infer output_dir from db_path (db is in output_dir/entity_graph.db)
    _output_dir = db_path.parent

    # Try to initialize DB if file exists, but don't fail if it doesn't
    if db_path.exists():
        if _try_init_db():
            print(f"✅ Database loaded: {db_path}")
        else:
            print(f"⚠️  Database file exists but couldn't be opened: {db_path}")
    else:
        print(f"⏳ Waiting for database file: {db_path}")
        print("   (The server will start and wait for the file to appear)")

    print("🚀 Graph visualization server starting...")
    print(f"📊 Database path: {db_path}")
    print("🌐 Open http://localhost:8080 in your browser")
    print("Press Ctrl+C to stop")

    app.run(host="127.0.0.1", port=8080, debug=False, threaded=True)


if __name__ == "__main__":
    main()
