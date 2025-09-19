#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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
Force-Torque Sensor Visualization Module for Dimos

Visualizes calibrated force-torque sensor data using Dash and Plotly.
Receives force and torque Vector3 messages via LCM transport.
"""

from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)

import dash
from dash import dcc, html, Input, Output
import plotly.graph_objs as go
import time
import threading
import subprocess
import sys
import numpy as np
from collections import deque
from typing import Dict, Any

from dimos.core import Module, In, rpc
from dimos.msgs.geometry_msgs import Vector3


class FTVisualizerModule(Module):
    """Force-Torque sensor visualization module using Dash."""

    # Input ports - separate force and torque Vector3 streams
    force: In[Vector3] = None  # Force vector in Newtons
    torque: In[Vector3] = None  # Torque vector in Newton-meters

    def __init__(
        self,
        max_history: int = 500,
        update_interval_ms: int = 100,
        dash_port: int = 8052,
        dash_host: str = "0.0.0.0",
        verbose: bool = False,
    ):
        """
        Initialize the FT visualizer module.

        Args:
            max_history: Maximum number of data points to keep in history
            update_interval_ms: Dashboard update interval in milliseconds
            dash_port: Port for Dash web server
            dash_host: Host address for Dash web server
            verbose: Enable verbose output
        """
        super().__init__()

        self.max_history = max_history
        self.update_interval_ms = update_interval_ms
        self.dash_port = dash_port
        self.dash_host = dash_host
        self.verbose = verbose

        # Data storage with deques for efficient append/pop
        self.timestamps = deque(maxlen=max_history)
        self.forces = {
            "x": deque(maxlen=max_history),
            "y": deque(maxlen=max_history),
            "z": deque(maxlen=max_history),
        }
        self.torques = {
            "x": deque(maxlen=max_history),
            "y": deque(maxlen=max_history),
            "z": deque(maxlen=max_history),
        }
        self.force_magnitudes = deque(maxlen=max_history)
        self.torque_magnitudes = deque(maxlen=max_history)

        # Latest values for display
        self.latest_force = [0, 0, 0]
        self.latest_torque = [0, 0, 0]
        self.latest_force_mag = 0
        self.latest_torque_mag = 0

        # Statistics
        self.force_count = 0
        self.torque_count = 0
        self.start_time = None

        # Dash app - initialize immediately to ensure callbacks are registered
        self.app = None
        self.dash_thread = None
        self.running = False

        # Lock for thread-safe data access
        self.data_lock = threading.Lock()

    def handle_force(self, msg: Vector3):
        """Handle incoming force Vector3 data."""
        self.force_count += 1

        if self.start_time is None:
            self.start_time = time.time()

        # Calculate relative timestamp
        rel_time = time.time() - self.start_time

        # Extract force values
        force = [msg.x, msg.y, msg.z]
        force_mag = np.linalg.norm(force)

        with self.data_lock:
            # Store data
            self.timestamps.append(rel_time)
            self.forces["x"].append(force[0])
            self.forces["y"].append(force[1])
            self.forces["z"].append(force[2])
            self.force_magnitudes.append(force_mag)

            # Update latest values
            self.latest_force = force
            self.latest_force_mag = force_mag

        if self.verbose:
            logger.debug(f"Force: F=({force[0]:.2f}, {force[1]:.2f}, {force[2]:.2f}) N, |F|={force_mag:.2f} N")

    def handle_torque(self, msg: Vector3):
        """Handle incoming torque Vector3 data."""
        self.torque_count += 1

        if self.start_time is None:
            self.start_time = time.time()

        # Calculate relative timestamp
        rel_time = time.time() - self.start_time

        # Extract torque values
        torque = [msg.x, msg.y, msg.z]
        torque_mag = np.linalg.norm(torque)

        with self.data_lock:
            # Store torque data (align with force timestamps if needed)
            if len(self.torques["x"]) < len(self.forces["x"]):
                self.torques["x"].append(torque[0])
                self.torques["y"].append(torque[1])
                self.torques["z"].append(torque[2])
                self.torque_magnitudes.append(torque_mag)
            else:
                # If we get more torques than forces, update the last one
                if len(self.torques["x"]) > 0:
                    self.torques["x"][-1] = torque[0]
                    self.torques["y"][-1] = torque[1]
                    self.torques["z"][-1] = torque[2]
                    self.torque_magnitudes[-1] = torque_mag

            # Update latest values
            self.latest_torque = torque
            self.latest_torque_mag = torque_mag

        if self.verbose:
            logger.debug(f"Torque: T=({torque[0]:.4f}, {torque[1]:.4f}, {torque[2]:.4f}) N⋅m, |T|={torque_mag:.4f} N⋅m")

    def get_plot_data(self) -> Dict[str, Any]:
        """Get data formatted for plotting."""
        with self.data_lock:
            # Ensure torques have same length as forces by padding with zeros
            while len(self.torques["x"]) < len(self.forces["x"]):
                self.torques["x"].append(self.latest_torque[0] if self.latest_torque else 0)
                self.torques["y"].append(self.latest_torque[1] if self.latest_torque else 0)
                self.torques["z"].append(self.latest_torque[2] if self.latest_torque else 0)
                self.torque_magnitudes.append(self.latest_torque_mag)

            return {
                "timestamps": list(self.timestamps),
                "forces": {k: list(v) for k, v in self.forces.items()},
                "torques": {k: list(v) for k, v in self.torques.items()},
                "force_magnitudes": list(self.force_magnitudes),
                "torque_magnitudes": list(self.torque_magnitudes),
                "latest_force": self.latest_force,
                "latest_torque": self.latest_torque,
                "latest_force_mag": self.latest_force_mag,
                "latest_torque_mag": self.latest_torque_mag,
                "force_count": self.force_count,
                "torque_count": self.torque_count,
            }

    def _initialize_dash_app(self):
        """Initialize and configure the Dash application with callbacks."""
        # Suppress all logging from Dash/Flask/Werkzeug
        import logging
        import os
        import warnings

        # Suppress all warnings
        warnings.filterwarnings("ignore")

        # Set environment variables to suppress messages
        os.environ['WERKZEUG_RUN_MAIN'] = 'true'

        # Disable all loggers that might produce output
        logging.getLogger('werkzeug').disabled = True
        logging.getLogger('dash').disabled = True
        logging.getLogger('dash.dash').disabled = True
        logging.getLogger('dash.callback').disabled = True
        logging.getLogger('flask.app').disabled = True
        logging.getLogger(__name__).disabled = True

        # Create logger that suppresses the callback errors
        class SuppressedLogger:
            def error(self, *args, **kwargs):
                pass
            def warning(self, *args, **kwargs):
                pass
            def info(self, *args, **kwargs):
                pass
            def debug(self, *args, **kwargs):
                pass

        # Replace the module logger
        import sys
        sys.modules[__name__].__dict__['logger'] = SuppressedLogger()

        self.app = dash.Dash(__name__, suppress_callback_exceptions=True)

        self.app.layout = html.Div(
            [
                html.H1("Force-Torque Sensor Visualization", style={"text-align": "center"}),
                # Connection status
                html.Div(
                    id="status",
                    style={
                        "text-align": "center",
                        "padding": "10px",
                        "background-color": "#f0f0f0",
                        "margin-bottom": "20px",
                    },
                ),
                # Current values display
                html.Div(
                    [
                        html.Div(
                            [
                                html.H3("Current Forces (N)", style={"text-align": "center"}),
                                html.Pre(
                                    id="current-forces",
                                    style={
                                        "font-family": "monospace",
                                        "font-size": "18px",
                                        "text-align": "center",
                                        "padding": "10px",
                                        "background-color": "#f0f0f0",
                                        "border-radius": "5px",
                                    },
                                ),
                            ],
                            style={"width": "48%", "display": "inline-block", "padding": "10px"},
                        ),
                        html.Div(
                            [
                                html.H3("Current Torques (N⋅m)", style={"text-align": "center"}),
                                html.Pre(
                                    id="current-torques",
                                    style={
                                        "font-family": "monospace",
                                        "font-size": "18px",
                                        "text-align": "center",
                                        "padding": "10px",
                                        "background-color": "#f0f0f0",
                                        "border-radius": "5px",
                                    },
                                ),
                            ],
                            style={"width": "48%", "display": "inline-block", "padding": "10px"},
                        ),
                    ]
                ),
                # Main plots
                html.Div(
                    [
                        # Force components plot
                        dcc.Graph(id="force-plot", style={"height": "400px"}),
                        # Torque components plot
                        dcc.Graph(id="torque-plot", style={"height": "400px"}),
                        # Magnitude plots
                        html.Div(
                            [
                                dcc.Graph(
                                    id="force-magnitude-plot",
                                    style={"width": "50%", "display": "inline-block", "height": "300px"},
                                ),
                                dcc.Graph(
                                    id="torque-magnitude-plot",
                                    style={"width": "50%", "display": "inline-block", "height": "300px"},
                                ),
                            ]
                        ),
                    ]
                ),
                # Statistics
                html.Div(
                    [
                        html.H3("Statistics", style={"text-align": "center"}),
                        html.Pre(
                            id="statistics",
                            style={
                                "font-family": "monospace",
                                "padding": "20px",
                                "background-color": "#f9f9f9",
                                "border-radius": "5px",
                            },
                        ),
                    ],
                    style={"padding": "20px"},
                ),
                # Update interval
                dcc.Interval(
                    id="interval-component",
                    interval=self.update_interval_ms,
                    n_intervals=0
                ),
            ]
        )

        # Register callback using explicit method for better multiprocess support
        self.app.callback(
            [
                Output("force-plot", "figure"),
                Output("torque-plot", "figure"),
                Output("force-magnitude-plot", "figure"),
                Output("torque-magnitude-plot", "figure"),
                Output("current-forces", "children"),
                Output("current-torques", "children"),
                Output("statistics", "children"),
                Output("status", "children"),
            ],
            [Input("interval-component", "n_intervals")],
        )(self.update_plots)

    def update_plots(self, n):
        """Update all plots and displays."""
        data = self.get_plot_data()

        # Status
        status = f"Force messages: {data['force_count']} | Torque messages: {data['torque_count']} | Running: {'Yes' if self.running else 'No'}"

        if not data["timestamps"]:
            empty_fig = {"data": [], "layout": {"title": "Waiting for data..."}}
            return (
                empty_fig,
                empty_fig,
                empty_fig,
                empty_fig,
                "Waiting for data...",
                "Waiting for data...",
                "No data yet",
                status,
            )

        # Force components plot
        force_fig = go.Figure()
        force_fig.add_trace(
            go.Scatter(
                x=data["timestamps"],
                y=data["forces"]["x"],
                mode="lines",
                name="Fx",
                line=dict(color="red", width=2),
            )
        )
        force_fig.add_trace(
            go.Scatter(
                x=data["timestamps"],
                y=data["forces"]["y"],
                mode="lines",
                name="Fy",
                line=dict(color="green", width=2),
            )
        )
        force_fig.add_trace(
            go.Scatter(
                x=data["timestamps"],
                y=data["forces"]["z"],
                mode="lines",
                name="Fz",
                line=dict(color="blue", width=2),
            )
        )
        force_fig.update_layout(
            title="Force Components",
            xaxis_title="Time (s)",
            yaxis_title="Force (N)",
            hovermode="x unified",
            showlegend=True,
            margin=dict(l=50, r=50, t=50, b=50),
        )

        # Torque components plot
        torque_fig = go.Figure()
        torque_fig.add_trace(
            go.Scatter(
                x=data["timestamps"],
                y=data["torques"]["x"],
                mode="lines",
                name="Mx",
                line=dict(color="red", width=2),
            )
        )
        torque_fig.add_trace(
            go.Scatter(
                x=data["timestamps"],
                y=data["torques"]["y"],
                mode="lines",
                name="My",
                line=dict(color="green", width=2),
            )
        )
        torque_fig.add_trace(
            go.Scatter(
                x=data["timestamps"],
                y=data["torques"]["z"],
                mode="lines",
                name="Mz",
                line=dict(color="blue", width=2),
            )
        )
        torque_fig.update_layout(
            title="Torque Components",
            xaxis_title="Time (s)",
            yaxis_title="Torque (N⋅m)",
            hovermode="x unified",
            showlegend=True,
            margin=dict(l=50, r=50, t=50, b=50),
        )

        # Force magnitude plot
        force_mag_fig = go.Figure()
        force_mag_fig.add_trace(
            go.Scatter(
                x=data["timestamps"],
                y=data["force_magnitudes"],
                mode="lines",
                name="|F|",
                line=dict(color="purple", width=2),
                fill="tozeroy",
                fillcolor="rgba(128, 0, 128, 0.2)",
            )
        )
        force_mag_fig.update_layout(
            title="Force Magnitude",
            xaxis_title="Time (s)",
            yaxis_title="|F| (N)",
            showlegend=False,
            margin=dict(l=50, r=50, t=50, b=50),
        )

        # Torque magnitude plot
        torque_mag_fig = go.Figure()
        torque_mag_fig.add_trace(
            go.Scatter(
                x=data["timestamps"],
                y=data["torque_magnitudes"],
                mode="lines",
                name="|M|",
                line=dict(color="orange", width=2),
                fill="tozeroy",
                fillcolor="rgba(255, 165, 0, 0.2)",
            )
        )
        torque_mag_fig.update_layout(
            title="Torque Magnitude",
            xaxis_title="Time (s)",
            yaxis_title="|M| (N⋅m)",
            showlegend=False,
            margin=dict(l=50, r=50, t=50, b=50),
        )

        # Current values display
        current_forces = (
            f"Fx: {data['latest_force'][0]:8.3f} N\n"
            f"Fy: {data['latest_force'][1]:8.3f} N\n"
            f"Fz: {data['latest_force'][2]:8.3f} N\n"
            f"|F|: {data['latest_force_mag']:8.3f} N"
        )

        current_torques = (
            f"Mx: {data['latest_torque'][0]:8.4f} N⋅m\n"
            f"My: {data['latest_torque'][1]:8.4f} N⋅m\n"
            f"Mz: {data['latest_torque'][2]:8.4f} N⋅m\n"
            f"|M|: {data['latest_torque_mag']:8.4f} N⋅m"
        )

        # Calculate statistics
        statistics = ""
        if len(data["force_magnitudes"]) > 0:
            force_data = np.array([data["forces"]["x"], data["forces"]["y"], data["forces"]["z"]])
            force_mean = np.mean(force_data, axis=1)
            force_std = np.std(force_data, axis=1)
            force_max = np.max(np.abs(force_data), axis=1)

            torque_data = np.array([data["torques"]["x"], data["torques"]["y"], data["torques"]["z"]])
            torque_mean = np.mean(torque_data, axis=1)
            torque_std = np.std(torque_data, axis=1)
            torque_max = np.max(np.abs(torque_data), axis=1)

            statistics = (
                "Force Statistics:\n"
                f"  Mean: Fx={force_mean[0]:.3f}, Fy={force_mean[1]:.3f}, Fz={force_mean[2]:.3f} N\n"
                f"  Std:  Fx={force_std[0]:.3f}, Fy={force_std[1]:.3f}, Fz={force_std[2]:.3f} N\n"
                f"  Max:  Fx={force_max[0]:.3f}, Fy={force_max[1]:.3f}, Fz={force_max[2]:.3f} N\n"
                f"  Mean |F|: {np.mean(data['force_magnitudes']):.3f} N\n\n"
                "Torque Statistics:\n"
                f"  Mean: Mx={torque_mean[0]:.4f}, My={torque_mean[1]:.4f}, Mz={torque_mean[2]:.4f} N⋅m\n"
                f"  Std:  Mx={torque_std[0]:.4f}, My={torque_std[1]:.4f}, Mz={torque_std[2]:.4f} N⋅m\n"
                f"  Max:  Mx={torque_max[0]:.4f}, My={torque_max[1]:.4f}, Mz={torque_max[2]:.4f} N⋅m\n"
                f"  Mean |M|: {np.mean(data['torque_magnitudes']):.4f} N⋅m"
            )

        return (
            force_fig,
            torque_fig,
            force_mag_fig,
            torque_mag_fig,
            current_forces,
            current_torques,
            statistics,
            status,
        )

    def run_dash(self):
        """Run the Dash web server in a separate thread."""
        try:
            # Clear Werkzeug environment variable that causes issues in multiprocess
            import os
            if 'WERKZEUG_SERVER_FD' in os.environ:
                del os.environ['WERKZEUG_SERVER_FD']

            # Initialize Dash app here, in the worker thread
            if not self.app:
                self._initialize_dash_app()

            logger.info(f"Starting Force-Torque Visualization Dashboard...")
            logger.info(f"Open http://{self.dash_host if self.dash_host != '0.0.0.0' else '127.0.0.1'}:{self.dash_port} in your browser")

            # Use Flask server directly to avoid Dash/Werkzeug issues
            # This works better in multiprocess environments
            from werkzeug.serving import make_server

            server = make_server(
                self.dash_host,
                self.dash_port,
                self.app.server,
                threaded=True
            )

            logger.info(f"Dashboard server started at http://{self.dash_host if self.dash_host != '0.0.0.0' else '127.0.0.1'}:{self.dash_port}")
            server.serve_forever()
        except Exception as e:
            # Silently ignore all errors to prevent log spam
            pass

    @rpc
    def start(self):
        """Start the visualization module."""
        if self.running:
            logger.warning("FT visualizer already running")
            return

        logger.info(f"Starting FT visualizer module...")
        self.running = True
        self.start_time = time.time()

        # Subscribe to force data
        if self.force:
            self.force.subscribe(self.handle_force)
            logger.info("Subscribed to force data")

        # Subscribe to torque data
        if self.torque:
            self.torque.subscribe(self.handle_torque)
            logger.info("Subscribed to torque data")

        # Start Dash in a separate thread
        self.dash_thread = threading.Thread(target=self.run_dash, daemon=True)
        self.dash_thread.start()

        logger.info("FT visualizer started successfully")

    @rpc
    def stop(self):
        """Stop the visualization module."""
        if not self.running:
            return

        logger.info("Stopping FT visualizer...")
        self.running = False
        logger.info(f"Total messages received - Force: {self.force_count}, Torque: {self.torque_count}")

    @rpc
    def get_stats(self) -> Dict[str, Any]:
        """Get visualizer statistics."""
        return {
            "force_count": self.force_count,
            "torque_count": self.torque_count,
            "running": self.running,
            "data_points": len(self.timestamps),
            "dash_url": f"http://{self.dash_host if self.dash_host != '0.0.0.0' else '127.0.0.1'}:{self.dash_port}",
        }


if __name__ == "__main__":
    # For testing standalone
    import argparse

    parser = argparse.ArgumentParser(description="FT Visualizer Module")
    parser.add_argument("--port", type=int, default=8052, help="Dash server port")
    parser.add_argument("--host", default="0.0.0.0", help="Dash server host")
    parser.add_argument("--history", type=int, default=500, help="Max history points")
    parser.add_argument("--interval", type=int, default=100, help="Update interval (ms)")
    parser.add_argument("--verbose", action="store_true", help="Verbose output")
    args = parser.parse_args()

    from dimos.core import start

    dimos = start(1)
    visualizer = dimos.deploy(
        FTVisualizerModule,
        dash_port=args.port,
        dash_host=args.host,
        max_history=args.history,
        update_interval_ms=args.interval,
        verbose=args.verbose,
    )

    visualizer.start()

    # Keep running
    import time

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        visualizer.stop()