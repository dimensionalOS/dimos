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
Receives data via LCM transport from the FT driver module.
"""

import dash
from dash import dcc, html, Input, Output
import plotly.graph_objs as go
import time
import threading
import queue
import numpy as np
from collections import deque
from datetime import datetime
from typing import Dict, Any
from dataclasses import dataclass, field

from dimos.core import Module, In, rpc
from dimos.msgs.geometry_msgs import Vector3


# Import data structures from driver module
from dimos.hardware.ft_driver_module import ForceTorqueData, RawSensorData


class FTVisualizerModule(Module):
    """Force-Torque sensor visualization module using Dash."""

    # Input ports
    calibrated_data: In[ForceTorqueData] = None  # Calibrated force-torque data
    raw_sensor_data: In[RawSensorData] = None  # Raw sensor values (optional)

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

        # Raw sensor data storage (optional)
        self.raw_sensors = deque(maxlen=max_history)

        # Latest values for display
        self.latest_forces = [0, 0, 0]
        self.latest_torques = [0, 0, 0]
        self.latest_force_mag = 0
        self.latest_torque_mag = 0
        self.latest_raw_sensors = []

        # Statistics
        self.message_count = 0
        self.start_time = None

        # Thread-safe queue for data exchange with Dash
        self.data_queue = queue.Queue(maxsize=100)

        # Dash app
        self.app = None
        self.dash_thread = None
        self.running = False

    def handle_calibrated_data(self, msg: ForceTorqueData):
        """Handle incoming calibrated force-torque data."""
        self.message_count += 1

        if self.start_time is None:
            self.start_time = time.time()

        # Calculate relative timestamp
        rel_time = time.time() - self.start_time

        # Store data
        self.timestamps.append(rel_time)

        # Extract force and torque values
        forces = [msg.forces.x, msg.forces.y, msg.forces.z]
        torques = [msg.torques.x, msg.torques.y, msg.torques.z]

        self.forces["x"].append(forces[0])
        self.forces["y"].append(forces[1])
        self.forces["z"].append(forces[2])

        self.torques["x"].append(torques[0])
        self.torques["y"].append(torques[1])
        self.torques["z"].append(torques[2])

        self.force_magnitudes.append(msg.force_magnitude)
        self.torque_magnitudes.append(msg.torque_magnitude)

        # Update latest values
        self.latest_forces = forces
        self.latest_torques = torques
        self.latest_force_mag = msg.force_magnitude
        self.latest_torque_mag = msg.torque_magnitude
        if hasattr(msg, "raw_sensors"):
            self.latest_raw_sensors = msg.raw_sensors

        # Update queue for dashboard
        try:
            self.data_queue.put_nowait(
                {
                    "timestamp": rel_time,
                    "forces": forces,
                    "torques": torques,
                    "force_magnitude": msg.force_magnitude,
                    "torque_magnitude": msg.torque_magnitude,
                }
            )
        except queue.Full:
            # Remove old item if queue is full
            try:
                self.data_queue.get_nowait()
                self.data_queue.put_nowait(
                    {
                        "timestamp": rel_time,
                        "forces": forces,
                        "torques": torques,
                        "force_magnitude": msg.force_magnitude,
                        "torque_magnitude": msg.torque_magnitude,
                    }
                )
            except:
                pass

        if self.verbose:
            print(
                f"Received FT data: F={forces}, T={torques}, |F|={msg.force_magnitude:.3f}, |T|={msg.torque_magnitude:.4f}"
            )

    def handle_raw_sensor_data(self, msg: RawSensorData):
        """Handle incoming raw sensor data (optional)."""
        if hasattr(msg, "sensor_values"):
            self.raw_sensors.append(msg.sensor_values)
            self.latest_raw_sensors = msg.sensor_values

    def get_plot_data(self) -> Dict[str, Any]:
        """Get data formatted for plotting."""
        return {
            "timestamps": list(self.timestamps),
            "forces": {k: list(v) for k, v in self.forces.items()},
            "torques": {k: list(v) for k, v in self.torques.items()},
            "force_magnitudes": list(self.force_magnitudes),
            "torque_magnitudes": list(self.torque_magnitudes),
            "latest_forces": self.latest_forces,
            "latest_torques": self.latest_torques,
            "latest_force_mag": self.latest_force_mag,
            "latest_torque_mag": self.latest_torque_mag,
            "latest_raw_sensors": self.latest_raw_sensors,
            "message_count": self.message_count,
        }

    def create_dash_app(self):
        """Create and configure the Dash application."""
        self.app = dash.Dash(__name__)

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
                                html.Div(
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
                                html.Div(
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
                                    style={
                                        "width": "50%",
                                        "display": "inline-block",
                                        "height": "300px",
                                    },
                                ),
                                dcc.Graph(
                                    id="torque-magnitude-plot",
                                    style={
                                        "width": "50%",
                                        "display": "inline-block",
                                        "height": "300px",
                                    },
                                ),
                            ]
                        ),
                    ]
                ),
                # Statistics
                html.Div(
                    [
                        html.H3("Statistics", style={"text-align": "center"}),
                        html.Div(
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
                # Raw sensor values (optional)
                html.Div(
                    [
                        html.H3("Raw Sensor Values", style={"text-align": "center"}),
                        html.Div(
                            id="raw-sensors",
                            style={
                                "font-family": "monospace",
                                "padding": "10px",
                                "background-color": "#f9f9f9",
                                "border-radius": "5px",
                                "max-height": "150px",
                                "overflow-y": "auto",
                            },
                        ),
                    ],
                    style={
                        "padding": "20px",
                        "display": "none" if not self.latest_raw_sensors else "block",
                    },
                ),
                # Update interval
                dcc.Interval(
                    id="interval-component", interval=self.update_interval_ms, n_intervals=0
                ),
            ]
        )

        @self.app.callback(
            [
                Output("force-plot", "figure"),
                Output("torque-plot", "figure"),
                Output("force-magnitude-plot", "figure"),
                Output("torque-magnitude-plot", "figure"),
                Output("current-forces", "children"),
                Output("current-torques", "children"),
                Output("statistics", "children"),
                Output("raw-sensors", "children"),
                Output("status", "children"),
            ],
            [Input("interval-component", "n_intervals")],
        )
        def update_plots(n):
            """Update all plots and displays."""
            data = self.get_plot_data()

            # Status
            status = (
                f"Messages: {data['message_count']} | Running: {'Yes' if self.running else 'No'}"
            )

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
                    "No sensor data",
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
                f"Fx: {data['latest_forces'][0]:8.3f} N\n"
                f"Fy: {data['latest_forces'][1]:8.3f} N\n"
                f"Fz: {data['latest_forces'][2]:8.3f} N\n"
                f"|F|: {data['latest_force_mag']:8.3f} N"
            )

            current_torques = (
                f"Mx: {data['latest_torques'][0]:8.4f} N⋅m\n"
                f"My: {data['latest_torques'][1]:8.4f} N⋅m\n"
                f"Mz: {data['latest_torques'][2]:8.4f} N⋅m\n"
                f"|M|: {data['latest_torque_mag']:8.4f} N⋅m"
            )

            # Calculate statistics
            statistics = ""
            if len(data["force_magnitudes"]) > 0:
                force_data = np.array(
                    [data["forces"]["x"], data["forces"]["y"], data["forces"]["z"]]
                )
                force_mean = np.mean(force_data, axis=1)
                force_std = np.std(force_data, axis=1)
                force_max = np.max(np.abs(force_data), axis=1)

                torque_data = np.array(
                    [data["torques"]["x"], data["torques"]["y"], data["torques"]["z"]]
                )
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

            # Raw sensor values
            raw_sensors_text = ""
            if data["latest_raw_sensors"]:
                raw_sensors_text = "Sensor Values (with moving average):\n"
                for i, val in enumerate(data["latest_raw_sensors"]):
                    magnet = i // 4 + 1
                    sensor = i % 4 + 1
                    raw_sensors_text += f"  S{i + 1:02d} (M{magnet}S{sensor}): {val:8.3f}\n"
                    if (i + 1) % 4 == 0 and i < 15:
                        raw_sensors_text += ""

            return (
                force_fig,
                torque_fig,
                force_mag_fig,
                torque_mag_fig,
                current_forces,
                current_torques,
                statistics,
                raw_sensors_text,
                status,
            )

    def run_dash(self):
        """Run the Dash web server in a separate thread."""
        self.create_dash_app()
        print(f"Starting Force-Torque Visualization Dashboard...")
        print(
            f"Open http://{self.dash_host if self.dash_host != '0.0.0.0' else '127.0.0.1'}:{self.dash_port} in your browser"
        )
        self.app.run(debug=False, port=self.dash_port, host=self.dash_host, use_reloader=False)

    @rpc
    def start(self):
        """Start the visualization module."""
        if self.running:
            print("FT visualizer already running")
            return

        print(f"Starting FT visualizer module...")
        self.running = True
        self.start_time = time.time()

        # Subscribe to calibrated data
        if self.calibrated_data:
            self.calibrated_data.subscribe(self.handle_calibrated_data)
            print("Subscribed to calibrated force-torque data")

        # Optionally subscribe to raw sensor data
        if self.raw_sensor_data:
            self.raw_sensor_data.subscribe(self.handle_raw_sensor_data)
            print("Subscribed to raw sensor data")

        # Start Dash in a separate thread
        self.dash_thread = threading.Thread(target=self.run_dash, daemon=True)
        self.dash_thread.start()

        print("FT visualizer started successfully")

    @rpc
    def stop(self):
        """Stop the visualization module."""
        print("\nStopping FT visualizer...")
        self.running = False
        print(f"Total messages received: {self.message_count}")

    @rpc
    def get_stats(self) -> Dict[str, Any]:
        """Get visualizer statistics."""
        return {
            "message_count": self.message_count,
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
