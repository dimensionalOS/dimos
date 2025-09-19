#!/usr/bin/env python3
"""
Force-Torque Sensor Real-time Visualization

Visualizes calibrated force-torque sensor data using Dash and Plotly.
Subscribes to ZMQ port 5556 for calibrated data from calc_calibration_matrix.py
"""

import dash
from dash import dcc, html, Input, Output
import plotly.graph_objs as go
import plotly.subplots as ps
import zmq
import json
import time
import threading
import queue
import numpy as np
from collections import deque
from datetime import datetime


# Configuration
MAX_HISTORY_POINTS = 500  # Number of points to keep in history
UPDATE_INTERVAL_MS = 100  # Dashboard update interval
ZMQ_PORT = 5556  # Port for receiving calibrated data


class ForceToqueDataReceiver:
    """Receives and stores calibrated force-torque data from ZMQ."""
    
    def __init__(self, port=ZMQ_PORT, max_history=MAX_HISTORY_POINTS):
        self.port = port
        self.max_history = max_history
        self.running = True
        
        # Data storage with deques for efficient append/pop
        self.timestamps = deque(maxlen=max_history)
        self.forces = {
            'x': deque(maxlen=max_history),
            'y': deque(maxlen=max_history),
            'z': deque(maxlen=max_history)
        }
        self.torques = {
            'x': deque(maxlen=max_history),
            'y': deque(maxlen=max_history),
            'z': deque(maxlen=max_history)
        }
        self.force_magnitudes = deque(maxlen=max_history)
        self.torque_magnitudes = deque(maxlen=max_history)
        
        # Latest values for display
        self.latest_forces = [0, 0, 0]
        self.latest_torques = [0, 0, 0]
        self.latest_force_mag = 0
        self.latest_torque_mag = 0
        
        # Thread-safe queue for data exchange
        self.data_queue = queue.Queue(maxsize=10)
        
        # Setup ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://localhost:{port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
        
    def receive_data(self):
        """Continuously receive calibrated data from ZMQ."""
        print(f"Listening for calibrated force-torque data on port {self.port}...")
        
        start_time = time.time()
        
        while self.running:
            try:
                # Try to receive data
                try:
                    data_str = self.socket.recv_string(zmq.NOBLOCK)
                    data = json.loads(data_str)
                    
                    if 'forces' in data and 'torques' in data:
                        # Calculate relative timestamp
                        rel_time = time.time() - start_time
                        
                        # Store data
                        self.timestamps.append(rel_time)
                        
                        forces = data['forces']
                        torques = data['torques']
                        
                        self.forces['x'].append(forces[0])
                        self.forces['y'].append(forces[1])
                        self.forces['z'].append(forces[2])
                        
                        self.torques['x'].append(torques[0])
                        self.torques['y'].append(torques[1])
                        self.torques['z'].append(torques[2])
                        
                        self.force_magnitudes.append(data.get('force_magnitude', np.linalg.norm(forces)))
                        self.torque_magnitudes.append(data.get('torque_magnitude', np.linalg.norm(torques)))
                        
                        # Update latest values
                        self.latest_forces = forces
                        self.latest_torques = torques
                        self.latest_force_mag = self.force_magnitudes[-1]
                        self.latest_torque_mag = self.torque_magnitudes[-1]
                        
                        # Update queue for dashboard
                        try:
                            if self.data_queue.full():
                                self.data_queue.get_nowait()
                            self.data_queue.put_nowait({
                                'timestamp': rel_time,
                                'forces': forces,
                                'torques': torques
                            })
                        except queue.Full:
                            pass
                            
                except zmq.Again:
                    # No data available, continue
                    time.sleep(0.01)
                    
            except zmq.ZMQError as e:
                print(f"ZMQ error: {e}")
                time.sleep(0.1)
            except json.JSONDecodeError as e:
                print(f"JSON decode error: {e}")
            except Exception as e:
                print(f"Error receiving data: {e}")
                time.sleep(0.1)
    
    def get_plot_data(self):
        """Get data formatted for plotting."""
        return {
            'timestamps': list(self.timestamps),
            'forces': {k: list(v) for k, v in self.forces.items()},
            'torques': {k: list(v) for k, v in self.torques.items()},
            'force_magnitudes': list(self.force_magnitudes),
            'torque_magnitudes': list(self.torque_magnitudes),
            'latest_forces': self.latest_forces,
            'latest_torques': self.latest_torques,
            'latest_force_mag': self.latest_force_mag,
            'latest_torque_mag': self.latest_torque_mag
        }
    
    def stop(self):
        """Stop the receiver."""
        self.running = False
        self.socket.close()
        self.context.term()


# Create Dash application
app = dash.Dash(__name__)

# Global data receiver
receiver = None

app.layout = html.Div([
    html.H1("Force-Torque Sensor Visualization", style={'text-align': 'center'}),
    
    # Current values display
    html.Div([
        html.Div([
            html.H3("Current Forces (N)", style={'text-align': 'center'}),
            html.Div(id='current-forces', style={
                'font-family': 'monospace',
                'font-size': '18px',
                'text-align': 'center',
                'padding': '10px',
                'background-color': '#f0f0f0',
                'border-radius': '5px'
            })
        ], style={'width': '48%', 'display': 'inline-block', 'padding': '10px'}),
        
        html.Div([
            html.H3("Current Torques (N⋅m)", style={'text-align': 'center'}),
            html.Div(id='current-torques', style={
                'font-family': 'monospace',
                'font-size': '18px',
                'text-align': 'center',
                'padding': '10px',
                'background-color': '#f0f0f0',
                'border-radius': '5px'
            })
        ], style={'width': '48%', 'display': 'inline-block', 'padding': '10px'}),
    ]),
    
    # Main plots
    html.Div([
        # Force components plot
        dcc.Graph(id='force-plot', style={'height': '400px'}),
        
        # Torque components plot
        dcc.Graph(id='torque-plot', style={'height': '400px'}),
        
        # Magnitude plots
        html.Div([
            dcc.Graph(id='force-magnitude-plot', style={'width': '50%', 'display': 'inline-block', 'height': '300px'}),
            dcc.Graph(id='torque-magnitude-plot', style={'width': '50%', 'display': 'inline-block', 'height': '300px'}),
        ]),
    ]),
    
    # Statistics
    html.Div([
        html.H3("Statistics", style={'text-align': 'center'}),
        html.Div(id='statistics', style={
            'font-family': 'monospace',
            'padding': '20px',
            'background-color': '#f9f9f9',
            'border-radius': '5px'
        })
    ], style={'padding': '20px'}),
    
    # Update interval
    dcc.Interval(
        id='interval-component',
        interval=UPDATE_INTERVAL_MS,
        n_intervals=0
    )
])


@app.callback(
    [Output('force-plot', 'figure'),
     Output('torque-plot', 'figure'),
     Output('force-magnitude-plot', 'figure'),
     Output('torque-magnitude-plot', 'figure'),
     Output('current-forces', 'children'),
     Output('current-torques', 'children'),
     Output('statistics', 'children')],
    [Input('interval-component', 'n_intervals')]
)
def update_plots(n):
    """Update all plots and displays."""
    if receiver is None:
        return [{}, {}, {}, {}, "No data", "No data", "No data"]
    
    data = receiver.get_plot_data()
    
    if not data['timestamps']:
        return [{}, {}, {}, {}, "Waiting for data...", "Waiting for data...", "No data yet"]
    
    # Force components plot
    force_fig = go.Figure()
    force_fig.add_trace(go.Scatter(
        x=data['timestamps'], y=data['forces']['x'],
        mode='lines', name='Fx', line=dict(color='red', width=2)
    ))
    force_fig.add_trace(go.Scatter(
        x=data['timestamps'], y=data['forces']['y'],
        mode='lines', name='Fy', line=dict(color='green', width=2)
    ))
    force_fig.add_trace(go.Scatter(
        x=data['timestamps'], y=data['forces']['z'],
        mode='lines', name='Fz', line=dict(color='blue', width=2)
    ))
    force_fig.update_layout(
        title="Force Components",
        xaxis_title="Time (s)",
        yaxis_title="Force (N)",
        hovermode='x unified',
        showlegend=True,
        margin=dict(l=50, r=50, t=50, b=50)
    )
    
    # Torque components plot
    torque_fig = go.Figure()
    torque_fig.add_trace(go.Scatter(
        x=data['timestamps'], y=data['torques']['x'],
        mode='lines', name='Mx', line=dict(color='red', width=2)
    ))
    torque_fig.add_trace(go.Scatter(
        x=data['timestamps'], y=data['torques']['y'],
        mode='lines', name='My', line=dict(color='green', width=2)
    ))
    torque_fig.add_trace(go.Scatter(
        x=data['timestamps'], y=data['torques']['z'],
        mode='lines', name='Mz', line=dict(color='blue', width=2)
    ))
    torque_fig.update_layout(
        title="Torque Components",
        xaxis_title="Time (s)",
        yaxis_title="Torque (N⋅m)",
        hovermode='x unified',
        showlegend=True,
        margin=dict(l=50, r=50, t=50, b=50)
    )
    
    # Force magnitude plot
    force_mag_fig = go.Figure()
    force_mag_fig.add_trace(go.Scatter(
        x=data['timestamps'], y=data['force_magnitudes'],
        mode='lines', name='|F|', line=dict(color='purple', width=2),
        fill='tozeroy', fillcolor='rgba(128, 0, 128, 0.2)'
    ))
    force_mag_fig.update_layout(
        title="Force Magnitude",
        xaxis_title="Time (s)",
        yaxis_title="|F| (N)",
        showlegend=False,
        margin=dict(l=50, r=50, t=50, b=50)
    )
    
    # Torque magnitude plot
    torque_mag_fig = go.Figure()
    torque_mag_fig.add_trace(go.Scatter(
        x=data['timestamps'], y=data['torque_magnitudes'],
        mode='lines', name='|M|', line=dict(color='orange', width=2),
        fill='tozeroy', fillcolor='rgba(255, 165, 0, 0.2)'
    ))
    torque_mag_fig.update_layout(
        title="Torque Magnitude",
        xaxis_title="Time (s)",
        yaxis_title="|M| (N⋅m)",
        showlegend=False,
        margin=dict(l=50, r=50, t=50, b=50)
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
    if len(data['force_magnitudes']) > 0:
        stats = []
        
        # Force statistics
        force_data = np.array([data['forces']['x'], data['forces']['y'], data['forces']['z']])
        force_mean = np.mean(force_data, axis=1)
        force_std = np.std(force_data, axis=1)
        force_max = np.max(np.abs(force_data), axis=1)
        
        stats.append("Force Statistics:")
        stats.append(f"  Mean: Fx={force_mean[0]:.3f}, Fy={force_mean[1]:.3f}, Fz={force_mean[2]:.3f} N")
        stats.append(f"  Std:  Fx={force_std[0]:.3f}, Fy={force_std[1]:.3f}, Fz={force_std[2]:.3f} N")
        stats.append(f"  Max:  Fx={force_max[0]:.3f}, Fy={force_max[1]:.3f}, Fz={force_max[2]:.3f} N")
        stats.append(f"  Mean |F|: {np.mean(data['force_magnitudes']):.3f} N")
        stats.append("")
        
        # Torque statistics
        torque_data = np.array([data['torques']['x'], data['torques']['y'], data['torques']['z']])
        torque_mean = np.mean(torque_data, axis=1)
        torque_std = np.std(torque_data, axis=1)
        torque_max = np.max(np.abs(torque_data), axis=1)
        
        stats.append("Torque Statistics:")
        stats.append(f"  Mean: Mx={torque_mean[0]:.4f}, My={torque_mean[1]:.4f}, Mz={torque_mean[2]:.4f} N⋅m")
        stats.append(f"  Std:  Mx={torque_std[0]:.4f}, My={torque_std[1]:.4f}, Mz={torque_std[2]:.4f} N⋅m")
        stats.append(f"  Max:  Mx={torque_max[0]:.4f}, My={torque_max[1]:.4f}, Mz={torque_max[2]:.4f} N⋅m")
        stats.append(f"  Mean |M|: {np.mean(data['torque_magnitudes']):.4f} N⋅m")
        
        statistics = '\n'.join(stats)
    else:
        statistics = "Collecting data..."
    
    return (force_fig, torque_fig, force_mag_fig, torque_mag_fig,
            current_forces, current_torques, statistics)


if __name__ == '__main__':
    # Start data receiver in separate thread
    receiver = ForceToqueDataReceiver(port=ZMQ_PORT)
    receiver_thread = threading.Thread(target=receiver.receive_data, daemon=True)
    receiver_thread.start()
    
    try:
        # Run Dash app
        print(f"Starting Force-Torque Visualization Dashboard...")
        print(f"Listening for calibrated data on ZMQ port {ZMQ_PORT}")
        print(f"Open http://127.0.0.1:8052 in your browser")
        app.run(debug=False, port=8052, host='0.0.0.0')
    finally:
        if receiver:
            receiver.stop()
            receiver_thread.join(timeout=1.0)
        print("Visualization stopped.")