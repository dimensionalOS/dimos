#!/usr/bin/env python3
"""
Force-Torque Sensor Calibration Matrix Calculator

This script calculates a calibration matrix for a 16-channel magnetic force-torque sensor
using least squares regression. It can also apply the calibration to live sensor data.

The sensor has 4 magnets, each with 4 hall effect sensors arranged in a plus pattern,
giving 16 raw measurements that are mapped to 6 DOF (3 forces + 3 torques).
"""

import numpy as np
import pandas as pd
import argparse
import json
import time
import zmq
from pathlib import Path
from typing import Tuple, Optional, Dict, Any


class ForceTorqueCalibrator:
    """Calibrates and applies calibration to force-torque sensor data."""
    
    def __init__(self):
        self.calibration_matrix = None  # 6x16 matrix
        self.bias_vector = None  # 6x1 vector
        self.sensor_channels = 16
        self.output_channels = 6  # Fx, Fy, Fz, Mx, My, Mz
        
    def load_calibration_data(self, csv_path: str) -> Tuple[np.ndarray, np.ndarray]:
        """
        Load calibration data from CSV file.
        
        Args:
            csv_path: Path to calibration CSV file
            
        Returns:
            sensor_data: Nx16 array of sensor readings
            force_torque_data: Nx6 array of forces and torques
        """
        print(f"Loading calibration data from: {csv_path}")
        df = pd.read_csv(csv_path)
        
        # Extract sensor columns (sensor_1 through sensor_16)
        sensor_columns = [f'sensor_{i}' for i in range(1, 17)]
        missing_sensors = [col for col in sensor_columns if col not in df.columns]
        
        if missing_sensors:
            print(f"Warning: Missing sensor columns: {missing_sensors}")
            print("Available columns:", df.columns.tolist())
            raise ValueError(f"CSV file is missing sensor data columns: {missing_sensors}")
        
        sensor_data = df[sensor_columns].values
        
        # Extract force and torque columns in local frame
        force_torque_columns = [
            'force_local_x', 'force_local_y', 'force_local_z',
            'torque_local_x', 'torque_local_y', 'torque_local_z'
        ]
        
        missing_ft = [col for col in force_torque_columns if col not in df.columns]
        if missing_ft:
            raise ValueError(f"CSV file is missing force/torque columns: {missing_ft}")
        
        force_torque_data = df[force_torque_columns].values
        
        # Remove any rows with NaN values
        valid_mask = ~(np.isnan(sensor_data).any(axis=1) | np.isnan(force_torque_data).any(axis=1))
        sensor_data = sensor_data[valid_mask]
        force_torque_data = force_torque_data[valid_mask]
        
        print(f"Loaded {len(sensor_data)} valid data points")
        print(f"Sensor data shape: {sensor_data.shape}")
        print(f"Force/torque data shape: {force_torque_data.shape}")
        
        # Print data statistics
        print(f"\nSensor data statistics:")
        print(f"  Mean: {np.mean(sensor_data, axis=0).mean():.2f}")
        print(f"  Std:  {np.std(sensor_data, axis=0).mean():.2f}")
        print(f"  Min:  {np.min(sensor_data):.2f}")
        print(f"  Max:  {np.max(sensor_data):.2f}")
        
        print(f"\nForce/torque statistics:")
        print(f"  Force magnitude mean:  {np.linalg.norm(force_torque_data[:, :3], axis=1).mean():.2f} N")
        print(f"  Torque magnitude mean: {np.linalg.norm(force_torque_data[:, 3:], axis=1).mean():.4f} N⋅m")
        
        return sensor_data, force_torque_data
    
    def calculate_calibration_matrix(self, sensor_data: np.ndarray, force_torque_data: np.ndarray,
                                    use_bias: bool = True) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        Calculate calibration matrix using least squares.
        
        The model is: F = C @ S + b
        where F is 6x1 force/torque, S is 16x1 sensor reading, C is 6x16 calibration matrix
        
        Args:
            sensor_data: Nx16 array of sensor readings
            force_torque_data: Nx6 array of known forces/torques
            use_bias: Whether to include bias term
            
        Returns:
            calibration_matrix: 6x16 calibration matrix
            bias_vector: 6x1 bias vector (if use_bias=True)
        """
        print(f"\nCalculating calibration matrix using least squares...")
        print(f"Using bias term: {use_bias}")
        
        N = sensor_data.shape[0]
        
        if use_bias:
            # Augment sensor data with ones for bias term
            # S_aug = [S | 1] making it Nx17
            S_augmented = np.hstack([sensor_data, np.ones((N, 1))])
            
            # Solve: F = S_aug @ C_aug^T
            # Using least squares: C_aug^T = pinv(S_aug) @ F
            # C_aug is 6x17 (includes bias as last column)
            
            # Use numpy's least squares solver for numerical stability
            # lstsq solves S_augmented @ X = force_torque_data for X
            X, residuals, rank, singular_values = np.linalg.lstsq(S_augmented, force_torque_data, rcond=None)
            
            # X is 17x6, so C_aug = X^T is 6x17
            C_augmented = X.T
            
            # Extract calibration matrix and bias
            calibration_matrix = C_augmented[:, :16]  # 6x16
            bias_vector = C_augmented[:, 16]  # 6x1
            
            print(f"Matrix rank: {rank} (expected: {min(S_augmented.shape)})")
            print(f"Condition number: {singular_values[0]/singular_values[-1]:.2e}")
            
        else:
            # Without bias: F = S @ C^T
            # C^T = pinv(S) @ F
            X, residuals, rank, singular_values = np.linalg.lstsq(sensor_data, force_torque_data, rcond=None)
            calibration_matrix = X.T  # 6x16
            bias_vector = None
            
            print(f"Matrix rank: {rank} (expected: {min(sensor_data.shape)})")
            print(f"Condition number: {singular_values[0]/singular_values[-1]:.2e}")
        
        # Calculate and print residuals
        if use_bias:
            predictions = sensor_data @ calibration_matrix.T + bias_vector
        else:
            predictions = sensor_data @ calibration_matrix.T
        
        residuals = force_torque_data - predictions
        
        print(f"\nCalibration quality metrics:")
        print(f"  Force RMSE:  {np.sqrt(np.mean(residuals[:, :3]**2)):.4f} N")
        print(f"  Torque RMSE: {np.sqrt(np.mean(residuals[:, 3:]**2)):.6f} N⋅m")
        
        # Print per-axis errors
        axis_names = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
        for i, name in enumerate(axis_names):
            rmse = np.sqrt(np.mean(residuals[:, i]**2))
            unit = 'N' if i < 3 else 'N⋅m'
            print(f"  {name} RMSE: {rmse:.6f} {unit}")
        
        return calibration_matrix, bias_vector
    
    def save_calibration(self, filepath: str, calibration_matrix: np.ndarray, 
                         bias_vector: Optional[np.ndarray] = None,
                         metadata: Optional[Dict[str, Any]] = None):
        """Save calibration matrix and bias to file."""
        data = {
            'calibration_matrix': calibration_matrix.tolist(),
            'bias_vector': bias_vector.tolist() if bias_vector is not None else None,
            'sensor_channels': self.sensor_channels,
            'output_channels': self.output_channels,
            'timestamp': time.time(),
            'metadata': metadata or {}
        }
        
        filepath = Path(filepath)
        if filepath.suffix == '.npz':
            # Save as numpy archive
            np.savez(filepath, **data)
        else:
            # Save as JSON
            if filepath.suffix != '.json':
                filepath = filepath.with_suffix('.json')
            
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
        
        print(f"\nCalibration saved to: {filepath}")
    
    def load_calibration(self, filepath: str) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """Load calibration matrix and bias from file."""
        filepath = Path(filepath)
        
        if filepath.suffix == '.npz':
            data = np.load(filepath)
            calibration_matrix = np.array(data['calibration_matrix'])
            bias_vector = np.array(data['bias_vector']) if data['bias_vector'] is not None else None
        else:
            with open(filepath, 'r') as f:
                data = json.load(f)
            calibration_matrix = np.array(data['calibration_matrix'])
            bias_vector = np.array(data['bias_vector']) if data['bias_vector'] is not None else None
        
        self.calibration_matrix = calibration_matrix
        self.bias_vector = bias_vector
        
        print(f"Calibration loaded from: {filepath}")
        print(f"  Calibration matrix shape: {calibration_matrix.shape}")
        print(f"  Has bias: {bias_vector is not None}")
        
        return calibration_matrix, bias_vector
    
    def apply_calibration(self, sensor_data: np.ndarray) -> np.ndarray:
        """
        Apply calibration to sensor data.
        
        Args:
            sensor_data: 16x1 or Nx16 array of sensor readings
            
        Returns:
            force_torque: 6x1 or Nx6 array of calibrated forces/torques
        """
        if self.calibration_matrix is None:
            raise ValueError("No calibration matrix loaded")
        
        sensor_data = np.atleast_2d(sensor_data)
        
        if sensor_data.shape[-1] != self.sensor_channels:
            raise ValueError(f"Expected {self.sensor_channels} sensor channels, got {sensor_data.shape[-1]}")
        
        # Apply calibration: F = S @ C^T + b
        force_torque = sensor_data @ self.calibration_matrix.T
        
        if self.bias_vector is not None:
            force_torque += self.bias_vector
        
        return force_torque.squeeze()
    
    def run_live_mode(self, calibration_file: str, zmq_in_port: int = 5555, zmq_out_port: int = 5556):
        """
        Run live calibration mode, subscribing to ZMQ sensor data and publishing calibrated data.
        
        Args:
            calibration_file: Path to calibration file
            zmq_in_port: ZMQ port to subscribe to for raw sensor data
            zmq_out_port: ZMQ port to publish calibrated force/torque data
        """
        # Load calibration
        self.load_calibration(calibration_file)
        
        # Setup ZMQ subscriber for raw sensor data
        context = zmq.Context()
        sub_socket = context.socket(zmq.SUB)
        sub_socket.connect(f"tcp://localhost:{zmq_in_port}")
        sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        # Setup ZMQ publisher for calibrated data
        pub_socket = context.socket(zmq.PUB)
        pub_socket.bind(f"tcp://*:{zmq_out_port}")
        
        print(f"\nListening for sensor data on port {zmq_in_port}...")
        print(f"Publishing calibrated data on port {zmq_out_port}...")
        print("Press Ctrl+C to stop\n")
        
        print("Calibrated Force-Torque Output:")
        print("-" * 80)
        print("Time         Fx(N)    Fy(N)    Fz(N)    Mx(N⋅m)  My(N⋅m)  Mz(N⋅m)  |F|(N)   |M|(N⋅m)")
        print("-" * 80)
        
        try:
            while True:
                try:
                    # Receive data with timeout
                    if sub_socket.poll(0):  # 100ms timeout
                        data_str = sub_socket.recv_string()
                        data = json.loads(data_str)
                        
                        if 'sensor_moving_averages' in data:
                            sensor_values = np.array(data['sensor_moving_averages'])
                            
                            if len(sensor_values) == self.sensor_channels:
                                # Apply calibration
                                force_torque = self.apply_calibration(sensor_values)
                                
                                # Calculate magnitudes
                                force_mag = np.linalg.norm(force_torque[:3])
                                torque_mag = np.linalg.norm(force_torque[3:])
                                
                                # Create output data dictionary
                                output_data = {
                                    'timestamp': time.time(),
                                    'forces': force_torque[:3].tolist(),  # [Fx, Fy, Fz]
                                    'torques': force_torque[3:].tolist(),  # [Mx, My, Mz]
                                    'force_magnitude': float(force_mag),
                                    'torque_magnitude': float(torque_mag),
                                    'raw_sensors': sensor_values.tolist()
                                }
                                
                                # Publish calibrated data
                                pub_socket.send_string(json.dumps(output_data))
                                
                                # Print formatted output
                                timestamp = time.strftime("%H:%M:%S")
                                print(f"{timestamp}  {force_torque[0]:8.3f} {force_torque[1]:8.3f} "
                                      f"{force_torque[2]:8.3f} {force_torque[3]:8.4f} "
                                      f"{force_torque[4]:8.4f} {force_torque[5]:8.4f} "
                                      f"{force_mag:8.3f} {torque_mag:8.4f}")
                            else:
                                print(f"Warning: Expected {self.sensor_channels} sensors, got {len(sensor_values)}")
                        
                except zmq.ZMQError as e:
                    print(f"ZMQ error: {e}")
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")
                except Exception as e:
                    print(f"Error: {e}")
                    
        except KeyboardInterrupt:
            print("\n\nStopping live mode...")
        finally:
            sub_socket.close()
            pub_socket.close()
            context.term()


def main():
    parser = argparse.ArgumentParser(
        description="Calculate calibration matrix for force-torque sensor",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Calculate calibration from CSV
  python calc_calibration_matrix.py --csv calibration_data.csv --output calibration.json
  
  # Run live mode with existing calibration
  python calc_calibration_matrix.py --live --calibration calibration.json
  
  # Calculate without bias term
  python calc_calibration_matrix.py --csv data.csv --output cal.json --no-bias
        """
    )
    
    parser.add_argument('--csv', type=str, help='Input CSV file with calibration data')
    parser.add_argument('--output', type=str, help='Output file for calibration matrix (.json or .npz)')
    parser.add_argument('--live', action='store_true', help='Run live mode with ZMQ sensor data')
    parser.add_argument('--calibration', type=str, help='Calibration file to use in live mode')
    parser.add_argument('--in-port', type=int, default=5555, help='ZMQ port for input sensor data (default: 5555)')
    parser.add_argument('--out-port', type=int, default=5556, help='ZMQ port for output calibrated data (default: 5556)')
    parser.add_argument('--no-bias', action='store_true', help='Calculate calibration without bias term')
    
    args = parser.parse_args()
    
    calibrator = ForceTorqueCalibrator()
    
    if args.live:
        # Live mode - apply calibration to real-time data
        if not args.calibration:
            print("Error: --calibration file required for live mode")
            return 1
        
        calibrator.run_live_mode(args.calibration, args.in_port, args.out_port)
        
    elif args.csv:
        # Calibration mode - calculate matrix from CSV
        if not args.output:
            # Default output name based on input
            output_path = Path(args.csv).stem + '_calibration.json'
        else:
            output_path = args.output
        
        # Load data
        sensor_data, force_torque_data = calibrator.load_calibration_data(args.csv)
        
        # Calculate calibration
        calibration_matrix, bias_vector = calibrator.calculate_calibration_matrix(
            sensor_data, force_torque_data, use_bias=not args.no_bias
        )
        
        # Save calibration
        metadata = {
            'source_csv': args.csv,
            'num_samples': len(sensor_data),
            'use_bias': not args.no_bias
        }
        calibrator.save_calibration(output_path, calibration_matrix, bias_vector, metadata)
        
        # Print calibration matrix info
        print(f"\nCalibration matrix shape: {calibration_matrix.shape}")
        print(f"Matrix norm: {np.linalg.norm(calibration_matrix):.4f}")
        
        # Analyze sensor contributions
        print("\nSensor contribution analysis (sum of absolute weights per sensor):")
        sensor_weights = np.sum(np.abs(calibration_matrix), axis=0)
        for i, weight in enumerate(sensor_weights):
            magnet_num = i // 4 + 1
            sensor_in_magnet = i % 4 + 1
            print(f"  Sensor {i+1:2d} (Magnet {magnet_num}, Sensor {sensor_in_magnet}): {weight:.4f}")
        
    else:
        parser.print_help()
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())