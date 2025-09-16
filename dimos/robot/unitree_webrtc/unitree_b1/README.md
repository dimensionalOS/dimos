# Unitree B1 Dimensional Integration

This module provides UDP-based control for the Unitree B1 quadruped robot with DimOS integration with ROS Twist cmd_vel interface.

## Overview

The system consists of two components:
1. **Server Side**: C++ UDP server running on the B1's internal computer
2. **Client Side**: Python control module running on external machine

Key features:
- 50Hz continuous UDP streaming
- 100ms command timeout for automatic stop
- Standard Twist velocity interface
- Emergency stop (Space/Q keys)
- IDLE/STAND/WALK mode control
- Optional pygame joystick interface

## Server Side Setup (B1 Internal Computer)

### Prerequisites

The B1 robot runs Ubuntu with the following requirements:
- Unitree Legged SDK v3.8.3 for B1
- Boost (>= 1.71.0)
- CMake (>= 3.16.3)
- g++ (>= 9.4.0)

### Step 1: Connect to B1 Robot

1. **Connect to B1's WiFi Access Point**:
   - SSID: `Unitree_B1_XXXXX` (where XXXXX is your robot's ID)
   - Password: `00000000` (8 zeros)

2. **SSH into the B1**:
   ```bash
   ssh unitree@192.168.12.1
   # Default password: 123
   ```

### Step 2: Build the UDP Server

1. **Add joystick_server_udp.cpp to CMakeLists.txt**:
   ```bash
   # Edit the CMakeLists.txt in the unitree_legged_sdk_B1 directory
   vim CMakeLists.txt
   
   # Add this line with the other add_executable statements:
   add_executable(joystick_server example/joystick_server_udp.cpp)
   target_link_libraries(joystick_server ${EXTRA_LIBS})```

2. **Build the server**:
   ```bash
   mkdir build
   cd build
   cmake ../
   make
   ```

### Step 3: Run the UDP Server

```bash
# Navigate to build directory
cd Unitree/sdk/unitree_legged_sdk_B1/build/
./joystick_server

# You should see:
# UDP Unitree B1 Joystick Control Server
# Communication level: HIGH-level
# Server port: 9090
# WARNING: Make sure the robot is standing on the ground.
# Press Enter to continue...
```

The server will now listen for UDP packets on port 9090 and control the B1 robot.

### Server Safety Features

- **100ms timeout**: Robot stops if no packets received for 100ms
- **Packet validation**: Only accepts correctly formatted 19-byte packets
- **Mode restrictions**: Velocities only applied in WALK mode
- **Emergency stop**: Mode 0 (IDLE) stops all movement

## Client Side Setup (External Machine)

### Prerequisites

- Python 3.10+
- DimOS framework installed
- pygame (optional, for joystick control)

### Step 1: Install Dependencies

```bash
# Install Dimensional
pip install -e .[cpu,sim]
```

### Step 2: Connect to B1 Network

1. **Connect your machine to B1's WiFi**:
   - SSID: `Unitree_B1_XXXXX`
   - Password: `00000000`

2. **Verify connection**:
   ```bash
   ping 192.168.12.1  # Should get responses
   ```

### Step 3: Run the Client

#### With Joystick Control (Recommended for Testing)

```bash
python -m dimos.robot.unitree_webrtc.unitree_b1.unitree_b1 \
    --ip 192.168.12.1 \
    --port 9090 \
    --joystick
```

**Joystick Controls**:
- `0/1/2` - Switch between IDLE/STAND/WALK modes
- `WASD` - Move forward/backward, turn left/right (only in WALK mode)
- `JL` - Strafe left/right (only in WALK mode)
- `Space/Q` - Emergency stop (switches to IDLE)
- `ESC` - Quit pygame window
- `Ctrl+C` - Exit program

#### Test Mode (No Robot Required)

```bash
python -m dimos.robot.unitree_webrtc.unitree_b1.unitree_b1 \
    --test \
    --joystick
```

This prints commands instead of sending UDP packets - useful for development.

## Safety Features

### Client Side
- **Command freshness tracking**: Stops sending if no new commands for 100ms
- **Emergency stop**: Q or Space immediately sets IDLE mode
- **Mode safety**: Movement only allowed in WALK mode
- **Graceful shutdown**: Sends stop commands on exit

### Server Side  
- **Packet timeout**: Robot stops if no packets for 100ms
- **Continuous monitoring**: Checks timeout before every control update
- **Safe defaults**: Starts in IDLE mode
- **Packet validation**: Rejects malformed packets

## Architecture

```
External Machine (Client)          B1 Robot (Server)
┌─────────────────────┐           ┌──────────────────┐
│ Joystick Module     │           │                  │
│ (pygame input)      │           │ joystick_server  │
│         ↓           │           │    _udp.cpp      │
│    Twist msg        │           │                  │
│         ↓           │  WiFi AP  │                  │
│ B1ConnectionModule  │◄─────────►│ UDP Port 9090    │
│ (Twist → B1Command) │ 192.168.  │                  │
│         ↓           │   12.1    │                  │
│  UDP packets 50Hz   │           │ Unitree SDK      │
└─────────────────────┘           └──────────────────┘
```

## Setting up ROS Navigation stack with Unitree B1

### Setup external Wireless USB Adapter on onboard hardware
This is because the onboard hardware (mini PC, jetson, etc.) needs to connect to both the B1 wifi AP network to send cmd_vel messages over UDP, as well as the network running dimensional


Plug in wireless adapter 
```bash
nmcli device status
nmcli device wifi list ifname *DEVICE_NAME*
# Connect to b1 network
nmcli device wifi connect "Unitree_B1-251" password "00000000" ifname *DEVICE_NAME*
# Verify connection 
nmcli connection show --active
```

### *TODO: add more docs*


## Troubleshooting

### Cannot connect to B1
- Ensure WiFi connection to B1's AP
- Check IP: should be `192.168.12.1`
- Verify server is running: `ssh unitree@192.168.12.1`

### Robot not responding
- Verify server shows "Client connected" message
- Check robot is in WALK mode (press '2')
- Ensure no timeout messages in server output

### Timeout issues
- Check network latency: `ping 192.168.12.1`
- Ensure 50Hz sending rate is maintained
- Look for "Command timeout" messages

### Emergency situations
- Press Space or Q for immediate stop
- Use Ctrl+C to exit cleanly
- Robot auto-stops after 100ms without commands

## Development Notes

- Packets are 19 bytes: 4 floats + uint16 + uint8
- Coordinate system: B1 uses different conventions, hence negations in `b1_command.py`
- LCM topics: `/cmd_vel` for Twist, `/b1/mode` for Int32 mode changes

## License

Copyright 2025 Dimensional Inc. Licensed under Apache License 2.0.
