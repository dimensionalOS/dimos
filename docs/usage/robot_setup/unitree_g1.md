# Unitree G1 Robot Setup Guide

This guide covers the setup and configuration of the Unitree G1 humanoid robot, including network connectivity, SDK installation, and basic usage.

To enable the robot for movement using the controller, note it can be different for different versions of Unitree G1's
1. L2 + B
2. L2 + Up
<!-- 3. R2 + A -->

### Safety Notes
- Always ensure the robot has clear space before enabling movement
- Keep the emergency stop button accessible
- When using low-level control, disable high-level motion services first
- Start with high-level control before attempting low-level motor control

```
(after pressing L2 + Up):  Current state: FSM 4: Unknown FSM 4
After sending stand command: FSM 200: Start
```

## Table of Contents
- [Network Configuration](#network-configuration)
- [System Requirements](#system-requirements)
- [Unitree SDK2 Python Installation](#unitree-sdk2-python-installation)
- [Verification](#verification)
- [Basic Usage Examples](#basic-usage-examples)
- [Robot Operation](#robot-operation)
- [Troubleshooting](#troubleshooting)

## Network Configuration

### Ethernet Connection
1. Connect the robot to your computer via Ethernet cable
2. Open the graphical network manager
3. Manually set your system's IP address to `192.168.123.100`
4. The robot's default Ethernet IP is: `192.168.123.164`

### SSH Access
```bash
ssh unitree@192.168.123.164
```
Password: `123`

### WiFi Connection
After connecting, you can find additional IP addresses:
```bash
hostname -I
```
The second address listed should allow SSH access even after disconnecting the Ethernet cable.

WiFi password (if needed): `888888888` or `00000000`

## System Requirements

### Hardware
- Unitree G1 humanoid robot
- Development computer (aarch64 or x86_64 architecture)

### Software
- Operating System: Ubuntu 20.04 LTS or later (tested on Ubuntu)
- Python: >= 3.8 (Python 3.10 recommended)
- Network interface for robot connection (Ethernet or WiFi)

### Required System Libraries
The following libraries should be installed on the system:
- cyclonedds (DDS middleware)
- gcc (version 9.4.0 or later)
- cmake (for building dependencies)
- git

## Unitree SDK2 Python Installation

The Unitree SDK2 Python package provides Python bindings for controlling and monitoring the G1 robot via DDS (Data Distribution Service).

### Step 1: Install System Dependencies

```bash
# Update package list
sudo apt update

# Install build tools
sudo apt install -y python3-pip git cmake build-essential libssl-dev

# Install UV (Python version manager) if not already installed
curl -LsSf https://astral.sh/uv/install.sh | sh
uv python install 3.10
```

### Step 2: Install CycloneDDS

CycloneDDS is required for DDS communication. Build from source:

```bash
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds
mkdir -p build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install -j$(nproc)
```

The library will be installed to `~/cyclonedds/install`.

### Step 3: Clone Unitree SDK2 Python

```bash
# Clone to /opt (recommended for system-wide access)
sudo git clone https://github.com/unitreerobotics/unitree_sdk2_python.git /opt/unitree_sdk2_python
sudo chown -R $USER:$USER /opt/unitree_sdk2_python
```

### Step 4: Install SDK in Virtual Environment

```bash
# Navigate to your project directory
cd /path/to/your/project

# Activate your virtual environment
source .venv/bin/activate

# Set CycloneDDS path and install SDK
cd /opt/unitree_sdk2_python
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
pip install -e .
```

Note: If you see numpy version conflicts, install a compatible version:
```bash
pip install "numpy<2.0,>=1.26"
```

### Step 5: Verify Installation

```bash
python -c "import unitree_sdk2py; print('SDK installed successfully!')"
python -c "from unitree_sdk2py import g1; print('G1 module loaded!')"
```

## Verification

### Test Import
Create a test script `test_sdk.py`:

```python
#!/usr/bin/env python3
import unitree_sdk2py

print("Successfully imported unitree_sdk2py")
print("Available modules:", dir(unitree_sdk2py))

# Test G1 module
from unitree_sdk2py import g1
print("Successfully imported G1 module")

# Test core functionality
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
print("Core imports successful!")
```

Run the test:
```bash
source .venv/bin/activate
python test_sdk.py
```

### Check Network Interface
Identify your network interface name (needed for SDK examples):
```bash
ifconfig
# or
ip addr show
```

Common interface names:
- `eth0` - Ethernet connection
- `wlan0` - WiFi connection
- `enp2s0` - Alternative Ethernet naming

## Basic Usage Examples

The SDK provides examples in `/opt/unitree_sdk2_python/example/`. Here are the most useful ones for G1:

### High-Level Examples (G1)

Located in `/opt/unitree_sdk2_python/example/g1/high_level/`:

1. **G1 Arm Control (7-DOF)**: Control the 7-DOF arm
```bash
python /opt/unitree_sdk2_python/example/g1/high_level/g1_arm7_sdk_dds_example.py <interface>
```

2. **G1 Arm Control (5-DOF)**: Control the 5-DOF arm
```bash
python /opt/unitree_sdk2_python/example/g1/high_level/g1_arm5_sdk_dds_example.py <interface>
```

3. **Arm Actions**: Predefined arm action sequences
```bash
python /opt/unitree_sdk2_python/example/g1/high_level/g1_arm_action_example.py <interface>
```

4. **Locomotion Control**: High-level walking/movement control
```bash
python /opt/unitree_sdk2_python/example/g1/high_level/g1_loco_client_example.py <interface>
```

Replace `<interface>` with your network interface (e.g., `eth0`, `wlan0`).

### Low-Level Control

For direct motor control:
```bash
python /opt/unitree_sdk2_python/example/g1/low_level/g1_low_level_example.py <interface>
```

Warning: Low-level control requires disabling high-level motion services first to avoid conflicts.

### Simple DDS Communication Test

Test basic DDS publish/subscribe:

Terminal 1 (Publisher):
```bash
python /opt/unitree_sdk2_python/example/helloworld/publisher.py
```

Terminal 2 (Subscriber):
```bash
python /opt/unitree_sdk2_python/example/helloworld/subscriber.py
```

### Python Code Example

Here's a minimal example to read the robot's state:

```python
#!/usr/bin/env python3
import time
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

class RobotStateReader:
    def __init__(self, network_interface):
        # Subscribe to low-level state
        self.sub = ChannelSubscriber(network_interface, "rt/lowstate", LowState_)
        self.sub.Init()

    def read_state(self):
        msg = LowState_()
        if self.sub.Read(msg):
            print(f"IMU Data - Quaternion: {msg.imu_state.quaternion}")
            print(f"Battery Voltage: {msg.bms_state.volt}")
            return msg
        return None

# Usage
if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python script.py <network_interface>")
        sys.exit(1)

    interface = sys.argv[1]
    reader = RobotStateReader(interface)

    print("Reading robot state... (Press Ctrl+C to stop)")
    try:
        while True:
            reader.read_state()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopped")
```

## Robot Operation

### Enabling Movement

## Configuration

### Environment Variables

For convenience, add to your `~/.bashrc`:

```bash
# Unitree SDK2 environment
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
export UNITREE_SDK2_PYTHON="/opt/unitree_sdk2_python"
export PATH="$CYCLONEDDS_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:$LD_LIBRARY_PATH"
```

Reload your shell:
```bash
source ~/.bashrc
```

## Troubleshooting

### Issue: "Could not locate cyclonedds"
**Solution**: Set the CYCLONEDDS_HOME environment variable:
```bash
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
```

### Issue: Import error when importing unitree_sdk2py
**Solution**: Ensure the virtual environment is activated and the SDK was installed with `-e` flag:
```bash
source .venv/bin/activate
cd /opt/unitree_sdk2_python
pip install -e .
```

### Issue: No data received from robot
**Solutions**:
1. Verify network connectivity: `ping 192.168.123.164`
2. Check network interface name: `ifconfig` or `ip addr show`
3. Ensure you're using the correct interface in your script
4. Verify the robot is powered on and network services are running

### Issue: Numpy version conflicts
**Solution**: Install a compatible numpy version:
```bash
pip install "numpy<2.0,>=1.26"
```

### Issue: Permission denied when accessing /opt/unitree_sdk2_python
**Solution**: Fix ownership:
```bash
sudo chown -R $USER:$USER /opt/unitree_sdk2_python
```

### Issue: Robot not responding to commands
**Solutions**:
1. Ensure high-level motion service is enabled (for high-level control)
2. Disable high-level motion service using the app (for low-level control)
3. Check that you pressed `L2+B` and `R2+UP` on the controller
4. Verify DDS communication is working using the helloworld examples

## Additional Resources

- [Unitree Developer Documentation](https://support.unitree.com/home/en/developer)
- [Sport Mode Services](https://support.unitree.com/home/en/developer/sports_services)
- [Basic Services](https://support.unitree.com/home/en/developer/Basic_services)
- [GitHub - Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [GitHub - Unitree SDK2 (C++)](https://github.com/unitreerobotics/unitree_sdk2)

## Additional Installation Requirements

### RealSense Camera
```bash
# Install from source or use pre-built packages
# Follow official Intel RealSense installation guide
```

### ZED SDK
```bash
# Download and install ZED SDK from Stereolabs
# https://www.stereolabs.com/developers/release/
```

---

Last updated: February 2026
