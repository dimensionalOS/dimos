# Unitree G1

## Requirements

- Unitree G1 (stock firmware)
- Ubuntu 22.04/24.04 with CUDA GPU (recommended), or macOS (experimental)
- Python 3.12
- ZED camera (mounted at chest height) for perception blueprints

## Robot Setup

### Network

1. Connect robot via Ethernet
2. Set your machine's IP to `192.168.123.100`
3. Robot's default IP: `192.168.123.164`

### SSH

```bash
ssh unitree@192.168.123.164
# Password: 123
```

### WiFi

After Ethernet connection, find additional IPs:
```bash
hostname -I
```
The second address allows SSH after disconnecting Ethernet.

WiFi passwords (varies by unit): `888888888` or `00000000`

### Install DimOS on the G1

SSH into the robot, then:

```bash
bash <(curl -fsSL https://pub-4767fdd15e6a41b6b2ce2558d71ec8d9.r2.dev/install.sh)
```

### Controller

Enable movement (may vary by G1 version):
1. **L2 + B**
2. **L2 + Up**

FSM state transitions after enabling:
```
(after L2 + Up):  FSM 4: Unknown FSM 4
After stand command: FSM 200: Start
```

### Safety
- Always ensure clear space before enabling movement
- Keep the emergency stop accessible
- When using low-level control, disable high-level motion services first

## Running DimOS

### On Hardware

```bash
export ROBOT_IP=<YOUR_G1_IP>
dimos run unitree-g1-basic
```

### Simulation (no hardware needed)

```bash
uv pip install 'dimos[base,unitree,sim]'
dimos --simulation run unitree-g1-basic-sim
```

### Navigation (LiDAR nav stack)

```bash
dimos run unitree-g1-nav-onboard   # on robot
dimos run unitree-g1-nav-sim       # in simulation
```

### Agentic Control

```bash
export OPENAI_API_KEY=<YOUR_KEY>
dimos run unitree-g1-agentic
```

### Keyboard Teleop

```bash
dimos run unitree-g1-joystick
```

## Available Blueprints

| Blueprint | Description |
|-----------|-------------|
| `unitree-g1-basic` | Connection + visualization |
| `unitree-g1-basic-sim` | Simulation with basic nav |
| `unitree-g1-nav-onboard` | LiDAR nav stack on hardware |
| `unitree-g1-nav-sim` | LiDAR nav stack in simulation |
| `unitree-g1` | Navigation + perception + spatial memory |
| `unitree-g1-sim` | Perception stack in simulation |
| `unitree-g1-agentic` | Full stack with LLM agent and G1 skills |
| `unitree-g1-agentic-sim` | Agentic stack in simulation |
| `unitree-g1-full` | Agentic + SHM + keyboard teleop |
| `unitree-g1-joystick` | Navigation + keyboard teleop |
| `unitree-g1-detection` | Navigation + YOLO person detection |
| `unitree-g1-shm` | Perception with shared memory transport |

## Troubleshooting

### No data received from robot
1. `ping 192.168.123.164`
2. Verify correct network interface name (`ip addr show`)
3. Confirm robot is powered on

### Robot not responding to commands
1. Ensure high-level motion service is enabled (for high-level control)
2. Disable high-level motion service via the app (for low-level control)
3. Verify L2+B / L2+Up was pressed on controller
4. Test DDS with the SDK helloworld examples

## Resources

- [Unitree Developer Docs](https://support.unitree.com/home/en/developer)
- [Sport Mode Services](https://support.unitree.com/home/en/developer/sports_services)
- [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [Navigation Stack](/docs/capabilities/navigation/readme.md)
- [Visualization](/docs/usage/visualization.md)
- [Blueprints](/docs/usage/blueprints.md)
