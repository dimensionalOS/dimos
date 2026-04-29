# Unitree G1 — Hardware Setup

Practical reference for working with the physical G1 robot: network addresses, SSH access, controller buttons, and common issues.

For DimOS software setup, see [Getting Started](index.md).

## Controller

Enable movement (may vary by G1 version):
1. **L2 + B**
3. **L2 + Up**
4. **R2 + A**

FSM state transitions after enabling:
```
(after L2 + Up):  FSM 4: Unknown FSM 4
After stand command: FSM 200: Start
```

### Safety
- Always ensure clear space before enabling movement
- Keep the emergency stop accessible
- When using low-level control, disable high-level motion services first

## Network

### Ethernet
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

## Network Interface Names

Common interface names needed for SDK examples:
- `eth0` / `enp2s0` — Ethernet
- `wlan0` — WiFi

Check with: `ip addr show`

## Troubleshooting

### No data received from robot
1. `ping 192.168.123.164`
2. Verify correct network interface name
3. Confirm robot is powered on

### Robot not responding to commands
1. Ensure high-level motion service is enabled (for high-level control)
2. Disable high-level motion service via the app (for low-level control)
3. Verify L2+B / L2+Up was pressed on controller
4. Test DDS with the SDK helloworld examples

### CycloneDDS issues
DimOS handles DDS setup automatically. If you're using the Unitree SDK directly, set:
```bash
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
```

## External Resources

- [Unitree Developer Docs](https://support.unitree.com/home/en/developer)
- [Sport Mode Services](https://support.unitree.com/home/en/developer/sports_services)
- [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)
