# Unitree G1

## Requirements

- Unitree G1 EDU (need SDK/SSH access)
- Laptop/Desktop with Ubuntu 22.04/24.04 with CUDA GPU (recommended), or macOS (experimental)

## 1. Get SSH Access

### Get Ethernet Working
1. Plug an Ethernet cable from the robot into your Laptop
2. Open up your Laptop's graphical network manager, manually set the IP addr of your system to `192.168.123.100`
3. Run wired ssh command:

```bash
ssh unitree@192.168.123.164
# Password: 123
```

### Use Ethernet to get WiFi Working

After ssh-ing in, find additional IPs:
```bash
hostname -I
```
The second address allows SSH after disconnecting Ethernet.

WiFi passwords (varies by unit): `888888888` or `00000000`

### Network Interface Names

Common interface names needed for SDK examples:
- `eth0` / `enp2s0`: Ethernet
- `wlan0`: WiFi

Check with: `ip addr show`

### Remote Network

Recommended to setup [tailscale](https://tailscale.com/tailscale-ssh) to avoid needing to setup rounter specific configuraions for wireless control.

## 2. Install dimOS

SSH into the robot, then:

```bash
# pick the "developer" setup
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash -s -- --mode dev
```

#### Notes

For DDS dependencies, follow the [DDS setup guide](/docs/usage/transports/dds.md). The installer does not provision every robot SDK. If you're using the Unitree SDK directly, set:
```bash
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
```

## 3. Get the G1 in Sport Mode

**WARNING**: You *need* to have the G1 in a good physical position before running this.

Get the hand-held controller for the G1.

Note: this button combination may vary based on the model of the G1

1. If you have a gantry, hang the robot up where its feet are touching the floor, knees straight.
   - Press **L2 + B** (no movement, color change)
   - Press **L2 + Up** (should straighten out)
   - Press **R2 + A** (will attempt to self-balance)
2. If don't have a gantry, there is a make-shift way to get it working. You should get a second person to help.
   - Make the robot lie down flat on the ground
   - Press **L2 + B** (no movement, color change)
   - Press **L2 + Up** (should straighten out)
   - The robot will be super stiff now. Manually pick it up into a standing position and hold it there.
   - Press **R2 + A** (will attempt to self-balance)

## 4. Choose a controller

G1 has two blueprints. Run one at a time on a host with the Unitree DDS SDK
installed and network access to the robot. Set `--network-interface` to the
interface connected to it.

| Blueprint | Controller |
|-----------|------------|
| `unitree-g1` | Stock Unitree DDS locomotion |
| `unitree-g1-groot-wbc` | GR00T whole-body policy and bounded arm joint commands |

```bash
uv run dimos run unitree-g1 --network-interface eth0
# Or, after stopping the stock stack:
uv run dimos run unitree-g1-groot-wbc --network-interface eth0
```

The stock stack exposes locomotion RPCs through `dimos shell` and accepts
`cmd_vel`. GR00T accepts `/g1/cmd_vel` and arm targets on `/g1/joint_command`.
Both are control stacks; sensors, mapping, navigation, agents, WebXR and
collection are no longer bundled as G1-specific variants.

The old G1 simulation and replay paths have been removed. These blueprints
reject `--simulation` and `--replay`; sim2 support is pending integration.

## 5. Activate GR00T

Keep the remote and emergency stop reachable, with the robot supported in a
clear work area. GR00T starts unarmed with policy output in dry-run. In another
terminal on the same host:

```bash
uv run dimos hardware g1 status
uv run dimos hardware g1 activate
uv run dimos hardware g1 status
```

`activate` runs the pose ramp, asks for confirmation, then enables live policy
output. For separate steps, use `arm` followed by `enable`. The arming ramp
itself moves the robot even though policy output remains in dry-run.

```bash
uv run dimos hardware g1 disable
```

`disable` cancels arm trajectories, restores dry-run and disarms the controller.
The GR00T operator commands do not apply to the stock controller.
