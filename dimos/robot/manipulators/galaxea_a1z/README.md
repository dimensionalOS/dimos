# Galaxea A1Z + G1Z

The A1Z integration uses native Linux SocketCAN, the vendor's 250 Hz MIT
position-control loop, the G1Z URDF for gravity compensation, and the vendor
G1Z gripper implementation.

## Host setup

The G1Z requires the vendor SDK's `gripper` branch; vendor `main` does not
accept `with_gripper` and cannot actuate CAN motor 7.

Run the one-command hardware setup as your normal user. The pinned vendor SDK
and, on macOS, the PyUSB/gs-usb transport are installed from the locked
`galaxea-a1z` dependency group. Linux then configures SocketCAN; macOS installs
or verifies Homebrew libusb and checks the attached HHS adapter without
enabling the arm:

```bash
./dimos/robot/manipulators/galaxea_a1z/scripts/setup_a1z.sh
```

Add `--with-lerobot` to install the dataset, training, and live-policy runtime
in the same environment. Use `--sdk-only` to synchronize and verify Python
dependencies without checking or configuring attached CAN hardware:

```bash
./dimos/robot/manipulators/galaxea_a1z/scripts/setup_a1z.sh --with-lerobot
./dimos/robot/manipulators/galaxea_a1z/scripts/setup_a1z.sh --sdk-only
```

The equivalent manual dependency sync is:

```bash
uv sync --locked --inexact \
  --group galaxea-a1z \
  --extra learning \
  --extra lerobot
```

Always include `--group galaxea-a1z` in later exact syncs, or rerun the setup
script. The group keeps the non-PyPI vendor SDK and macOS transport inside
`uv.lock`; no manual `uv pip install` step is required.

DimOS deliberately has no Linux userspace-CAN fallback. After boot or
reconnecting the HHS adapter, the one-command setup can be rerun, or the CAN
portion can be invoked directly to bind the adapter to the kernel driver,
configure the stable `a1zcan` SocketCAN interface, and verify transmission:

```bash
sudo ./dimos/robot/manipulators/galaxea_a1z/scripts/setup_a1z_can.sh
```

Do not start DimOS unless the script prints `A1Z CAN setup passed`. Galaxea's
HHS USB-CANFD adapter is incompatible with `gs_usb` in some Linux kernels. An
affected kernel still creates a normal-looking, UP CAN interface but drops
every transmission. The setup script detects that misleading state and prints
the supported-kernel and exact-kernel patch options. Galaxea recommends kernel
6.8.0-124 or newer; Jetsons and other pinned-kernel hosts require a persistent
driver patch built for their exact kernel.

The A1Z has no brakes. Support the arm and keep the workspace clear before
starting a hardware blueprint. Enabling the G1Z also initializes the gripper.

## Camera, teach, replay, and LeRobot export

The teach command uses a standard UVC camera through DimOS's generic `Webcam`
and `CameraModule`. On Linux, index N normally maps to `/dev/videoN`. On macOS,
grant the terminal application Camera access in **System Settings → Privacy &
Security → Camera**, then select the AVFoundation device index with
`--camera-index N`. Use the index reported by OpenCV, not `ffmpeg`: their
AVFoundation device ordering can differ. Each saved episode contains 640x480
RGB images at 15 Hz plus the measured six arm joints and gripper position.

After the CAN setup check passes, record one or more episodes:

```bash
uv run dimos a1z teach --task "pick up the object"
```

On the hackathon Mac, OpenCV enumerates the external KS2A418 camera as index 0:

```bash
uv run --no-sync dimos a1z teach --camera-index 0 --task "pick up the object"
```

While recording, press `SPACE` to save the current episode or `d` to discard
it. While idle, press `d` to discard the most recently saved episode; replay
and dataset export will exclude it.

The command prints the Memory2 `.db` path. Replay a saved episode by passing
that path (the latest saved episode is selected by default):

```bash
uv run dimos a1z replay ~/.local/state/dimos/recordings/a1z_teach_<timestamp>.db
```

Convert the same recording into a LeRobot v3 dataset with synchronized video,
seven-element observation state, and seven-element action:

```bash
uv run dimos dataprep build \
  --source ~/.local/state/dimos/recordings/a1z_teach_<timestamp>.db \
  --output ./a1z_lerobot_dataset \
  --format lerobot \
  --config dimos/learning/dataprep/galaxea_a1z_state_config.json

uv run dimos dataprep inspect ./a1z_lerobot_dataset
```

The LeRobot output stores images as
`observation.images.image`, the measured arm and gripper state as
`observation.state`, and the next measured state as `action`.

Install or verify the complete locked training runtime:

```bash
./dimos/robot/manipulators/galaxea_a1z/scripts/setup_a1z.sh \
  --sdk-only \
  --with-lerobot
```

Train an ACT checkpoint from the converted local dataset:

```bash
uv run lerobot-train \
  --dataset.repo_id=galaxea_a1z \
  --dataset.root=./a1z_lerobot_dataset \
  --policy.type=act \
  --policy.device=mps \
  --policy.push_to_hub=false \
  --output_dir=outputs/a1z_act \
  --job_name=a1z_act \
  --wandb.enable=false
```

Use `--policy.device=cuda` on an NVIDIA training host. Apple-silicon Macs use
`mps`; CPU-only hosts can use `cpu` for a slow smoke test.

ACT is the tested A1Z policy type, but the runtime uses LeRobot's policy
factory rather than hard-coding ACT. Another LeRobot policy type can be used
when its checkpoint exposes the same single RGB image input, seven-value state
input, and seven-value action output.

After the host setup passes, run the trained policy. Loading and hardware
initialization require confirmation, and inference starts only after live RGB
and seven-joint observations are ready:

```bash
uv run dimos a1z run-policy \
  outputs/a1z_act/checkpoints/last/pretrained_model \
  --camera-index 0 \
  --device mps \
  --task "pick up the object" \
  --duration 20
```

This command is the one-checkpoint hardware test path. It installs that
checkpoint in the policy catalog under the name `default`, starts the same
camera/coordinator/policy module stack used by a full blueprint, and invokes
`execute_learned_policy("default")`.

## Turn trained policies into an agentic robot

Once individual checkpoints pass `run-policy`, put them in one catalog and
give the behaviors stable, meaningful skill names. Checkpoints are loaded on
first use and cached, so the running robot can execute several trained
behaviors without restarting. The complete blueprint can stay small:

```python
from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.learning.lerobot_policy import LeRobotPolicyConfig, LeRobotPolicyModule
from dimos.robot.manipulators.galaxea_a1z.blueprints.basic import (
    make_a1z_learned_policy_blueprint,
)


class HackathonPolicies(LeRobotPolicyModule):
    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def pick_up_cup(self) -> str:
        """Pick up the wooden cup from the table."""
        return self.start_configured_policy("pick_up_cup", tool_name="pick_up_cup")

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def place_cup(self) -> str:
        """Place the held wooden cup on the table."""
        return self.start_configured_policy("place_cup", tool_name="place_cup")


a1z_policies = make_a1z_learned_policy_blueprint(
    policies={
        "pick_up_cup": LeRobotPolicyConfig(
            policy_path="outputs/pick_up_cup/checkpoints/last/pretrained_model",
            task="pick up the wooden cup",
            device="mps",
            default_duration=20.0,
        ),
        "place_cup": LeRobotPolicyConfig(
            policy_path="outputs/place_cup/checkpoints/last/pretrained_model",
            task="place the wooden cup on the table",
            device="mps",
            default_duration=20.0,
        ),
    },
    policy_module=HackathonPolicies,
    camera_index=0,
)

A1Z_AGENT_PROMPT = """You control a Galaxea A1Z manipulation arm.
Use the available learned manipulation skills to carry out the user's request.
Call only one movement skill at a time and report failures clearly.
"""

a1z_learned_agent = autoconnect(
    a1z_policies,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=A1Z_AGENT_PROMPT),
)
```

Expose `a1z_learned_agent` as a runnable blueprint using the normal DimOS
blueprint registration process, then start it like any other stack:

```bash
uv run dimos run a1z-learned-agent --daemon
uv run dimos humancli
```

The same running blueprint can be driven without the interactive terminal:

```bash
uv run dimos agent-send "pick up the wooden cup, then place it back down"
uv run dimos mcp list-tools
uv run dimos mcp call pick_up_cup
```

The composition is now standard DimOS: the A1Z helper supplies the hardware,
servo coordinator, camera, and one multi-policy module; `McpServer` exposes the
named skills; and `McpClient` lets the language agent select and sequence those
skills. Adding a trained behavior means adding one catalog entry and one small
documented `@skill` wrapper. It does not require a new executor module or any
changes to DimOS core.
