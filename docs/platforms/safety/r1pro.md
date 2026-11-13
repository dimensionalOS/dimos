# Galaxea R1 Pro Safety

Read [Robot Safety](/docs/platforms/safety/index.md) first. This page adds what is specific to the R1 Pro.

![R1 Pro standing upright](assets/r1pro-overview.png)

Two 7-joint arms on a 4-joint torso, on a wheeled base. The base can drive and the torso can fold and stand up, so keep a clear floor around the whole robot, not just in front of the arms.

## Switches and E-stop

![R1 Pro soft switch, emergency stop button, power switch and charging port](assets/r1pro-switches.png)

| Control | Where | What it does |
|---|---|---|
| Emergency stop button | Top of the back of the chassis | Stops all motion. It does **not** cut power. |
| Power switch (rocker) | Bottom of the back of the chassis | Main power. |
| Soft switch | Under the torso, at the back | Turns the computer and drives on and off. |
| Charging port (48 V) | Bottom of the back of the chassis | Charger only. |

## Remote controller

![R1 Pro remote controller](assets/r1pro-remote.png)

![Remote controller switch positions for each mode](assets/r1pro-remote-modes.png)

- Before you turn the remote on, push all four switches (SWA, SWB, SWC, SWD) **up**.
- Whenever you are not driving, keep all four switches up. The robot stays stopped.
- **SWD down = emergency stop**, in any mode.

## Power on

1. Floor clear around the robot, nothing under or next to the arms.
2. Emergency stop button released. Remote on, all switches up.
3. Turn the power switch on.
4. Press the soft switch and wait for the blue light.
5. Start dimos on the robot, for example `dimos run r1pro-coordinator`. See the [R1 Pro README](https://github.com/dimensionalOS/dimos/blob/main/dimos/robot/galaxea/r1pro/README.md) for the full setup and the other blueprints.

## Power off

1. `dimos stop`.
2. Hold the arms steady, or make sure nothing is under them. They can drop when power is cut.
3. Hold the soft switch until the blue light goes off.
4. Turn the power switch off.

## After an E-stop

The R1 Pro needs a full restart after its E-stop (button or SWD) has been used. Releasing it is not enough; carrying on without a restart makes the robot misbehave.

1. `dimos stop`.
2. Release the E-stop and put SWD back up.
3. Turn the power switch off, then follow [Power on](#power-on) from the top.

Images: [Galaxea Dynamics R1 Pro documentation](https://docs.galaxea-dynamics.com/R1Pro/en/docs/2026/getting_started/quick_start/r1pro_quick_start/).
