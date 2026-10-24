# SONIC with Enterprise Pico WebXR

This blueprint uses the headset browser's full-body tracking. It does not use
an APK, XRoboToolkit PC Service, gRPC, or the native Pico workaround. It composes
the existing SONIC stack, adding browser body input and stick teleoperation.
There is still one policy controller and one motor-command owner.

## Start in simulation

macOS simulation selects ONNX CPU inference automatically. Hardware still
requires CUDA; the CPU option is rejected without a simulator. On macOS, install
the simulation dependencies and model release before starting:

```bash
uv sync --extra control --extra sim
uv run --no-sync dimos-sonic-models
uv run --no-sync dimos --simulation mujoco --transport zenoh run unitree-g1-sonic-webxr-teleop
```

For a CUDA host, use the [SONIC installation instructions](/dimos/robot/unitree/g1/sonic.md)
before running the same blueprint command.

Use DimOS Desktop's Controller app for cameras and policy controls. This blueprint
does not launch Rerun.

Open `https://<simulation-host>:8443/teleop` in the enterprise Pico browser.
Trust the host certificate using the WebXR setup instructions. The page requires
the browser's `body-tracking` feature and requests an immersive AR session.
Unsupported browsers must fail, not quietly substitute wrist/head tracking.
Both controllers and all 24 named body joints must be available.

## Controls

| Input | Result |
| --- | --- |
| Desktop Activate | Fresh-feedback check, pose ramp, then live balancing. Hardware starts unarmed; the sim retains its existing automatic activation. |
| Left stick | Forward/back and lateral walking through MovementManager. Ignored in pose mode. |
| Right stick | Turning while walking; reference-heading adjustment in pose mode. |
| Release, then A+X | Toggle between walking and full-body pose tracking. Does not arm the robot. |
| Desktop Stop motion | Cancel navigation, zero velocity, and return from pose tracking while keeping balancing active. |
| A+B+X+Y | Latched software damping stop. Restart required. Not a physical power disconnect. |

Pose references are retargeted using NVIDIA's canonical SMPL skeleton, resampled
at 50 Hz into ten-frame windows, and passed through SONIC's encoder and decoder.
The tracked pose is a policy reference, not direct joint servo control. Changes
between pose and planner references blend the encoder token over one second;
the decoder and measured-state history continue running throughout.

Incomplete tracking, a recenter, backwards capture time, a capture gap over
150 ms, or missing body/controller input for 250 ms exits pose mode. The last
valid pose remains the reference while a fresh stationary plan is computed.
Planner failure still invokes the existing latched damping watchdog. Tracking
recovery never re-engages pose mode without a fresh A+X release/press.

`G1SonicPolicyModule.status()` includes `webxr.mode`, `webxr.reason`, buffered
frames and body age. Clips, three-point hints and arm targets cannot replace a
full-body reference while WebXR owns it. Stop pose mode first.

## Check before hardware

In simulation, verify walking, A+X entry, asymmetric arm motion, return to
walking, headset disconnect/reconnect, recentering, Desktop stop and damping.
Also inspect the policy and planner timing in `status()` on the intended
host. Local synthetic-pose tests are not a headset or physical-robot acceptance
test. Keep gantry support and a separate physical stop for the later G1 test.

The original retargeting and token-transition approach comes from Tom's
`cc/feat/pico-teleop-sonic` work (#3721), adapted here to the standalone controller
rather than the old ControlCoordinator task.
