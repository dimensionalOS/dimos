# Robocurve π0.5 with DimOS

This deployment uses separate Python environments: OpenPI/JAX serves inference;
DimOS owns cameras, CAN and command execution. The checkpoint is
`robocurve/pi0.5-yam`. The server reconstructs the public model-card contract;
the private `yam_pi05` training preprocessing has not been verified.

Start the OpenPI server in the background on this Thor:

```bash
cd /home/dimos/demiurge/dimos
systemd-run --user --unit=dimos-openpi-yam --collect \
  --property=WorkingDirectory="$PWD" "$PWD/scripts/openpi/start_server.sh"
journalctl --user -u dimos-openpi-yam -f
```

The launcher defaults to the sibling `openpi-yam` environment and downloaded
`robotwin-pi05-checkpoint-eval/pi0.5-yam` checkpoint. Override these with
`OPENPI_ROOT` and `YAM_CHECKPOINT` when invoking the script directly. It configures
the Thor CUDA library path and XLA flag used in the verified deployment.
To run in the foreground, use `scripts/openpi/start_server.sh`.

Do not start a second server while the service is active. Check and stop it with:

```bash
systemctl --user status dimos-openpi-yam
systemctl --user stop dimos-openpi-yam
```

The existing OpenPI checkout includes an Orbax metadata compatibility patch
needed by this checkpoint. The server binds localhost by default; `--host` and
`--port` are configurable. Startup includes checkpoint loading; the first
inference additionally compiles the model. Warm the server before hardware use.

The hardware blueprint currently uses USB CAN `can0` for left and `can1` for
right. Confirm the USB device mapping after reconnecting: interface numbering
can change. Configure both at 1 Mbps using the existing DimOS CAN command:

```bash
uv run --no-sync dimos hardware can setup can0
uv run --no-sync dimos hardware can setup can1
```

Camera mapping is `/dev/video10` top, `/dev/video16` left wrist and
`/dev/video4` right wrist. The bridge keeps all three cameras open, grabs each before decoding, and publishes RGB images at 15 Hz. Device paths, dimensions and FPS are module configuration fields.

Install the lightweight client into the DimOS environment without resolving or
replacing the existing Thor hardware dependencies:

```bash
uv pip install --python .venv/bin/python --no-deps \
  -e /home/dimos/demiurge/openpi-yam/packages/openpi-client
```

Start the hardware policy loop (this enables arm control):

```bash
.venv/bin/dimos run openpi-dual-openyam --daemon
.venv/bin/dimos log -f
```

The default instruction is `pick up the red cube`. Stop with:

```bash
.venv/bin/dimos stop
```

The client packs state and actions as left six joints, left gripper, right six
joints, right gripper. It converts named coordinator state into this order and
separates arm and gripper commands on output. Grippers use normalized opening
(0 closed, 1 open). Images are RGB, padded/resized to 224 square with OpenPI's
image helper. Each response must contain 16 finite absolute 14-value actions;
commands are streamed at 30 Hz. Inference is synchronous between chunks, so this
is not continuous 30 Hz replanning. Observations older than three seconds at
receipt-time checks are rejected before inference and before action execution.

Verified so far: real images and state reach inference, valid action chunks
reach the controller, and regression tests cover state ordering and serializable
arm/gripper commands. A successful cube pickup has **not** been verified. Early
policy outputs stayed near the zero pose. With persistent camera streams, later
runs produced substantial measured motion on both arms. Controller
"Trajectory completed" logs describe interpolation, not measured goal attainment.

The client uses bounded connection and inference waits (`timeout_s`, default 30
seconds) and closes the socket on failure. A failed inference stops policy
execution; restart the DimOS run after fixing the server. `prompt`, `timeout_s`
and `max_observation_age_s` are native OpenPIClient module configuration fields.
Stopping the client closes the websocket before joining its worker thread.

## Verification on this Thor (2026-10-29)

The corrected client ran with default Zenoh transports and repeatedly delivered
16-point chunks to the coordinator. One stale observation was rejected. No LCM
override is required.

With policy execution stopped, a two-second coordinator trajectory requested
+0.120 radians on each arm's first joint. Measured changes were +0.116 left and
+0.113 right. A return trajectory finished within 0.009 radians of the original
positions. This verifies measured hardware tracking separately from policy
quality.

The live server was also queried with synchronized frames/state from episode 0
of `allenai/19012026-block-13`, using its recorded instruction `spell out NEURIPS`.
Quantile-normalized chunk MSE at 0, 5, 10, 20 and 40 seconds was respectively
0.000059, 0.003761, 0.000276, 0.002642 and 0.000267. These are five selected chunks,
not a reproduction of the model card's full evaluation or proof of private
preprocessing parity. The predictions included substantial nonzero arm poses.
The early live cube scene produced near-rest targets. The later managed-server
run with persistent cameras produced substantial motion on both arms; a pickup
is still unverified.

The final running deployment uses the repository server launcher under
`dimos-openpi-yam.service` and the `openpi-dual-openyam` DimOS daemon. In run
`20261029-025847-0da2-openpi-dual-openyam`, 35 action chunks had been published
with no logged errors at the final check. Measured left shoulder/elbow positions
were 1.586/1.316 radians and right shoulder/elbow positions 0.347/0.215 radians.
The camera confirmed changed arm poses. These readings establish live policy
execution through DimOS, not grasp success.
