# Run π0.5 YAM on the real dual OpenYAM

This is the procedure used on the Thor machine on 2026-10-29. OpenPI serves the
`robocurve/pi0.5-yam` checkpoint; dimOS reads the three cameras and joint state,
then sends policy actions to both arms. **Starting the dimOS blueprint enables
physical arm motion.** Keep the workspace clear, have a way to stop the arms,
and watch the robot throughout the run.

## 1. Check the local installation

Run these from the dimOS checkout:

```bash
cd /home/dimos/demiurge/dimos
test -x ../openpi-yam/.venv/bin/python
test -d ../robotwin-pi05-checkpoint-eval/pi0.5-yam/params
test -x .venv/bin/dimos
```

The server launcher is `scripts/openpi/start_server.sh`. It uses those sibling
OpenPI and checkpoint paths by default. If the OpenPI client has not yet been
installed into the dimOS environment, install it once:

```bash
uv pip install --python .venv/bin/python --no-deps \
  -e /home/dimos/demiurge/openpi-yam/packages/openpi-client
```

## 2. Check the hardware mapping

The current blueprint assigns `can0` to the **left** arm and `can1` to the
**right** arm. Confirm which USB CAN adapter is which after reconnecting them;
interface numbers can change. It expects `/dev/video10` for the overhead camera,
`/dev/video16` for the left wrist, and `/dev/video4` for the right wrist. Confirm
that these devices are present and show the intended views before starting arm
control. If the mapping changed, update the blueprint configuration in
`dimos/robot/manipulators/dual_openyam/blueprints/openpi.py` first.

Configure the two CAN interfaces at 1 Mbps:

```bash
uv run --no-sync dimos hardware can setup can0
uv run --no-sync dimos hardware can setup can1
uv run --no-sync dimos hardware can status can0
uv run --no-sync dimos hardware can status can1
```

## 3. Start checkpoint inference

Check whether the server is already running. Do not start a second copy on the
same port.

```bash
systemctl --user status dimos-openpi-yam
```

If it is not running, start it:

```bash
cd /home/dimos/demiurge/dimos
systemd-run --user --unit=dimos-openpi-yam --collect \
  --property=WorkingDirectory="$PWD" "$PWD/scripts/openpi/start_server.sh"
journalctl --user -u dimos-openpi-yam -f
```

Wait for checkpoint loading to finish. The server listens on localhost port
8000. The first inference compiles the model and can take longer than later
requests. Use Ctrl-C to leave `journalctl`; this does not stop the server.

## 4. Start the real-arm policy loop

With the robot workspace clear and the server running:

```bash
cd /home/dimos/demiurge/dimos
.venv/bin/python scripts/openpi/run_interactive.py
```

To open both grippers and move both arms to literal zero joint angles before
the first policy action, use `--reset` instead:

```bash
.venv/bin/python scripts/openpi/run_interactive.py --reset
```

With `--reset`, the policy client starts paused. dimOS commands both grippers
fully open, verifies their measured openings, then uses its joint trajectory
task to move the twelve arm joints to zero. It checks that the measured arm
positions are within 0.05 radians before starting policy inference. The move
is planned at about 0.25 radians per second per joint. This is the literal
zero pose; dimOS's named OpenYAM home pose instead has joints 2 and 3 at
1.047 radians.
The joint trajectory task does not plan around obstacles, so clear the sweep
of both arms before using this option.
If the reset fails, the launcher leaves the policy paused and requests an
operator hold. Inspect the robot before retrying.

The default instruction is `pick up the red cube`. The blueprint connects to
the server on `127.0.0.1:8000`, captures live images from all three cameras,
and executes 16-action chunks on the two arms. The interactive launcher starts
the dimOS daemon and keeps this terminal open for control: press `t` to pause,
`r` to resume, or `q`/Ctrl-C to pause, open both grippers, move both arms to
literal zero, and leave the console. The exit reset uses the same checked arm
trajectory as `--reset`; it briefly releases the operator hold while the policy
is paused, then engages a new hold at zero. If the reset fails, the console
reports the error and requests an operator hold. Leaving the console
does not stop dimOS or disable torque. For logs, use a separate terminal with
`.venv/bin/dimos log -f` and look for `OpenPI action chunk received` and `OpenPI
action chunk published`.

### Pause and resume without disabling torque

The interactive launcher above handles `t` and `r`. These equivalent commands
also work from a separate terminal while the dimOS blueprint is running:

```bash
cd /home/dimos/demiurge/dimos
.venv/bin/python scripts/openpi/policy_control.py pause
.venv/bin/python scripts/openpi/policy_control.py status
.venv/bin/python scripts/openpi/policy_control.py resume
```

`pause` waits for the current 16-action chunk to finish, blocks the next policy
inference, then requests the coordinator's higher-priority operator hold. The
hold commands each joint's measured position while the motors remain enabled.
`resume` releases the hold and lets the client infer again from fresh live
observations. Ctrl-C in `.venv/bin/dimos log -f` only closes the log viewer; it
does not pause the robot. These commands require a dimOS run started with this
version of the blueprint and client.

## 5. Stop and inspect

Stop arm control first, then stop the inference server:

```bash
cd /home/dimos/demiurge/dimos
.venv/bin/dimos stop
systemctl --user stop dimos-openpi-yam
```

Inspect a saved run with `.venv/bin/dimos log` or under `logs/<run-id>/main.jsonl`.
The verified run `20261029-025847-0da2-openpi-dual-openyam` published 35 action
chunks and showed measured motion on both arms. A successful pickup was not
verified. If inference fails, policy execution stops; inspect the server journal
and dimOS log before restarting the run.

For implementation details and the verification record, see [README.md](README.md).
