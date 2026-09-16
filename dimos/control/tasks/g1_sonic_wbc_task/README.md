# G1 SONIC controller

SONIC runs the planner, encoder and decoder at a 50 Hz policy rate. It accepts
coordinator velocity commands, selectable gaits, motion clips and optional
upper-body reference targets. No headset or teleop service is required.

Install the workstation dependencies and NVIDIA assets:

```bash
uv sync --extra control --extra cuda --extra sim \
  --no-install-package onnxruntime --reinstall-package onnxruntime-gpu
source .venv/bin/activate
dimos-sonic-models
dimos-sonic-models --check
dimos --transport zenoh --simulation mujoco --viewer none run unitree-g1-sonic-wbc
```

The CPU and GPU ONNX Runtime distributions share the same Python package;
excluding the CPU distribution prevents it overwriting the CUDA provider.
Keep these options when syncing this environment again.

Assets live in the DimOS cache (`~/.cache/dimos/sonic` by default), outside
the Git LFS data directory. `SONIC_MODEL_DIR` overrides this location for
both the installer and blueprint. The installer pins the Hugging Face revision and verifies SHA-256 hashes for
the policy, planner and observation configuration. The 13 example motion clips
come from a pinned NVIDIA deployment revision. `--check` performs no downloads.
SONIC v1.1 is the single supported policy bundle.

MuJoCo starts in the SONIC pose and waits for a complete control command before
advancing physics. Simulation arms automatically. Commands from a second shell:

```bash
dimos --transport zenoh shell
```

```python
c = app.ControlCoordinator
c.task_invoke("sonic_wbc", "state_snapshot")
c.task_invoke("sonic_wbc", "list_locomotion_modes")
c.task_invoke("sonic_wbc", "list_motion_clips")
c.task_invoke("sonic_wbc", "play_motion_clip", {"name": "macarena_001__A545"})
c.task_invoke("sonic_wbc", "stop_motion_clip")
c.set_estop(True)
```

E-stop latches damping. Releasing a control, resetting runtime state, or arming
again cannot clear it; recovery requires restarting the stack. Hardware also
checks feedback and command freshness in its independent motor publisher.

For JetPack 5 or 6, use `bin/hardware/g1/setup-sonic`. It detects the release and
installs the matching ONNX Runtime and model assets; `--check` checks system
prerequisites only. JetPack 5 requires CUDA 11.8 with its compatibility driver
and cuDNN 8; JetPack 6 requires CUDA 12.6 and cuDNN 9.
Hardware starts unarmed with policy outputs in dry-run.
The updated controller has not been validated by another physical activation.

Squat and kneeling request zero translation. Centered sticks stop crawling
while retaining its posture. Face-down mode 7 is unavailable, matching NVIDIA's
selectable motion menu. Floor transitions remain timed; crawl stability and
the earlier hardware instability remain open validation issues.
