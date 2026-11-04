# Unitree Go2 drive-train adapter

[`adapter.py`](adapter.py) — `UnitreeGo2TwistAdapter` (high-level): Twist `(vx, vy, wz)` via SportClient, with optional Rage Mode (`rage_mode=True`, ~2.5 m/s forward envelope). Auto-registered as `"unitree_go2"`, so a `ControlCoordinator` `HardwareComponent` with `adapter_type="unitree_go2"` drives it (see `dimos/control/blueprints/mobile.py` for the pattern). This is the one you want for teleop, navigation, or anything velocity-commanded.

---

## Running

Build the CycloneDDS C library via nix (once per machine — creates
`./result` symlink at the repo root, which acts as a GC root):

```bash
nix build nixpkgs#cyclonedds
```

Point your shell / venv at it so `cyclonedds-python` can find the C
library at install and runtime. Easiest: append to `.venv/bin/activate`
so it's set every time you activate the venv:

```bash
cat >> .venv/bin/activate <<EOF

# Nix-provided cyclonedds C library
export CYCLONEDDS_HOME=$(readlink -f ./result)
export LD_LIBRARY_PATH="\$CYCLONEDDS_HOME/lib:\${LD_LIBRARY_PATH:-}"
EOF
```

Re-activate the venv (`deactivate && source .venv/bin/activate`) so the
exports take effect, then install the `unitree-dds` extra (pulls
`unitree-sdk2py-dimos` + builds `cyclonedds-python` against the nix lib):

```bash
uv pip install -e ".[unitree-dds]"
```

Alternatives if you don't want to edit the activate script: `export`
both vars in `~/.bashrc`, or use `nix develop` (the flake's shell sets
them automatically), or `direnv` with `.envrc.nix`. See
[`docs/usage/transports/dds.md`](../../../../docs/usage/transports/dds.md).

Set the robot IP (`export ROBOT_IP=192.168.123.161`) before starting a blueprint that uses the adapter.

Troubleshooting:

| Symptom                               | Fix                                                         |
|---------------------------------------|-------------------------------------------------------------|
| `ModuleNotFoundError: unitree_sdk2py` | `uv pip install -e ".[unitree-dds]"`                        |
| `Could not locate cyclonedds`         | See [`docs/usage/transports/dds.md`](../../../../docs/usage/transports/dds.md) |
| DDS discovery failures                | Verify `ping $ROBOT_IP` succeeds; only one DDS domain active |
| `StandUp()` / `FreeWalk()` fails      | Power-cycle the Go2 on flat ground and retry                |
| Robot ignores velocity commands       | Wait ~5s for `[Go2] Locomotion ready` after startup       |
