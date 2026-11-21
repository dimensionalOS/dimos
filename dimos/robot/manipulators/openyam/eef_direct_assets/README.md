# OpenYAM SuperReach EEF direct 11.5k

`teleop-webxr-openyam-superreach-eef-direct-11p5k-real` is an opt-in physical
blueprint. It selects the historical `noise-dr-10ms@11.5k` actor, not the
ordinary OpenYAM reference-driven policy. It runs the CPU actor on the nominal
50 Hz policy cadence with `obs` float32 `[1, 311]` (16 oldest-to-newest
19-value history rows plus XYZ and WXYZ) and `q_des` float32 `[1, 6]`.
The six outputs are absolute joint targets in radians, in native arm order.
The native MIT write uses the trained Kp/Kd, zero desired velocity and zero
feedforward effort. The jaw continues through the stock gripper task.

## Provisioning

- Use the repository's `manipulation` dependencies (`planning`, `sim`, `cpu`),
  including its locked `can-motor-control==0.0.12` and the CPU ONNX Runtime.
  No modified SDK or private native extension is required. The lockfile pins
  wheels by platform; the published Linux x86-64 wheel's native module matches
  the frozen release.
- Provision the existing `yam_description` resource using DimOS's Git LFS
  data provisioning. The base pins `data/.lfs/yam_description.tar.gz` at
  SHA-256 `db9a2160e414830eb1c2203b573ec7557563d3f963667c19752d7da7e3e940f1`.
  It contains the stock control, planning and visualization assets; its 29
  files match the frozen release's corresponding resource.
- Obtain `policy_eef_direct_noise_dr_10ms_11.5k.onnx` from an authorized
  artifact custodian for the frozen `noise-dr-10ms@11.5k` release. Copy it
  next to this README without adding it to Git. No public download is provided.
  Do not use a different checkpoint or re-export in its place.
- `tcp_fk.xml` is the checked-in, mesh-free FK description for the training
  TCP anchor; it is not a replacement for `yam_description`.

From this asset directory, verify the exact inputs with `sha256sum`:

```text
5de4bef7c67e85e3a3d175327dee75a7a0ee5cdb723457410c7cdd5c099ac1b0  policy_eef_direct_noise_dr_10ms_11.5k.onnx
99a4f14707937dcd7fd49c84e27e7e6ac96c55fa051651eb6b6a1dfcd7648863  tcp_fk.xml
```

The blueprint verifies both digests before assembling, and the actor worker
also verifies its ONNX ABI when started.

## Operator-owned configuration

Set `OPENYAM_EEF_DIRECT_CONFIG` to a local TOML file **outside** the repository.
It must specify `can_port` (the CAN selector for the arm intentionally bound to
right-hand Quest control), `zenoh_domain`, and `zenoh_scout_addr` (an isolated
local discovery group). Optional `quest_bind`, `quest_port`, `viser_bind`, and
`viser_port` override the loopback listener defaults (8443 and 8080). Select
only the authorized operator-network interface for off-host access.

By default the WebXR server uses DimOS's native HTTPS path. It uses the pair
under `assets/teleop_certs` or generates a local self-signed pair there if
needed (OpenSSL and a writable directory are required). A browser must trust
the selected endpoint's certificate to use WebXR/WSS. To override this
behavior, set `certs` to an operator-owned directory containing `cert.pem` and
`key.pem`; a relative path resolves against the TOML file. A supplied override
is validated before assembly. Do not commit certificates, keys, or site config.

**Not commissioned for motion.** Opening the real connection can enable
motors and calibrate the jaw before grip engagement. A zero-delta grip anchor
does not guarantee the actor's first target equals measured joints. Host write
receipts are not CAN or motor acknowledgments. Timing, collision clearance,
stop behavior and physical response remain unverified; only the operator may
commission or start this blueprint.
