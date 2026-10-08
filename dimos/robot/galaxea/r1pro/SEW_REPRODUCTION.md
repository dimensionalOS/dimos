# Experimental R1 Pro SEW teleoperation

Independent Python/CPU reimplementation of [SEW-Mimic](https://sew-mimic.com/), [paper v1, Algorithms 1/2 and Appendix C](https://arxiv.org/html/2602.01632v1). No author implementation was run or copied: an official reusable algorithm repository and license were not verified. No SEW-TWIST data, RL training, or real hardware is used.

The optional `sew` backend retargets absolute body-relative shoulder/elbow/wrist directions and hand orientation. **It does not guarantee wrist task-position tracking.** `pink` remains the default and uses existing relative controller hand-pose teleoperation, with the torso held. These are deliberately different input semantics, not interchangeable objectives.

## Setup and fake-hardware command

Use the existing repository environment with its base CPU IK dependencies (`pin`, `pin-pink`, `qpsolvers[proxqp]`) and WebXR server dependencies. The tested CPU environment used Python 3.11.16, NumPy 2.4.6, SciPy 1.17.1, Pinocchio 4.1.0, and Pink 4.4.0. A minimal isolated environment can be provisioned from the repository root:

```sh
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python -e . numpy scipy pin pin-pink 'qpsolvers[proxqp]' 'cmeel-tinyxml2>=11,<12' fastapi uvicorn sse-starlette attrs requests ffmpeg-python python-multipart soundfile
```

The robot-description dependency is the same pinned `R1PRO_DESCRIPTION_SOURCE` / `R1PRO_MODEL_PATH` already defined in [config.py](config.py), using dimOS's existing `RobotDescriptionSource` cache. If already resolved, pass that local URDF path. To obtain only the small URDF instead of fetching the entire description/mesh repository:

```sh
mkdir -p .sew-local
gh api 'repos/userguide-galaxea/URDF/contents/R1Pro/urdf_r1pro_g1z_2026/urdf/r1pro_2026.urdf?ref=2e5d31e1784481a34d178006c0d0e18e0a84a82a' --jq .content > .sew-local/urdf.base64
python3 -c 'import base64,pathlib; p=pathlib.Path(".sew-local"); (p/"r1pro_2026.urdf").write_bytes(base64.b64decode((p/"urdf.base64").read_text()))'
```

A root LICENSE was not found in the inspected vendor listing. This change does not redistribute the vendor URDF or meshes. The solver requires an explicit local `--urdf-path` and does not automatically download data.

Run from the repository root:

```sh
export XDG_STATE_HOME="$PWD/.sew-local/state"
export XDG_CACHE_HOME="$PWD/.sew-local/cache"
.venv/bin/dimos run teleop-webxr-r1pro-fake \
  --solver-backend sew \
  --urdf-path "$PWD/.sew-local/r1pro_2026.urdf" \
  --viewer none --n-workers 2
```

This blueprint constructs only `mock_whole_body` for the 18 arm/torso joints and skips optional host configuration. Existing hardware blueprints and the default Pink solver remain unchanged. Stop with Ctrl+C. Replace `sew` with `pink` to use the original hand-pose task.

The local page is `http://127.0.0.1:8443/teleop`. **That URL is not a Quest deployment.** A separately configured trusted HTTPS reverse proxy with WebSocket forwarding and headset reachability is required. No certificate, CA trust, VPN, Serve, firewall, or public hosting is configured by this blueprint. Actual Quest body support, palm-to-flange mapping, and headset readability remain unverified. `--body-tracking-mode required` requests browser feature negotiation; otherwise negotiation is optional, but SEW always requires valid body samples before enabling.

## Retargeting and enable behavior

Release both grips, align with the robot, then hold both grips. The experimental fake-hardware gate requires 0.3 seconds within 8 degrees of arm/hand/chest orientation targets, 25 mm chest height, and 10 degrees of the selected arm joint solution. No catch-up command is emitted before alignment. These thresholds are not hardware validation.

Commands share the existing streaming command-envelope behavior: URDF position and velocity bounds, a 0.5 rad/s cap, and a 10 degree feedback envelope. Body/button loss, invalid targets, changed reference space, stalled timing, or solver failure disarms arms and torso together. Both controllers must report physical release before rearming; missing controller packets do not manufacture release.

Waist Pink IK tracks reachable chest pitch/yaw/height, without roll tracking or world-fixed end-effector compensation. The first valid body sample establishes heading/height reference, not an arm enable offset. The body adapter explicitly requires hips, upper/lower arms, wrists and palms in a floor reference space. It does not substitute controller poses for missing body joints. Hand mapping is a fixed experimental proper rotation requiring headset validation.

The XR HUD shows state, errors, thresholds, and measured/target front-projected arm skeletons. Backend, HTTP, WebSocket and browser errors feed its status text. The canvas is uploaded to a WebGL texture and drawn for each eye, even without video/episode data. The panel is world-placed from the initial viewer pose, 1.2 m forward and 0.6 m down, not continuously head-locked. Mock draw-call verification does not establish physical readability or recovery if the renderer itself crashes.

## Geometry and limits

Only the checked R1 Pro seven-axis Y-X-Z-Y-Z-Y-X signature with zero arm joint-origin rotations is supported. Joint-three/five axes point proximally, so signed -Z directions are used. R1 link offsets mean these axis proxies differ from physical shoulder-to-elbow and elbow-to-wrist vectors; the comparison logs both errors. Do not assume another seven-DoF or tabletop robot is compatible.

The analytic subproblems enumerate equivalent angles and reject limit-infeasible branches, rather than clipping and claiming an exact solution. Singular direction subproblems retain the seed where possible; the wrist Euler singularity is rejected. Sequential branch selection is not a complete global feasibility search. The paper's XPBD collision procedure is not reproduced. This implementation provides no collision avoidance, torque/contact model, dynamics feasibility, or safety guarantee.

## Reproduce the checks

With repository test dependencies installed:

```sh
CI=1 .venv/bin/python -m pytest -o addopts='' --confcutdir=dimos/control/tasks \
  dimos/robot/test_all_blueprints_generation.py \
  dimos/control/tasks/test_pose_target_ik.py \
  dimos/control/tasks/sew_teleop_task/test_task.py \
  dimos/control/test_joint_command_envelope.py \
  dimos/manipulation/planning/kinematics/test_sew_retargeting.py \
  dimos/core/coordination/test_system_configuration.py \
  dimos/robot/galaxea/r1pro/blueprints/basic/test_r1pro_sew_fake.py -q
.venv/bin/python -m dimos.robot.galaxea.r1pro.demo_sew_offline \
  --urdf .sew-local/r1pro_2026.urdf --frames 100 --output .sew-local/comparison.json
```

Tests cover signed-axis/hand reconstruction, scale/translation invariance, infeasible limits, singular extended arms, geometry rejection, enable alignment, loss/rearm, stalled frames, bounded commands, physical controller release, and host-configuration opt-out, alongside existing Pink regression and blueprint registration checks. Unit fixtures generate their own small robot descriptions and require no vendor download.

A bounded local integration exercise also launched the native CLI, sent generated body/Joy inputs through its actual WebSocket, observed 18-DoF mock motion, verified body/controller loss and release/rearm, then stopped the process. Both SEW and default Pink startup were exercised. This is not a real-headset test; local integration logs and vendor artifacts are not committed.

The 100-frame torso-fixed comparison supplies the same generated physical SEW/hand inputs to both solvers and records proxy/physical bone orientation, hand orientation, actual wrist position, step size, limits, failures and runtime. It compares raw analytic SEW output against bounded streaming Pink, so timing includes different work and cannot establish a speed advantage. One local run had zero failures or limit violations, median times about 0.75/0.72 ms and maximum wrist errors about 25/72 mm (SEW/Pink). Raw SEW's initial step was 0.399 rad versus Pink's 0.0144 rad; actual teleop applies the alignment gate and shared envelope. These are small generated kinematic checks, not safety or dynamics evidence.
