# Release testing plan

Use this plan to test an existing `release/X.Y.Z` branch. The release workflow is documented in [Releasing](../../docs/development/releasing.md).

## Before testing

With the release branch checked out and a clean working tree, capture the candidate used by the checks:

```bash
export RELEASE_BRANCH="$(git branch --show-current)"
export RELEASE_COMMIT="$(git rev-parse HEAD)"
git log -1 --format='%H %s'
```

- Record the version, branch, full commit SHA and OS alongside the results. Mark each check passed, failed or not run, with a reason for failures and missing prerequisites.
- Copy the values of `RELEASE_BRANCH` and `RELEASE_COMMIT` into terminals on fresh test machines. Installation checks use these values to fetch the candidate installer and verify the resulting checkout.
- Run commands from the repository root. Set up dependencies using the [installation guide](../../docs/installation/index.md) in developer mode.
- Configure the robot connection and credentials required by the selected blueprints. See [configuration](../../docs/usage/configuration.md).
- Run foreground blueprints separately. Use another terminal for commands that interact with them, and stop each blueprint before moving to the next check.

## Checklist

- [ ] [1.1 Tests](01_automated.md#11-tests)
- [ ] [1.2 Type checking](01_automated.md#12-type-checking)
- [ ] [2.1 Replay mode (no hardware)](02_replay-and-simulation.md#21-replay-mode-no-hardware)
- [ ] [2.2 MuJoCo simulation](02_replay-and-simulation.md#22-mujoco-simulation)
- [ ] [2.3 Recording and playback](02_replay-and-simulation.md#23-recording-and-playback)
- [ ] [3.1 Motion planning in Viser](03_manipulation.md#31-motion-planning-in-viser)
- [ ] [3.2 Arms on mock hardware](03_manipulation.md#32-arms-on-mock-hardware)
- [ ] [3.3 xArm pick and place in MuJoCo](03_manipulation.md#33-xarm-pick-and-place-in-mujoco)
- [ ] [3.4 Agentic xArm in MuJoCo](03_manipulation.md#34-agentic-xarm-in-mujoco)
- [ ] [3.5 WebXR headset teleop](03_manipulation.md#35-webxr-headset-teleop)
- [ ] [3.6 Imitation learning data collection](03_manipulation.md#36-imitation-learning-data-collection)
- [ ] [3.7 R1 Pro planar-base planning](03_manipulation.md#37-r1-pro-planar-base-planning)
- [ ] [4.1 Cockpit](04_web.md#41-cockpit)
- [ ] [4.2 WebRTC video](04_web.md#42-webrtc-video)
- [ ] [4.3 Hosted relay with TLS and auth](04_web.md#43-hosted-relay-with-tls-and-auth)
- [ ] [4.4 Hosted teleop](04_web.md#44-hosted-teleop)
- [ ] [5.1 Daemon lifecycle](05_cli-and-api.md#51-daemon-lifecycle)
- [ ] [5.2 MCP CLI](05_cli-and-api.md#52-mcp-cli)
- [ ] [5.3 Python API](05_cli-and-api.md#53-python-api)
- [ ] [5.4 dimos shell](05_cli-and-api.md#54-dimos-shell)
- [ ] [5.5 Module configuration](05_cli-and-api.md#55-module-configuration)
- [ ] [5.6 Transports and topics](05_cli-and-api.md#56-transports-and-topics)
- [ ] [5.7 dtop](05_cli-and-api.md#57-dtop)
- [ ] [5.8 Rerun viewer modes](05_cli-and-api.md#58-rerun-viewer-modes)
- [ ] [5.9 Blueprint graph and docs site](05_cli-and-api.md#59-blueprint-graph-and-docs-site)
- [ ] [5.10 Host daemon](05_cli-and-api.md#510-host-daemon)
- [ ] [5.11 Native module examples](05_cli-and-api.md#511-native-module-examples)
- [ ] [5.12 External blueprint packages](05_cli-and-api.md#512-external-blueprint-packages)
- [ ] [5.13 Cold-cache arm startup](05_cli-and-api.md#513-cold-cache-arm-startup)
- [ ] [5.14 Cloud sign-in and data](05_cli-and-api.md#514-cloud-sign-in-and-data)
- [ ] [6.1 New Ubuntu install](06_installation.md#61-new-ubuntu-install)
- [ ] [6.2 New macOS install](06_installation.md#62-new-macos-install)
- [ ] [6.3 Native modules built with Nix](06_installation.md#63-native-modules-built-with-nix)
- [ ] [7.1 Local LLM backends and request tracing](07_agents.md#71-local-llm-backends-and-request-tracing)
- [ ] [7.2 Tool streams and capability conflicts](07_agents.md#72-tool-streams-and-capability-conflicts)
- [ ] [7.3 Point navigation skill](07_agents.md#73-point-navigation-skill)
- [ ] [7.4 TypeSafe navigation agent](07_agents.md#74-typesafe-navigation-agent)
- [ ] [7.5 Perception localize](07_agents.md#75-perception-localize)
- [ ] [7.6 Camera calibration and AprilTag markers](07_agents.md#76-camera-calibration-and-apriltag-markers)
- [ ] [7.7 Agent evals](07_agents.md#77-agent-evals)
- [ ] [7.8 VQA dataset generation](07_agents.md#78-vqa-dataset-generation)
- [ ] [7.9 xArm6 WorldBelief](07_agents.md#79-xarm6-worldbelief)
- [ ] [8.1 Go2 3D navigation in MuJoCo](08_navigation.md#81-go2-3d-navigation-in-mujoco)
- [ ] [8.2 Go2 premap and relocalization](08_navigation.md#82-go2-premap-and-relocalization)
- [ ] [8.3 Mid-360 relocalization in replay](08_navigation.md#83-mid-360-relocalization-in-replay)
- [ ] [8.4 Mid-360 driver and Point-LIO](08_navigation.md#84-mid-360-driver-and-point-lio)
- [ ] [8.5 Habitat simulation](08_navigation.md#85-habitat-simulation)
- [ ] [9.1 Test launching multiple blueprints](09_hardware.md#91-test-launching-multiple-blueprints)
- [ ] [9.2 Commanding the robot from rerun](09_hardware.md#92-commanding-the-robot-from-rerun)
- [ ] [9.3 demo-camera](09_hardware.md#93-demo-camera)
- [ ] [9.4 Go2 connection with an AES key](09_hardware.md#94-go2-connection-with-an-aes-key)
- [ ] [9.5 Go2 rage mode](09_hardware.md#95-go2-rage-mode)
- [ ] [9.6 Go2 drive and record](09_hardware.md#96-go2-drive-and-record)
- [ ] [9.7 RealSense camera](09_hardware.md#97-realsense-camera)
- [ ] [10.1 G1 agent in simulation](10_robots.md#101-g1-agent-in-simulation)
- [ ] [10.2 G1 whole-body control in MuJoCo](10_robots.md#102-g1-whole-body-control-in-mujoco)
- [ ] [10.3 G1 navigation and arm teleop on hardware](10_robots.md#103-g1-navigation-and-arm-teleop-on-hardware)
- [ ] [10.4 FlowBase holonomic base](10_robots.md#104-flowbase-holonomic-base)

## Coverage gaps

These have no complete manual procedure and are not pass/fail checkboxes:

- **Imitation learning policy rollout:** `LeRobotPolicyModule` can run a trained policy, but there is no shipped blueprint and checkpoint pair to exercise. Add a reproducible rollout procedure when those are available.
- **Alfred wheeled robot:** the `alfred-keyboard-teleop`, `alfred-mls-nav` and `alfred-mls-nav-lidar` blueprints need the in-house Alfred robot and a documented end-to-end procedure.
- **xArm6 WorldBelief on hardware:** [7.9](07_agents.md#79-xarm6-worldbelief) checks a recorded session. The live blueprint's broken TF tree remains a hardware coverage gap.
