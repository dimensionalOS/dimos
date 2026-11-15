# Run a compatible checkpoint

Start with a local pretrained-model directory and the single-arm, single-camera
OpenYAM setup expected by the policy.

Stop the collection stack before launching rollout. Replace `CHECKPOINT_DIR`
with a compatible pretrained-model directory, such as
`outputs/openyam-act/checkpoints/last/pretrained_model`.

```bash skip
dimos run openyam-lerobot-rollout --daemon \
  --controlcoordinator.can-port follower_l \
  --policy.policy-path CHECKPOINT_DIR \
  --policy.task "pick up the red block" \
  --policy.device cuda \
  --wristcamera.hardware.camera-index 0
dimos imitation control
```

Use `openyam-lerobot-quest-rollout` for the graph with Quest takeover.
The shared control panel discovers `RolloutControlSpec` and shows policy
state and errors. Select **Start policy** to request execution. Before
starting, preflight loads the checkpoint and checks the control task and fresh
observations without sending a trajectory. A failed check leaves the policy
stopped and reports the error.

Q only detaches, even while the policy is active. Run `dimos imitation control`
again to reattach. Select **Stop policy** to stop execution while keeping the stack running.
To finish, select **Stop stack…**, support the arm, and confirm shutdown.
The panel is not a deadman switch: lost connectivity does not guarantee stopping
motion. Use `dimos stop` if the panel is unavailable.

See the [LeRobot module contract](/dimos/imitation/policy/lerobot/README.md)
for checkpoint and control requirements. Rollout uses the existing single-arm,
single-camera contract with absolute joint targets in the hardware's native
coordinates. A prepared dataset does not establish checkpoint compatibility
with another robot. ABC integration, dual-arm policy rollout, and policy
backend generalization are deferred.
