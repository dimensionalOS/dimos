# Interactive random-object ACT

On this workstation, use a fresh desktop terminal:

```bash
dimos run r1pro-objects-sim-agent
```

Then open another terminal:

```bash
dimos humancli
```

The agent uses your existing `OPENAI_API_KEY` configuration. The blueprint opens
MuJoCo, generates four or five objects, starts Zenoh and stays idle until asked.
No environment activation, LCM/scout address or EGL setting is needed with the
workstation's installed `dimos` launcher. This branch and the local house assets
and trained checkpoint must be installed; those large assets are not in git.

Try one request at a time:

- “What objects are on the table?”
- “Put object_3 in the tray with your right hand.”
- “Put the rightmost remaining object in the tray.”
- “Put the nearest box in the tray.”
- “Stop.”
- “Recover the arm.”
- “Reset the scene.”

IDs remain stable throughout a run. Spatial descriptions use the robot's frame,
not the viewer camera. The agent resolves combined descriptions using measured
positions and shapes; the skill also accepts `nearest`, `furthest`, `rightmost`
and `leftmost` directly. Ambiguous or unavailable objects are not replaced by a
different one. Placement uses a planned empty spot and stops when no object-sized
spot remains. Coordinates and object identities currently come from simulator
ground truth, not visual detection.

The current checkpoint was trained for **right-hand grasps**. Explicit left-hand
requests return `unsupported_arm` before motion. Left-hand transfer is ongoing
work; this blueprint does not claim bimanual ACT picking. The larger household
mesh collection is also deferred. Current objects are boxes, cylinders and
compound bottles with varied poses, dimensions, mass and color.

This blueprint focuses on individual object-to-tray picks. The original
`r1pro-home-sim-agent` remains the separate bottle/tray/navigation demo.

## Direct commands without a language model

```bash
dimos run r1pro-objects-sim
```

From a second terminal:

```bash
dimos mcp call get_scene
dimos mcp call pick_object --json-args '{"object":"object_3","arm":"right"}'
dimos mcp call wait_for_action --arg seconds=20
```

Repeat `wait_for_action` until the state is `completed`, `failed` or `cancelled`.
Acceptance is not completion. The robot executes the grasp, lift, placement and
return using ACT through the ControlCoordinator. It stays idle between requests.

```bash
dimos mcp call stop_action
dimos mcp call wait_for_action --arg seconds=20
dimos mcp call recover_action
dimos mcp call wait_for_action --arg seconds=20
```

A failed ACT pick can automatically release supported contacts and retreat home
using classical SDK planning. Recovery never counts as a successful ACT pick and
never retries the grasp. If an object is airborne or the retreat is blocked, the
robot holds position and reports `recovery_required`. `stop_action` holds without
automatic release or retreat. Explicit `reset_scene` restores the seeded layout
and clears progress; it is never called silently.

Each run writes `scene.xml`, object metadata and `action-001.json`, etc. under
`recordings/r1pro-object-sim/<session>/`. Action files include the selected ID,
measured physical outcome, stop status, recovery and error evidence.

```bash
dimos run r1pro-objects-sim-agent --seed 210001
dimos status
dimos log -f
```

A new seed changes the layout; it does not retrain the policy. The default reuses
`policy-objects-interactive` under `recordings/r1pro-act-task`: the refined
weights with 30 actions executed per prediction. The original checkpoint is preserved.
An alternative checkpoint can be supplied with `--artifact /path/to/policy`.

## Validation and limits

Native MCP testing of the default completed eight requested picks across two
four-object layouts (seeds 210000 and 210006), with different request orders.
These are development layouts, not a fresh generalization test. A separate
desktop GLFW run completed one ACT pick. Deliberate timeouts recovered twice in succession without a reset, and
cancellation followed by explicit reset passed. Unit/physics regressions cover
selection, incorrect arm requests, concurrent commands, supported recovery and
refusing to release an airborne grasp.

The policy remains imperfect. The earlier 20-action setting scored 9/12 single
picks and 1/8 complete scenes. The 30-action setting improved those development
results to 11/12 single picks and 2/8 complete scenes; two native four-object
runs then passed. Other layouts can still fail. These are not arbitrary
household-object or left-arm success rates. The live external-language
validation is pending API-use approval. A recorded-response test passed through
the actual HumanCLI input/response streams and McpClient to a successful physical
ACT pick; it verifies wiring, not language understanding. See the dedicated random-object handoff for current experiment results.
