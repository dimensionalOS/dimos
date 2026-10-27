# Hybrid Teleop: proposed PRs and Linear drafts

From the integration branch `mustafa/integration/hybrid-teleop-alfred` (FINDINGS.md has the issues; the letters
there are the PRs here). Every PR is under 10 files. Order is merge order.

## Demo status (Oct 27)

| Step | Sim (alfred-manipulation-sim) | Alfred (mock arms, real lift + base) |
|---|---|---|
| 1 Pick on its own | Works with the grasp check off (sim has no physics) and the base driven 25 cm in by hand | Not run: arms mock (right wrist dead) |
| 2 Stuck → hold | Failed Pick → hold (failsafe); agent `request_operator` over MCP → hold (agent); all 20 joints frozen | Agent request → hold; base frozen (0.3 mm vs 7.8 cm control); lift jog frozen, **then the lift faulted** (FINDINGS 27) |
| 3 Acknowledge, resolve | Acknowledge, Take over + teleop work; re-target + Pick run and log, but the re-picks failed in the planner | Acknowledge, Take over + teleop 3.9 cm work; nothing moved after Acknowledge |
| 4 Restart autonomy | Works; refused while held or e-stopped | Works |
| 5 Event log | 27 events per run, all fields, `operator_events.jsonl` | Same |
| 6 E-STOP | Stops arm, lift, base; 0.4 s late over RPC, one tick over the bus | Stops the base (bus +1 ms, RPC +326 ms); lift E-STOP untested (faulted) |

The demo stops on Alfred at step 1 (no real arms) and the lift cannot be held without faulting (FINDINGS 27). The LLM
agent itself never ran (no `OPENAI_API_KEY` on the Orin); `request_operator` was called over MCP as an agent would.

## dimos, in merge order

| # | PR | Files (≈) | One line |
|---|---|---|---|
| B | `fix(control): the operator hold runs on any coordinator` | 8 | Rework of operator-ui-hold without the page: `OperatorHoldCoordinator` in `dimos/control/`, the hold task works on a plain `ControlCoordinator` again, preemption warnings once per episode, the gripper drops its target when held. |
| C | `feat(control): e-stop and operator status over RPC` | 5 | Backend of operator-ui-estop: the `operator_estop` port, `estop(bool)` (latch + hold + cancel) and `get_operator_status()` in plain JSON for Desktop apps. |
| D | `feat(control): OperatorHandoff` | 4 | The agent's `request_operator(reason)` skill (route agent), the operator event log (`operator_events.jsonl`), `restart_autonomy`, and the `operator_status` JSON topic; acknowledge stays operator-only. |
| F | `fix(alfred): the lift under the operator hold` | 5 | No zero-length lift moves from the hold, a serial parser that survives a STATUS line inside telemetry, `home` refused while held, and `handle_motor_command` not logging as a skill on every tick. |
| A | `fix(alfred): the Alfred stacks on main` | 9 | After mb/feat/alfred-full-stack lands: Alfred perception on main's self filter (#4612), `alfred_urdf_manager` on main's `UrdfManager`, the pick flow with main's contact check, the leaked test thread. |
| E | `feat(alfred): the bench stacks can be held` | 6 | alfred-manipulation(-sim) run the hold, `OperatorHandoff` and an MCP server (agent when a key is set), Alfred's prompt learns `request_operator`, the sim skips grasp verification. |
| I | `fix(core): investigate the list_streams stall` | — | Investigation first, not a PR yet: 3 min gateway stall, the pick module's frozen tf after it, gateway module restart failing with EBADF, `dimos stop` needing SIGKILL, log rotation races between workers. |

Not for main: `web/hybrid-teleop` (dropped) and `OperatorCommandsModule` from operator-ui-target (duplicates the
app, acknowledges holds by itself).

## Desktop gateway (dimos-desktop)

| # | PR | One line |
|---|---|---|
| G | `perf(gateway): RPC calls without listing every module and stream` | `Live.call` lists all modules and streams before each call (0.2-0.3 s); cache the module lookup so a button press is one RPC. |

## dim-manipulation (branch `hybrid-teleop`, 4 commits so far)

| # | PR | Files | One line |
|---|---|---|---|
| H1 | Operator card | 3 | Hold banner (route, reason, since), Request operator, Acknowledge, Take over, Restart autonomy, E-STOP, the operator log with Save, motion buttons locked while held or e-stopped. |
| H2 | A failed Pick holds the robot | 2 | A Pick that doesn't end holding the object requests the hold (route failsafe, the pick's message as reason). |
| H3 | E-STOP over zenoh too | 2 | E-STOP also puts a `std_msgs.Bool` on `operator_estop` through the zenoh-gateway: one tick instead of 0.4 s, and it works when the dimos gateway stalls. |

## Linear drafts

### P-END-177 Operator hold + handoff

Integrated main + the Alfred stack + the operator-ui backend on `mustafa/integration/hybrid-teleop-alfred` and
validated in sim and on Alfred (mock arms, real lift and base).

- Works: hold from an operator, the agent (`request_operator`, route agent, over MCP) and a failed pick (failsafe);
  every joint frozen (sim: 20 of 20; Alfred: base 0.3 mm against 7.8 cm without the hold); Acknowledge is
  operator-only and nothing moves afterwards; Restart autonomy is refused while held or e-stopped; every action is
  logged with kind, action, time, object id and hold reason.
- E-STOP stops arm, lift and base. Over the dimos gateway it lands ~0.4 s late; the app now also sends it over
  zenoh (one tick).
- Blockers: holding the lift faults it on hardware (zero-length moves + a lost `move_done`); the lift's `home`
  skill gets around the hold; a 3 min gateway stall left the pick module with a stale tf.
- Next: PRs B, C, D, F, then A, E after the Alfred stack lands; an investigation ticket for the stall.

### P-END-178 Operator assist UI

The Manipulation app has an Operator card on branch `hybrid-teleop`: hold banner with route, reason and time,
Request operator, Acknowledge, Take over, Restart autonomy, E-STOP (RPC + zenoh), the operator log with Save, and
motion buttons locked during a hold. A failed Pick holds the robot for the operator.

- Verified through the app's own calls in sim and on Alfred; a person has clicked Scan, a target and Pick (the
  failed pick turned the hold on through the app). The rest of the card still needs a click-through.
- Depends on P-END-177's `get_operator_status`, `estop`, `OperatorHandoff`.
- Next: click-through on Desktop, then PRs H1, H2, H3.
