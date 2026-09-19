# Local operator view

The operator view uses the existing Cockpit session, manifests, subscriptions, video/map renderers
and teleop lease state machine. Open `http://127.0.0.1:7780/?view=operator` after launching the
relay.

From the worktree root, launch recorded Go2 data:

```sh
uv run dimos --replay --viewer none run unitree-go2-cockpit --local-relay
```

For live hardware, substitute the reachable robot IP:

```sh
uv run dimos --no-replay --no-simulation --robot-ip 192.168.123.161 --viewer none run unitree-go2-cockpit --local-relay
```

The relay builds the frontend automatically. For frontend-only iteration, run `deno task dev` here
and use the Vite URL with `?view=operator` while the relay is running. `deno task check`,
`deno task test`, and `deno task build` validate the UI.

Camera/map switching changes layout without remounting either renderer. The first advertised video,
map2d and teleop panels are used. Missing panels show explicit placeholders; the authored Go2
cockpit supplies all three.

Focus the keyboard pad to acquire control. WASD drives, QE strafes, Shift uses the blueprint's
boost, Space or the Stop button clears motion while retaining control. Stop is enabled only while
this viewer holds the lease. Escape, focus loss, hidden tab and disconnect disarm. Stop is not a
hardware E-stop latch. The generic blueprint layout is available at `/`.

The upstream header navigation is available in operator mode: Overview returns to the operator
dashboard, Channels opens the channel table, and Stats opens the dtop resource monitor. The Go2
cockpit enables resource monitoring through its Stats page; `--no-dtop` disables collection. The
robot picker appears when several robots share the relay. The operator dashboard also retains its
inline channel table below the controls.

## Go2 controls

The Go2 cockpit blueprint advertises typed operator command, state and result channels. The operator
view provides battery state of charge, headlight brightness (0–100% in 10% steps), Stand / Drive,
Sit / Lie down, Wave / Shake hand, Stretch, and Recover Stand. The wave button calls the firmware's
Hello action.

Actions use the SDK's acknowledged shared publish API. Delivery means the command reached the DimOS
stream; a separate, request-ID-matched result reports robot API acceptance or rejection. Neither is
physical motion-completion feedback. Commands with missing results or a lost connection are never
retried automatically. Last accepted light requests and actions are labelled as requests, not
measured brightness or posture. Battery is unavailable when replay supplies no lowstate.

The speed selector scales the blueprint's linear/angular limits to 25%, 50%, or 100%; Shift applies
the existing boost on top. Changing speed releases control and requires focusing the drive pad
again. It does not switch robot firmware locomotion modes.
