# Agent Capability Demo Instructions

This demo runs a simulated warehouse inspection robot. It has no hardware
dependencies. All work is simulated with sleeps and tool-stream updates.

## Start

Terminal 1:

```bash
uv run dimos run demo-capabilities
```

Terminal 2:

```bash
uv run humancli
```

## Manual Prompts

Type these in `humancli`.

### Sequential vs. Parallel Instant Tools

```text
Read the battery, read the temperature, and capture a photo at the same time.
```

Expected: the three timestamp windows should overlap substantially.

### Background Tool Streams Without Conflict

```text
Start patrolling and start an environment scan.
```

Expected: `start_patrol` starts and streams `visiting waypoint N` updates.
`start_environment_scan` also starts and streams air readings. These should run
together because environment scanning has no capability.

### Capability Conflict

```text
Without stopping patrol first, try to turn in place 90 degrees. If there is a conflict, report the exact error and do not recover.
```

Expected: `turn_in_place` is refused because `movement` is held by
`start_patrol`. The response should include text like:

```text
Cannot start 'turn_in_place': capability 'movement' is held by 'start_patrol'.
```

### Release and Retry

```text
Stop patrol, then turn in place 90 degrees.
```

Expected: `stop_patrol` closes the patrol tool stream, releasing `movement`.
`turn_in_place` then succeeds with a 2-second timestamp window.

### Self-Terminating Background Tool

```text
Do a lap and start an environment scan.
```

Expected: `do_a_lap` streams four checkpoint updates over about 8 seconds and
then stops by itself, releasing `movement`. The environment scan keeps running
until stopped.

Stop the scan when done:

```text
Stop the environment scan.
```

### Different Capability

```text
Weigh the sample box and secure it at the same time.
```

Expected: one of the two payload tools may be refused while the other holds `payload`.

### Competing Background Movement Tools

```text
Start patrol and do a lap at the same time.
```

Expected: one movement tool starts. The other should be refused because
`movement` is already held by the first movement tool.
