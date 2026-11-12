# Robot Safety

Read this before you touch any robot. These rules apply to every robot. Then read the page for your robot:

- [Galaxea R1 Pro](/docs/platforms/safety/r1pro.md)

## 1. The rules

1. Never work alone on hardware. One person runs code, one person stands at the E-stop.
2. Nothing of yours goes inside the robot's reach while it is powered on.
3. New code runs in sim first. First run on hardware: slow speed, hand on the E-stop.
4. Software stop is not a safety stop. If anything looks wrong, hit the E-stop first, debug later.
5. Never leave a powered robot unattended. Walking away = E-stop pressed or power off.
6. Don't change firmware, joint limits, speed limits or safety settings. Ask your Dimensional engineer.
7. Anything breaks, sparks, smells or hurts someone: power off, then tell us (see [Contacts](#8-contacts)).

## 2. Your workspace

- Keep a clear zone around the robot: its full reach + 30 cm. Nobody and nothing inside while powered.
- Fixed arms stay clamped to the table. Don't move them.
- Keep cables off the walkway and away from the joints.
- No liquids, loose sleeves, lanyards or loose long hair near the robot.
- Objects you pick up: under the robot's payload limit; nothing sharp, glass or heavy unless approved.

## 3. Three ways to stop (fastest first)

| What | How | What it does |
|---|---|---|
| Hardware E-stop | Hit the red button | Stops all motion. Works even if the software hangs. |
| Teleop release | Let go of the grip (Quest) / E-STOP button (browser) | Robot stops following you. |
| Software stop | Ctrl-C or `dimos stop` | Shuts dimos down. Can take seconds, can fail. |

## 4. Power on

1. Area clear, nobody in the zone.
2. E-stop released (twist it up). Some robots boot into a fault if it is pressed.
3. Power the robot on and wait until it is ready (see your robot's page).
4. Start dimos with the blueprint listed on your robot's page.
5. First move: small, slow, watched.

## 5. Power off

1. Move the robot to its rest pose.
2. Stop dimos: `dimos stop`.
3. Hold the arms, or make sure nothing is under them: some arms drop when power is cut.
4. Power off.

## 6. After an E-stop or fault

- Find out why it happened before you release the E-stop.
- Follow the recovery steps on your robot's page. Some robots need a full power cycle.
- Tell your engineer if your code caused it.

## 7. AI agents, policies and autonomous code

- Motion driven by an LLM agent (skills, MCP): someone at the E-stop the whole time.
- Trained policies: first runs at low speed, inside a small workspace.

## 8. Contacts

Your Dimensional engineer is your first contact for anything about the hardware. In an emergency, call **911** first.
