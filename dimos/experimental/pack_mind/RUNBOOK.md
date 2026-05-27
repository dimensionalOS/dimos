# PACK MIND — Live Go2 Runbook

> Run this top-to-bottom at the venue. Every command is copy-paste. Do **not**
> debug architecture on-site — that work is done; this is wiring + checks.

**Pitch:** Most teams make robot dogs share *maps*. Pack Mind makes them share
*meaning*. Alpha sees → the pack remembers → Bravo acts on a teammate's memory,
by zone **name**, never coordinates.

---

## 0. Topology (read once)

```
[Go2 alpha] <--robot-ip-- [GPU box A]  dimos agentic stack  --MCP 0.0.0.0:9990-->  \
[Go2 bravo] <--robot-ip-- [GPU box B]  dimos agentic stack  --MCP 0.0.0.0:9990-->   >  [operator laptop]
                                                                                    /    conductor.py
                                                                                         dashboard :8080
```

- Each dog needs a **CUDA** companion node (EdgeTAM perception). The agentic
  stack runs on the node, not the dog. The dog is reached via `--robot-ip`.
- One GPU box per dog → both use MCP port `9990`.
- One GPU box running **two** stacks → give each a distinct `--mcp-port`
  (`9990`, `9991`).
- The conductor runs on the **operator laptop** and talks to each node's MCP
  over the LAN.

### ⚠️ #1 KILLER — bind MCP to the LAN

DimOS defaults `listen_host = 127.0.0.1`. The MCP server then only answers
localhost and the conductor gets **connection refused** over the LAN. Every
agentic stack MUST be launched with:

```
--listen-host 0.0.0.0
```

If a G0 curl is refused, this is the cause 90% of the time. The other 10% is a
firewall on the GPU box.

---

## 1. Per-dog stack bring-up (on each GPU node)

```bash
# Node A → drives Go2 "alpha"
dimos run unitree-go2-agentic \
  --robot-ip <ALPHA_DOG_IP> \
  --listen-host 0.0.0.0 \
  --mcp-port 9990 \
  --daemon

# Node B → drives Go2 "bravo"   (or same box, --mcp-port 9991)
dimos run unitree-go2-agentic \
  --robot-ip <BRAVO_DOG_IP> \
  --listen-host 0.0.0.0 \
  --mcp-port 9990 \
  --daemon
```

Verify each node locally before involving the network:

```bash
dimos status            # run id, pid, uptime
dimos mcp status        # PID, module list, skill list
dimos mcp list-tools    # must include: speak, navigate_with_text, look_out_for, tag_location
```

---

## 2. G0 (h4) — remote MCP reachability over LAN

From the **operator laptop**, for every node `<NODE_IP>:<PORT>`:

```bash
curl -s -X POST http://<NODE_IP>:<PORT>/mcp \
  -H 'content-type: application/json' \
  -d '{"jsonrpc":"2.0","id":"1","method":"tools/list"}' | jq .
```

**PASS** = JSON with a `result.tools` array containing `speak`,
`navigate_with_text`, `look_out_for`. **G0 done.**

> The MCP wire format itself is already validated CPU-side
> (`sim_harness.py`, multica WEB-5). At the venue you are only proving the
> *network path*, not the protocol.

If refused: re-check `--listen-host 0.0.0.0`, then `ping <NODE_IP>`, then the
node's firewall (`sudo ufw allow 9990`).

---

## 3. Launch the conductor (operator laptop)

```bash
cd <dimos repo>
source .venv/bin/activate
python dimos/experimental/pack_mind/conductor.py \
  --dog alpha=<NODE_A_IP>:9990 \
  --dog bravo=<NODE_B_IP>:9990
# add --dog charlie=<NODE_C_IP>:9990 for the 3rd dog (gravy only)
```

Open **http://localhost:8080**. Six operator buttons drive the mission. Put this
on the projector — the causal event feed *is* the story.

---

## 4. G1 (h12) — single-dog magic moment

One dog, one human-legible beat. On the dashboard:

1. **start_act1** → Alpha announces it is scouting.
2. **inject_found** (alpha / red backpack / zone_b) → Alpha speaks "Found red
   backpack in zone_b" and the find lands on the shared blackboard.
3. **ask_where** (bravo / red backpack) → Bravo answers **from pack memory**:
   *"Alpha found it in zone_b. Follow me."*

If the dog physically speaks and the dashboard shows the causal chain, **G1 done.**

---

## 5. G2 (h24) — two-dog MVP, **twice in a row** (THE gate)

Full mission, both dogs, no crash, two consecutive clean runs:

1. **start_act1**
2. **inject_found** alpha → zone_b
3. **ask_where** bravo  → answers from shared memory
4. **send_dog** bravo   → acquires movement lock, navigates to **zone_b by name**
5. **verify** bravo     → VLM lookout confirms → "Pack memory was correct."

Watch for: only one dog moving at a time (movement lock), no coordinates ever
exchanged. Run it, **estop**, reset, run it again. Two clean passes = **G2 done.**
This is the gate the judges buy. Everything before it is low-risk; the real risk
is multi-dog concurrency + venue WiFi.

---

## 6. Fallback — the demo never hard-fails

**Operator `inject_found` produces the identical blackboard effect as a real
detection.** If live perception is flaky, the operator injects the find and the
entire downstream story (memory → query → navigate → verify) runs unchanged.
Rehearse the demo driving inject_found by hand; a real detection is a bonus, not
a dependency.

If a dog wanders or a nav call hangs: hit **estop** (releases the movement lock,
all dogs hold, mission → IDLE), then resume from start_act1.

---

## 7. G3 (h36) / G4 (h48)

- **G3:** Record a clean two-dog run end-to-end (screen + floor). Harden: rerun
  G2 ~5× and note any flake.
- **G4:** Rehearse the 90s presentation 3×. Script the narration to the
  dashboard event feed. Have the recorded G3 run ready as the fallback if the
  live floor misbehaves.

---

## 8. Quick reference

| Thing | Command / value |
|---|---|
| MCP port (default) | `9990` (`--mcp-port`) |
| LAN bind (REQUIRED) | `--listen-host 0.0.0.0` |
| Dashboard | `http://localhost:8080` |
| Stop a stack | `dimos stop` (on the node) |
| Tail logs | `dimos log -f` (on the node) |
| Send raw agent text | `dimos agent-send "..."` |
| CPU dry-run (no dogs) | `conductor.py --mock` |
| MCP tool args | `speak(text, blocking)` · `navigate_with_text(query)` · `look_out_for(description_of_things)` · `tag_location(location_name)` |

Zones are named (`zone_a/b/c`). Cross-dog handoff is **always** by zone name.
There is no coordinate field anywhere in the shared memory — by design.
