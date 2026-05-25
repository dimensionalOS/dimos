# Application pitch — "100 words on what your robot agent would do"

Submit **one**. Attach the GitHub link (this repo, or the build repo once it exists) + your resume
(the event doubles as an interview).

---

## Option A — Patrol + Reasoning (Agents track)  · ~100 words

Our agent turns the Go2 into an autonomous security patrol that thinks out loud. It walks a set of waypoints,
and at each stop a vision-language model describes the scene and compares it to a learned "normal" baseline
held in spatial memory. When something is off — a door left open, an unfamiliar person, a package that
shouldn't be there — the agent reasons about the anomaly, narrates it aloud, captures the frame, and logs an
incident with location and timestamp. If it gets stuck or loses localization, the LLM diagnoses the failure
and replans. Built entirely as DimOS Skills the agent composes itself.

---

## Option B — Swarm Choreography (Open / Creative track)  · ~100 words

We turn a pack of Go2s into a single choreographed performer. One DimOS Blueprint deploys the same behavior
file across every dog, and a conductor agent issues natural-language cues — "form a line," "wave down the
row," "scatter and regroup" — which each robot interprets relative to its own pose and neighbors. Formations,
synchronized tricks, and call-and-response emerge from shared Skills rather than hand-scripted timelines.
Because the logic is one file fanned out by `autoconnect`, adding a sixth dog is a config change, not a
rewrite. It's a live demonstration that DimOS's multi-robot abstractions actually scale.

---

### Notes
- A reads as "reliable + hireable"; B reads as "ambitious + on-brand." Pick to your appetite.
- Both are honest about being built *on* DimOS primitives — judges (the DimOS team) reward that.
- Trim/expand to land near 100 words if the form enforces a count.
