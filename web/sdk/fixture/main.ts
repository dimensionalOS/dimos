// Minimal @dimos/sdk consumer, used as the W1 demo: lists the relay's
// robots, rides the lone-robot auto-watch, and renders odom through a manual
// subscription - no cockpit panel involved. Run a local relay first
// (`dimos run <blueprint> --local-relay`), then `deno task fixture` in
// web/sdk and open http://localhost:5174/.

import { connect } from "@dimos/sdk";

const robotsEl = document.querySelector<HTMLElement>("#robots")!;
const watchedEl = document.querySelector<HTMLElement>("#watched")!;
const odomEl = document.querySelector<HTMLElement>("#odom")!;

const session = connect(); // same origin: /api/info rides the vite proxy

session.status.subscribe(() => {
  const status = session.status.get();
  const robots = status.robots.map((r) => `${r.name} (${r.id})`).join(", ");
  robotsEl.textContent = `robots: ${robots || "(none)"} [${status.transport.phase}]`;
  watchedEl.textContent = `watched: ${status.watchedRobot?.id ?? "(none)"}`;
});

session.subscribe("odom", (snapshot) => {
  odomEl.textContent = `odom: ${JSON.stringify(snapshot.slot?.value ?? null, null, 2)}`;
});
