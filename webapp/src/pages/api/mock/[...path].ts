import type { NextApiRequest, NextApiResponse } from "next";

// SSE must not be size-limited or buffered.
export const config = {
  api: { bodyParser: { sizeLimit: "1mb" }, responseLimit: false },
};

type Run = { startedAt: number } | null;

// Module-level state persists across requests within the single `next dev`
// process — enough to simulate "after a query, the agent replies for a bit".
let run: Run = null;

// Scripted plain-text agent_responses (mirrors the real backend's feed).
const SCRIPT: { at: number; text: string }[] = [
  { at: 300, text: "On it." },
  { at: 1400, text: "Navigation goal reached" },
  { at: 2400, text: "Spoke: Done — what would you like next?" },
];

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse,
) {
  const path = (Array.isArray(req.query.path) ? req.query.path : [req.query.path])
    .filter(Boolean)
    .join("/");

  // --- SSE: agent_responses stream (matches the real backend key) ---
  if (path === "text_stream/agent_responses" && req.method === "GET") {
    res.writeHead(200, {
      "Content-Type": "text/event-stream",
      "Cache-Control": "no-store, no-transform",
      Connection: "keep-alive",
    });
    let closed = false;
    req.on("close", () => {
      closed = true;
    });
    let seenStart = 0;
    let idx = 0;
    while (!closed && !res.writableEnded) {
      if (run) {
        if (run.startedAt !== seenStart) {
          seenStart = run.startedAt;
          idx = 0;
        }
        const elapsed = Date.now() - run.startedAt;
        while (idx < SCRIPT.length && SCRIPT[idx].at <= elapsed) {
          res.write(`data: ${SCRIPT[idx].text}\n\n`);
          idx++;
        }
        if (idx >= SCRIPT.length) run = null;
      } else {
        res.write(`event: ping\ndata: \n\n`);
      }
      await sleep(400);
    }
    res.end();
    return;
  }

  if (path === "text_streams" && req.method === "GET") {
    return res.status(200).json({ streams: ["agent_responses"] });
  }

  if (path === "unitree/status" && req.method === "GET") {
    return res.status(200).json({ status: "online", service: "unitree" });
  }

  if (path === "submit_query" && req.method === "POST") {
    run = { startedAt: Date.now() };
    return res.status(200).json({ ok: true });
  }

  if (path === "upload_audio" && req.method === "POST") {
    return res.status(200).json({ text: "stand up please" });
  }

  if (path === "unitree/command" && req.method === "POST") {
    return res
      .status(200)
      .json({ ok: true, command: req.body?.command ?? null });
  }

  if (path === "unitree/move" && req.method === "POST") {
    return res.status(200).json({ ok: true });
  }

  if (path === "interrupt" && req.method === "POST") {
    run = null;
    return res.status(200).json({ ok: true });
  }

  return res
    .status(404)
    .json({ error: `mock: no route for ${req.method} /${path}` });
}
