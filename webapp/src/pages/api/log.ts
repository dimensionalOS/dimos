import type { NextApiRequest, NextApiResponse } from "next";

/**
 * Dev-only sink for speech input. The browser POSTs here so the entry prints
 * server-side — i.e. in the terminal where you run `yarn dev` / `npm run dev`
 * (works even when speaking on the phone over an ngrok tunnel).
 */
export default function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== "POST") {
    res.status(405).end();
    return;
  }
  const body = req.body ?? {};
  if (body.event === "agent-msg") {
    // Every frame the frontend parses, before any speak/dedup decision.
    console.log(
      `[goldie] 📩 agent-msg [kind=${body.kind ?? "?"}]: ${JSON.stringify(body.text ?? "")}`,
    );
  } else if (body.event === "tts") {
    console.log(
      `[goldie] 🔊 tts(${body.spoke ? "SPOKEN" : "muted"}, kind=${body.kind ?? "?"}): ${JSON.stringify(body.text ?? "")}`,
    );
  } else if (body.event === "stt-error") {
    console.log(`[goldie] 🎤⚠️  stt error: ${body.code ?? "unknown"}`);
  } else {
    console.log(`\n[goldie] 🎤 speech: ${JSON.stringify(body.transcript ?? "")}`);
    if (body.payload) console.log(`[goldie]    → submit_query  query=${body.payload}`);
  }
  res.status(204).end();
}
