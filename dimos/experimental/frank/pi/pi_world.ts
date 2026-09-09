// Pi extension: append FRANK's world state as an ephemeral last message.
//
// loop.py writes cache/world.txt before every turn. Pi's `context` event fires before each LLM
// call with a copy of the messages; whatever we return is sent to the model but never written to
// the session. So the model always sees a fresh snapshot at the end of the array, the history
// never accumulates stale copies, and the cached prefix (system prompt + earlier turns) is untouched.
//
// Loaded by loop.py with `pi --extension dimos/experimental/frank/pi/pi_world.ts`.

import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const WORLD_FILE = join(dirname(fileURLToPath(import.meta.url)), "..", "cache", "world.txt");

export default function (pi: ExtensionAPI) {
  pi.on("context", async (event) => {
    let world: string;
    try {
      world = readFileSync(WORLD_FILE, "utf8").trim();
    } catch {
      return; // no snapshot written: send the messages unchanged
    }
    if (!world) return;
    const messages = [
      ...event.messages,
      {
        role: "user" as const,
        content: [{ type: "text" as const, text: `[ephemeral, not in your history]\n${world}` }],
        timestamp: Date.now(),
      },
    ];
    return { messages };
  });
}
