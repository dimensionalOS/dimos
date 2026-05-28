"use client";

import { Loader2, Send } from "lucide-react";
import { type FormEvent, useState } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { rpcClient } from "@/lib/orpc";

// Canned commands → the exact text sent to the agent (same as `dimos agent-send`).
const PRESETS: { label: string; text: string }[] = [
  { label: "Scan the room for VR", text: "scan the room for VR" },
  {
    label: "Explore the room",
    text: "explore the room and take pictures as you go",
  },
  { label: "Take a picture", text: "take a picture of what you see" },
];

export function RobotControls() {
  const [text, setText] = useState("");
  const [busy, setBusy] = useState<string | null>(null);
  const [status, setStatus] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  async function send(cmd: string) {
    setBusy(cmd);
    setError(null);
    setStatus(null);
    try {
      await rpcClient.commands.send({ text: cmd });
      setStatus(`Sent: “${cmd}”`);
      setText("");
    } catch (err) {
      setError(err instanceof Error ? err.message : "failed to send");
    } finally {
      setBusy(null);
    }
  }

  function onSubmit(e: FormEvent) {
    e.preventDefault();
    if (text.trim()) send(text.trim());
  }

  return (
    <div className="flex flex-col gap-3">
      <span className="font-mono text-muted-foreground text-xs uppercase tracking-wider">
        Send command
      </span>
      <div className="flex flex-wrap gap-2">
        {PRESETS.map((p) => (
          <Button
            disabled={busy !== null}
            key={p.text}
            onClick={() => send(p.text)}
            size="sm"
            variant="secondary"
          >
            {busy === p.text ? (
              <Loader2 className="animate-spin" size={14} />
            ) : null}
            {p.label}
          </Button>
        ))}
      </div>

      <form className="flex gap-2" onSubmit={onSubmit}>
        <Input
          onChange={(e) => setText(e.target.value)}
          placeholder="Or type a command for the robot…"
          value={text}
        />
        <Button disabled={busy !== null || !text.trim()} type="submit">
          {busy === text.trim() ? (
            <Loader2 className="animate-spin" size={16} />
          ) : (
            <Send size={16} />
          )}
          Send
        </Button>
      </form>

      {status ? (
        <p className="text-signal text-sm">{status}</p>
      ) : null}
      {error ? <p className="text-destructive text-sm">{error}</p> : null}
    </div>
  );
}
