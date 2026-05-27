"use client";

import Link from "next/link";
import { type FormEvent, useState } from "react";
import { Button } from "@/components/ui/button";
import { Card, CardContent } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { useSession } from "@/lib/auth-client";
import { rpcClient } from "@/lib/orpc";

// Canned commands → the exact text sent to the agent (same as `dimos agent-send`).
const PRESETS: { label: string; text: string }[] = [
  { label: "Scan the room for VR", text: "scan the room for VR" },
  { label: "Explore the room", text: "explore the room and take pictures as you go" },
  { label: "Take a picture", text: "take a picture of what you see" },
];

export function RobotControls() {
  const { data: session, isPending } = useSession();
  const [text, setText] = useState("");
  const [busy, setBusy] = useState<string | null>(null);
  const [status, setStatus] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  if (isPending) {
    return null;
  }

  if (!session) {
    return (
      <Card>
        <CardContent className="flex items-center justify-between py-1">
          <span className="text-muted-foreground text-sm">
            Sign in to control the robot.
          </span>
          <Button asChild size="sm">
            <Link href="/login">Sign in</Link>
          </Button>
        </CardContent>
      </Card>
    );
  }

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
    if (text.trim()) {
      send(text.trim());
    }
  }

  return (
    <Card>
      <CardContent className="flex flex-col gap-4">
        <div className="flex flex-wrap gap-2">
          {PRESETS.map((p) => (
            <Button
              disabled={busy !== null}
              key={p.text}
              onClick={() => send(p.text)}
              variant="secondary"
            >
              {busy === p.text ? "Sending…" : p.label}
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
            {busy === text.trim() ? "Sending…" : "Send"}
          </Button>
        </form>

        {status ? <p className="text-muted-foreground text-sm">{status}</p> : null}
        {error ? <p className="text-destructive text-sm">{error}</p> : null}
      </CardContent>
    </Card>
  );
}
