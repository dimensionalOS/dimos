"use client";

import type { RobotSettings } from "@robomoo/shared";
import { Loader2, RotateCcw } from "lucide-react";
import { useEffect, useState } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { rpcClient } from "@/lib/orpc";
import { cn } from "@/lib/utils";

// The "advanced" robot-connection panel: shows the effective dimos agent
// endpoint + connection status, and lets anyone (demo mode) point the server at
// a different robot URL at runtime. The override is stored server-side (DB) and
// wins over the DIMOS_AGENT_URL env; the bearer token stays on the server.
export function RobotConnection() {
  const [settings, setSettings] = useState<RobotSettings | null>(null);
  const [url, setUrl] = useState("");
  const [busy, setBusy] = useState(false);
  const [msg, setMsg] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    rpcClient.settings
      .get()
      .then((s) => {
        setSettings(s);
        setUrl(s.url ?? "");
      })
      .catch((e) =>
        setError(e instanceof Error ? e.message : "failed to load settings"),
      );
  }, []);

  const save = async (next: string | null) => {
    setBusy(true);
    setError(null);
    setMsg(null);
    try {
      const s = await rpcClient.settings.setRobotUrl({ url: next });
      setSettings(s);
      setUrl(s.url ?? "");
      setMsg(next ? "Robot URL saved." : "Reset to the server default.");
    } catch (e) {
      setError(e instanceof Error ? e.message : "failed to save");
    } finally {
      setBusy(false);
    }
  };

  const connected = Boolean(settings?.effectiveUrl && settings?.tokenConfigured);
  const urlOnly = Boolean(settings?.effectiveUrl) && !settings?.tokenConfigured;

  const pill = connected
    ? { cls: "bg-signal/15 text-signal", dot: true, label: "Connected" }
    : urlOnly
      ? { cls: "bg-working/15 text-working", dot: false, label: "URL set · no token" }
      : { cls: "bg-muted text-muted-foreground", dot: false, label: "Not configured" };

  const dirty = (url.trim() || null) !== (settings?.url ?? null);

  return (
    <div className="flex flex-col gap-3">
      <div className="flex items-center justify-between gap-2">
        <span className="font-mono text-muted-foreground text-xs uppercase tracking-wider">
          Robot connection
        </span>
        <span
          className={cn(
            "inline-flex items-center gap-1.5 rounded-full px-2 py-0.5 font-medium text-[10px] uppercase tracking-wide",
            pill.cls,
          )}
        >
          {pill.dot ? (
            <span className="size-1.5 rounded-full bg-current pulse-dot" />
          ) : null}
          {pill.label}
        </span>
      </div>

      <div className="flex flex-col gap-1 text-sm">
        <span className="text-muted-foreground text-xs">Effective endpoint</span>
        <span className="break-all font-mono text-xs">
          {settings?.effectiveUrl ? (
            <>
              {settings.effectiveUrl}
              <span className="ml-2 text-muted-foreground">
                ({settings.source})
              </span>
            </>
          ) : (
            <span className="text-muted-foreground">none</span>
          )}
        </span>
      </div>

      <div className="flex flex-col gap-1.5">
        <span className="text-muted-foreground text-xs">
          Override robot URL (point at your own robot)
        </span>
        <div className="flex gap-2">
          <Input
            onChange={(e) => setUrl(e.target.value)}
            placeholder="https://your-robot.ngrok-free.dev"
            value={url}
          />
          <Button
            disabled={busy || !dirty || !url.trim()}
            onClick={() => save(url.trim())}
          >
            {busy ? <Loader2 className="animate-spin" size={16} /> : null}
            Save
          </Button>
          {settings?.url ? (
            <Button
              disabled={busy}
              onClick={() => save(null)}
              title="Reset to the server's DIMOS_AGENT_URL"
              variant="outline"
            >
              <RotateCcw size={15} />
            </Button>
          ) : null}
        </div>
      </div>

      {urlOnly ? (
        <p className="text-muted-foreground text-xs">
          A URL is set but the server has no <code>DIMOS_AGENT_TOKEN</code> —
          commands will be rejected until a token is configured on the server.
        </p>
      ) : null}
      {msg ? <p className="text-signal text-xs">{msg}</p> : null}
      {error ? <p className="text-destructive text-xs">{error}</p> : null}
    </div>
  );
}
