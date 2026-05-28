"use client";

import type { Agent, AgentService } from "@robomoo/shared";
import { Loader2 } from "lucide-react";
import { useRouter } from "next/navigation";
import { useState } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { RadioGroup, RadioGroupItem } from "@/components/ui/radio-group";
import { rpcClient } from "@/lib/orpc";
import { cn } from "@/lib/utils";
import { useWallet } from "@/lib/wallet";

const X402_ENABLED = process.env.NEXT_PUBLIC_X402_ENABLED === "true";

// Mocked registry of maps the robot has on file (mirrors rooms-panel's list).
// "Use existing" picks one of these; "Create new" names a fresh map.
const KNOWN_MAPS = ["kitchen", "office"];

type MapChoice = { mode: "existing" | "new"; name: string };

// Map a service (+ optional map choice) to the natural-language command
// dispatched to the DimOS agent.
function commandFor(
  agent: Agent,
  service: AgentService,
  map: MapChoice | null,
): string {
  if (service.key === "room-scan") {
    const name = map?.name.trim();
    if (map && name) {
      return map.mode === "existing"
        ? `You're in the ${name}. Load that room's saved map, then scan the room for VR.`
        : `Explore and map this room as "${name}", save the map, then scan the room for VR.`;
    }
    return "scan the room for VR";
  }
  return `${service.name} (${agent.name})`;
}

type Phase = "idle" | "paying" | "dispatching";

export function HireFlow({ agent }: { agent: Agent }) {
  const router = useRouter();
  const { address, connect, connecting, ensureSepolia } = useWallet();
  const [serviceKey, setServiceKey] = useState(agent.services[0]?.key ?? "");
  const [mapMode, setMapMode] = useState<"existing" | "new">("existing");
  const [existingMap, setExistingMap] = useState(KNOWN_MAPS[0] ?? "");
  const [newMap, setNewMap] = useState("");
  const [phase, setPhase] = useState<Phase>("idle");
  const [error, setError] = useState<string | null>(null);

  const service =
    agent.services.find((s) => s.key === serviceKey) ?? agent.services[0];

  if (!service) {
    return (
      <p className="text-muted-foreground text-sm">
        This agent has no bookable services yet.
      </p>
    );
  }

  const isRoomScan = service.key === "room-scan";
  const mapChoice: MapChoice | null = isRoomScan
    ? { mode: mapMode, name: mapMode === "existing" ? existingMap : newMap }
    : null;
  const mapReady =
    !isRoomScan ||
    (mapMode === "existing" ? Boolean(existingMap) : Boolean(newMap.trim()));

  const hire = async () => {
    setError(null);
    if (!address) {
      await connect();
      return;
    }
    // Nudge to Sepolia (so the later on-chain rating works) but don't block the
    // mock payment on it.
    void ensureSepolia();
    try {
      setPhase("paying");
      const job = await rpcClient.jobs.create({
        agentSlug: agent.slug,
        service: service.key,
        requesterAddr: address,
      });
      // Mock settlement beat. Real x402 (Base Sepolia USDC) lands here when the
      // flag + facilitator are wired; for the MVP this is a simulated payment.
      await new Promise((r) => setTimeout(r, 700));
      await rpcClient.jobs.pay({
        id: job.id,
        paymentMode: X402_ENABLED ? "x402" : "mock",
        paymentTx: null,
      });
      setPhase("dispatching");
      await rpcClient.jobs.dispatch({
        id: job.id,
        command: commandFor(agent, service, mapChoice),
      });
      router.push(`/jobs/${job.id}`);
    } catch (e) {
      setError(e instanceof Error ? e.message : "failed to hire");
      setPhase("idle");
    }
  };

  const busy = phase !== "idle" || connecting;

  return (
    <div className="flex flex-col gap-5 rounded-xl border bg-card p-6">
      <div className="flex flex-col gap-1">
        <h3 className="font-display font-semibold text-lg">Hire {agent.name}</h3>
        <p className="text-muted-foreground text-xs">
          Pick a service, pay, and watch it work live.
        </p>
      </div>

      <RadioGroup
        className="flex flex-col gap-2"
        onValueChange={setServiceKey}
        value={serviceKey}
      >
        {agent.services.map((s) => {
          const sel = s.key === serviceKey;
          return (
            <label
              className={cn(
                "flex cursor-pointer items-start gap-3 rounded-lg border p-3 text-sm transition-colors",
                sel
                  ? "border-signal/50 bg-signal/5"
                  : "hover:border-foreground/20",
              )}
              htmlFor={`svc-${s.key}`}
              key={s.key}
            >
              <RadioGroupItem
                className="mt-0.5"
                id={`svc-${s.key}`}
                value={s.key}
              />
              <span className="flex flex-1 flex-col gap-0.5">
                <span className="flex items-center justify-between gap-2">
                  <span className="font-medium">{s.name}</span>
                  <span className="font-medium font-mono">${s.priceUsd}</span>
                </span>
                <span className="text-muted-foreground text-xs leading-relaxed">
                  {s.desc}
                </span>
                {s.durationHint ? (
                  <span className="text-muted-foreground/70 text-xs">
                    ⏱ {s.durationHint}
                  </span>
                ) : null}
              </span>
            </label>
          );
        })}
      </RadioGroup>

      {isRoomScan ? (
        <div className="flex flex-col gap-3 border-t pt-4">
          <span className="font-medium text-sm">Map</span>
          <div className="flex gap-2">
            <button
              className={cn(
                "flex-1 rounded-lg border p-2 text-sm transition-colors",
                mapMode === "existing"
                  ? "border-signal/50 bg-signal/5"
                  : "hover:border-foreground/20",
              )}
              onClick={() => setMapMode("existing")}
              type="button"
            >
              Use existing map
            </button>
            <button
              className={cn(
                "flex-1 rounded-lg border p-2 text-sm transition-colors",
                mapMode === "new"
                  ? "border-signal/50 bg-signal/5"
                  : "hover:border-foreground/20",
              )}
              onClick={() => setMapMode("new")}
              type="button"
            >
              Create new map
            </button>
          </div>
          {mapMode === "existing" ? (
            <div className="flex flex-wrap gap-2">
              {KNOWN_MAPS.map((room) => (
                <button
                  className={cn(
                    "rounded-md border px-3 py-1 text-sm capitalize transition-colors",
                    existingMap === room
                      ? "border-signal text-signal"
                      : "hover:border-foreground/30",
                  )}
                  key={room}
                  onClick={() => setExistingMap(room)}
                  type="button"
                >
                  {room}
                </button>
              ))}
            </div>
          ) : (
            <Input
              onChange={(e) => setNewMap(e.target.value)}
              placeholder="new room name (e.g. living room)"
              value={newMap}
            />
          )}
        </div>
      ) : null}

      <div className="flex flex-col gap-2 border-t pt-4 text-sm">
        <div className="flex items-center justify-between text-muted-foreground">
          <span>{service.name}</span>
          <span className="font-mono">${service.priceUsd}</span>
        </div>
        <div className="flex items-center justify-between font-medium">
          <span>Total</span>
          <span className="font-display text-xl">${service.priceUsd}</span>
        </div>
        <span className="inline-flex w-fit items-center gap-1.5 rounded-full bg-secondary/60 px-2 py-0.5 font-mono text-muted-foreground text-xs">
          {X402_ENABLED ? "instant payment" : "test payment · no real funds"}
        </span>
      </div>

      <Button
        className="w-full"
        disabled={busy || !mapReady}
        onClick={hire}
        size="lg"
      >
        {busy ? <Loader2 className="animate-spin" size={16} /> : null}
        {phase === "paying"
          ? "Processing payment…"
          : phase === "dispatching"
            ? `Dispatching ${agent.name}…`
            : address
              ? `Hire — pay $${service.priceUsd}`
              : "Connect wallet to hire"}
      </Button>

      {error ? <p className="text-destructive text-sm">{error}</p> : null}
      <p className="text-muted-foreground text-xs leading-relaxed">
        You&apos;ll be taken to a live job view to watch {agent.name} work and
        collect your deliverable.
      </p>
    </div>
  );
}
