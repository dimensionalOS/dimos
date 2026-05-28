"use client";

import type { Agent, AgentService } from "@robomoo/shared";
import { Loader2 } from "lucide-react";
import { useRouter } from "next/navigation";
import { useState } from "react";
import { Button } from "@/components/ui/button";
import { RadioGroup, RadioGroupItem } from "@/components/ui/radio-group";
import { rpcClient } from "@/lib/orpc";
import { cn } from "@/lib/utils";
import { useWallet } from "@/lib/wallet";

const X402_ENABLED = process.env.NEXT_PUBLIC_X402_ENABLED === "true";

// Map a service to the natural-language command dispatched to the DimOS agent.
function commandFor(agent: Agent, service: AgentService): string {
  if (service.key === "room-scan") return "scan the room for VR";
  return `${service.name} (${agent.name})`;
}

type Phase = "idle" | "paying" | "dispatching";

export function HireFlow({ agent }: { agent: Agent }) {
  const router = useRouter();
  const { address, connect, connecting, ensureSepolia } = useWallet();
  const [serviceKey, setServiceKey] = useState(agent.services[0]?.key ?? "");
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
        command: commandFor(agent, service),
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
          {X402_ENABLED ? "x402 · Base Sepolia USDC" : "test payment · no real funds"}
        </span>
      </div>

      <Button className="w-full" disabled={busy} onClick={hire} size="lg">
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
