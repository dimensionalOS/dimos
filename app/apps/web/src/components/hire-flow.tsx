"use client";

import type { Agent, AgentService } from "@robomoo/shared";
import { useRouter } from "next/navigation";
import { useState } from "react";
import { Button } from "@/components/ui/button";
import { rpcClient } from "@/lib/orpc";
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
    <div className="flex flex-col gap-4 rounded-xl border bg-card p-5">
      <h3 className="font-semibold">Hire {agent.name}</h3>

      {agent.services.length > 1 ? (
        <div className="flex flex-col gap-2">
          {agent.services.map((s) => (
            <label
              className={`flex cursor-pointer items-start justify-between gap-3 rounded-lg border p-3 text-sm ${
                s.key === serviceKey ? "border-foreground/40 bg-accent" : ""
              }`}
              key={s.key}
            >
              <span className="flex flex-col gap-0.5">
                <span className="font-medium">{s.name}</span>
                <span className="text-muted-foreground text-xs">{s.desc}</span>
              </span>
              <span className="flex items-center gap-2">
                <span className="font-medium">${s.priceUsd}</span>
                <input
                  checked={s.key === serviceKey}
                  name="service"
                  onChange={() => setServiceKey(s.key)}
                  type="radio"
                  value={s.key}
                />
              </span>
            </label>
          ))}
        </div>
      ) : (
        <div className="flex items-start justify-between gap-3 rounded-lg border p-3 text-sm">
          <span className="flex flex-col gap-0.5">
            <span className="font-medium">{service.name}</span>
            <span className="text-muted-foreground text-xs">{service.desc}</span>
            {service.durationHint ? (
              <span className="text-muted-foreground text-xs">
                {service.durationHint}
              </span>
            ) : null}
          </span>
          <span className="font-medium">${service.priceUsd}</span>
        </div>
      )}

      <div className="flex items-center justify-between">
        <span className="text-muted-foreground text-xs">
          Pay with {X402_ENABLED ? "x402 (Base Sepolia USDC)" : "test payment"}
        </span>
        <span className="font-semibold text-lg">${service.priceUsd}</span>
      </div>

      <Button disabled={busy} onClick={hire} size="lg">
        {phase === "paying"
          ? "Processing payment…"
          : phase === "dispatching"
            ? "Dispatching RoboDoc…"
            : address
              ? `Hire — pay $${service.priceUsd}`
              : "Connect wallet to hire"}
      </Button>

      {error ? <p className="text-destructive text-sm">{error}</p> : null}
      <p className="text-muted-foreground text-xs">
        You&apos;ll be taken to a live job view to watch {agent.name} work and
        collect your deliverable.
      </p>
    </div>
  );
}
