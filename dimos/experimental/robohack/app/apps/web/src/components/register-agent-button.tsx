"use client";

import type { Agent } from "@robomoo/shared";
import { Loader2 } from "lucide-react";
import { useRouter } from "next/navigation";
import { useState } from "react";
import { Button } from "@/components/ui/button";
import { registerAgentOnchain } from "@/lib/chain";
import { rpcClient } from "@/lib/orpc";
import { useWallet } from "@/lib/wallet";

// Operator-only setup action (rendered behind ?operator=1): give this agent a
// verifiable identity from the connected wallet. agentURI points at the
// server's card.json; the minted id + record are persisted so the marketplace
// can show a "Verified" badge linking to the public record.
export function RegisterAgentButton({ agent }: { agent: Agent }) {
  const router = useRouter();
  const { address, connect, ensureSepolia, onSepolia } = useWallet();
  const [busy, setBusy] = useState(false);
  const [msg, setMsg] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  if (agent.agentId) {
    return (
      <p className="text-muted-foreground text-xs">
        Identity set up ✓ (#{agent.agentId}).
      </p>
    );
  }

  const run = async () => {
    setError(null);
    setMsg(null);
    if (!address) {
      await connect();
      return;
    }
    if (!onSepolia) {
      const ok = await ensureSepolia();
      if (!ok) return;
    }
    setBusy(true);
    try {
      const origin = window.location.origin;
      const agentURI = `${origin}/api/agents/${agent.slug}/card.json`;
      setMsg("Confirm the transaction in your wallet…");
      const { txHash, agentId } = await registerAgentOnchain(address, agentURI);
      if (!agentId) {
        setError(
          "The transaction landed, but we couldn't read the identity back from it. Check it on the explorer and set it manually.",
        );
        return;
      }
      setMsg("Saving identity…");
      await rpcClient.agents.setOnchain({
        slug: agent.slug,
        agentId,
        agentWallet: address,
        registerTx: txHash,
      });
      setMsg(`Identity verified ✓ (#${agentId})`);
      router.refresh();
    } catch (e) {
      setError(e instanceof Error ? e.message : "verification failed");
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="flex flex-col gap-1.5">
      <Button disabled={busy} onClick={run} size="sm" variant="outline">
        {busy ? <Loader2 className="animate-spin" size={14} /> : null}
        {busy
          ? "Verifying…"
          : address
            ? "Verify identity"
            : "Connect wallet to verify"}
      </Button>
      {msg ? <span className="text-muted-foreground text-xs">{msg}</span> : null}
      {error ? <span className="text-destructive text-xs">{error}</span> : null}
    </div>
  );
}
