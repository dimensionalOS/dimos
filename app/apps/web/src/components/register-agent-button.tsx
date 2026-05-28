"use client";

import type { Agent } from "@robomoo/shared";
import { Loader2 } from "lucide-react";
import { useRouter } from "next/navigation";
import { useState } from "react";
import { Button } from "@/components/ui/button";
import { registerAgentOnchain } from "@/lib/chain";
import { rpcClient } from "@/lib/orpc";
import { useWallet } from "@/lib/wallet";

// Operator action: register this agent on the ERC-8004 Identity Registry from
// the connected browser wallet. agentURI points at the server's card.json. On
// success the minted agentId + tx are persisted so the marketplace shows a
// verified identity. See the PR description for the full how-to.
export function RegisterAgentButton({ agent }: { agent: Agent }) {
  const router = useRouter();
  const { address, connect, ensureSepolia, onSepolia } = useWallet();
  const [busy, setBusy] = useState(false);
  const [msg, setMsg] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  if (agent.agentId) {
    return (
      <p className="text-muted-foreground text-xs">
        Registered on-chain as agent #{agent.agentId}.
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
      setMsg("Confirm the register() transaction in your wallet…");
      const { txHash, agentId } = await registerAgentOnchain(address, agentURI);
      if (!agentId) {
        setError(
          "The register transaction landed, but we couldn't read the agent ID back from it. Check the tx on Sepolia Etherscan and set it manually.",
        );
        return;
      }
      setMsg("Saving on-chain identity…");
      await rpcClient.agents.setOnchain({
        slug: agent.slug,
        agentId,
        agentWallet: address,
        registerTx: txHash,
      });
      setMsg(`Verified as agent #${agentId} ✓`);
      router.refresh();
    } catch (e) {
      setError(e instanceof Error ? e.message : "registration failed");
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="flex flex-col gap-1.5">
      <Button disabled={busy} onClick={run} size="sm" variant="outline">
        {busy ? <Loader2 className="animate-spin" size={14} /> : null}
        {busy
          ? "Registering…"
          : address
            ? "Register on-chain (ERC-8004)"
            : "Connect wallet to register"}
      </Button>
      {msg ? <span className="text-muted-foreground text-xs">{msg}</span> : null}
      {error ? <span className="text-destructive text-xs">{error}</span> : null}
    </div>
  );
}
