import type { Agent } from "@robomoo/shared";
import { BadgeCheck, ShieldOff } from "lucide-react";

const ETHERSCAN = "https://sepolia.etherscan.io";

// ERC-8004 on-chain identity status. Registered → a cyan "verified" pill linking
// to the register tx (or the identity wallet); otherwise a muted hint.
export function IdentityPill({ agent }: { agent: Agent }) {
  if (agent.agentId) {
    const href = agent.registerTx
      ? `${ETHERSCAN}/tx/${agent.registerTx}`
      : `${ETHERSCAN}/address/${agent.agentWallet ?? ""}`;
    return (
      <a
        className="inline-flex items-center gap-1 rounded-full border border-verified/40 bg-verified/10 px-2 py-0.5 font-medium text-verified text-xs transition-colors hover:bg-verified/20"
        href={href}
        rel="noopener noreferrer"
        target="_blank"
        title="ERC-8004 verified identity on Ethereum Sepolia"
      >
        <BadgeCheck size={13} /> Agent #{agent.agentId}
      </a>
    );
  }
  return (
    <span className="inline-flex items-center gap-1 rounded-full border bg-muted/50 px-2 py-0.5 text-muted-foreground text-xs">
      <ShieldOff size={12} /> Unregistered
    </span>
  );
}
