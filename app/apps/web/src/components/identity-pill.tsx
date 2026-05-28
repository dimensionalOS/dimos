import type { Agent } from "@robomoo/shared";
import { BadgeCheck } from "lucide-react";

const EXPLORER = "https://sepolia.etherscan.io";

// Verified-identity badge. When an agent has a registered identity, show a
// subtle "Verified" pill linking to its public record; otherwise render nothing
// (no setup/registration jargon ever surfaces to the user).
export function IdentityPill({ agent }: { agent: Agent }) {
  if (!agent.agentId) return null;
  const href = agent.registerTx
    ? `${EXPLORER}/tx/${agent.registerTx}`
    : `${EXPLORER}/address/${agent.agentWallet ?? ""}`;
  return (
    <a
      className="inline-flex items-center gap-1 rounded-full border border-verified/40 bg-verified/10 px-2 py-0.5 font-medium text-verified text-xs transition-colors hover:bg-verified/20"
      href={href}
      rel="noopener noreferrer"
      target="_blank"
      title="Verified identity · view record"
    >
      <BadgeCheck size={13} /> Verified
    </a>
  );
}
