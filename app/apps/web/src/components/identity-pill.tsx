import type { Agent } from "@robomoo/shared";

const ETHERSCAN = "https://sepolia.etherscan.io";

// Shows the agent's ERC-8004 on-chain identity status. Registered → a verified
// pill linking to the register tx (or the identity token); otherwise a muted
// "not registered" hint.
export function IdentityPill({ agent }: { agent: Agent }) {
  if (agent.agentId) {
    const href = agent.registerTx
      ? `${ETHERSCAN}/tx/${agent.registerTx}`
      : `${ETHERSCAN}/address/${agent.agentWallet ?? ""}`;
    return (
      <a
        className="inline-flex items-center gap-1 rounded-full border border-emerald-500/40 bg-emerald-500/10 px-2 py-0.5 text-emerald-600 text-xs hover:bg-emerald-500/20"
        href={href}
        rel="noopener noreferrer"
        target="_blank"
        title="ERC-8004 verified identity on Ethereum Sepolia"
      >
        ✓ Agent #{agent.agentId}
      </a>
    );
  }
  return (
    <span className="inline-flex items-center gap-1 rounded-full border bg-muted px-2 py-0.5 text-muted-foreground text-xs">
      Identity: not registered
    </span>
  );
}
