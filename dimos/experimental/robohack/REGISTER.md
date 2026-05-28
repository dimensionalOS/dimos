# Registering RoboDoc as an on-chain agent (ERC-8004)

How to give an agent a verifiable on-chain identity + reputation on the
**Ethereum Sepolia** testnet, using the browser-wallet flow built into the
marketplace. RoboDoc is the live agent; the same steps work for any agent.

> The whole on-chain layer is **resilient**: if the reputation registry isn't
> configured (or the chain is unreachable), the UI falls back to job-derived
> reputation and the demo still works. Wire the chain up to make identity +
> reputation *real* and verifiable on a block explorer.

---

## 1. The agent card (`agentURI`)

ERC-8004 registers an agent by pointing the Identity Registry at a metadata
document — the **`agentURI`**. We serve it automatically per agent at:

```
GET /api/agents/<slug>/card.json      e.g. /api/agents/robodoc/card.json
```

It's generated from the seeded agent row (`apps/server/src/seed-agents.ts`) and
looks like:

```json
{
  "name": "RoboDoc",
  "description": "An autonomous Unitree Go2 you can hire to survey a space…",
  "tagline": "Comes to your home, scans every room, hands you a 3D virtual tour.",
  "image": "🐕‍🦺",
  "type": "robot-agent",
  "provider": "robomoo",
  "chain": "eip155:11155111",
  "capabilities": ["room-scan", "segmentation", "gaussian-splat", "mapping"],
  "services": [{ "key": "room-scan", "name": "Room Scan + 3D Tour", "priceUsd": 25, … }],
  "url": "https://<your-app>/agents/robodoc",
  "cardUrl": "https://<your-app>/api/agents/robodoc/card.json"
}
```

To **edit** an agent's card, edit its entry in `seed-agents.ts` and redeploy
(the seed is idempotent on `slug` and won't clobber on-chain fields). To **add**
a new agent, add a `SEED` entry. The `agentURI` must be publicly reachable for
anyone resolving the identity — so use your deployed `APP_URL`, not localhost,
when you register for real.

---

## 2. One-time setup

1. **Browser wallet** — MetaMask (or any injected EIP-1193 wallet).
2. **Sepolia ETH for gas** — from a faucet (e.g. https://www.alchemy.com/faucets/ethereum-sepolia).
3. **Env** (`apps/web/.env`):
   ```bash
   # Identity Registry — default is the verified reference deployment; keep as-is.
   NEXT_PUBLIC_ERC8004_IDENTITY_ADDRESS=0xf66e7CBdAE1Cb710fee7732E4e1f173624e137A7
   # Reputation Registry — REQUIRED for on-chain reputation. Get the Sepolia
   # address from the reference repo's deployments (see step 5).
   NEXT_PUBLIC_ERC8004_REPUTATION_ADDRESS=0x...
   # Optional dedicated RPC (else viem's public Sepolia default is used).
   NEXT_PUBLIC_SEPOLIA_RPC_URL=
   ```

---

## 3. Register the agent on-chain

1. Open the agent profile: **`/agents/robodoc`**.
2. Click **Connect wallet** (top-right) and approve. Switch to **Sepolia** if
   prompted.
3. Under **On-chain identity**, click **Register on-chain (ERC-8004)**.
4. Confirm the **`register(agentURI)`** transaction in your wallet. The app:
   - waits for the receipt,
   - parses the minted **`agentId`** from the ERC-721 `Transfer` event,
   - persists `agentId` + your wallet + the tx hash (`agents.setOnchain`).
5. The page reloads and the identity pill flips to **✓ Agent #\<id\>**, linking
   to the tx on Sepolia Etherscan.

**If the agentId can't be auto-read** (some deployments emit it differently): open
the register tx on Sepolia Etherscan, copy the minted token id, and set it via
the oRPC `agents.setOnchain` mutation (or directly: `UPDATE agents SET agent_id =
'<id>', register_tx = '<tx>', agent_wallet = '<addr>' WHERE slug = 'robodoc';`).

---

## 4. Earn reputation (per completed job)

1. Hire RoboDoc from its profile → watch the **live job view** → it produces the
   scanned/segmented rooms + a 3D tour.
2. When the job hits **Done**, the **Rate \<agent\>** panel appears.
3. Pick a star rating and **Submit**. If `agentId` is set **and** the reputation
   registry is configured, this signs **`giveFeedback(agentId, value, …)`** from
   your wallet on Sepolia; the tx hash is stored on the job (etherscan link
   shown). Otherwise the rating is recorded off-chain only.
4. The **reputation badge** (marketplace + profile) polls
   **`getSummary(agentId, …)`** and ticks up — the live on-chain reputation
   counter.

---

## 5. Confirming the Reputation Registry address + ABIs

The Identity Registry address is verified (in `lib/chain.ts`). The **Reputation
Registry** Sepolia address and the exact `giveFeedback` / `getSummary` ABIs come
from the reference implementation — confirm them against:

- `ChaosChain/trustless-agents-erc-ri` (Sepolia deployments / broadcast)
- EIP-8004: https://eips.ethereum.org/EIPS/eip-8004

The ABIs in `apps/web/src/lib/chain.ts` are best-effort fragments for the
reference shape; if your deployed contract differs, adjust them there. Reads are
wrapped in try/catch so a mismatch degrades gracefully to off-chain stats rather
than breaking the page.

---

## 6. Notes for a live demo

- **Pre-register before the demo** and **pre-fund** the wallet with Sepolia ETH
  — faucets rate-limit and venue Wi-Fi fights you.
- Keep rating an **operator action** (your own connected wallet) so the
  reputation tick-up always lands, even if visitors don't have wallets.
- Payments are **mocked** for the MVP (`NEXT_PUBLIC_X402_ENABLED` only changes
  the label). Wiring real x402 on Base Sepolia is the deferred P6 step.
- On a mainland-China public floor, treat the on-chain pieces as the private
  "global mode" on **testnet only** — never put a mainnet key on the venue
  laptop (see `X402_ERC8004.md`).
