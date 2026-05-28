# PLAN — robomoo: Robot-Agent Marketplace MVP

> Wrap the existing RoboDoc (Go2) scan → segmentation → 3D-tour pipeline in a polished,
> demo-ready UI framed as an **on-chain agent marketplace**: browse hireable robot agents,
> each with an **ERC-8004 identity + live reputation**, hire one, watch it work, and get the
> deliverable (segmented rooms + Gaussian-splat virtual tour).

---

## 0. Decisions locked

| Decision | Choice |
|---|---|
| Scope | MVP polish — one coherent demo story, not a research toy |
| UI shape | **Agent marketplace** (registry of agents); RoboDoc is the one fully-wired card |
| Brand | Keep **robomoo** name |
| Chain lib | **viem** |
| ERC-8004 | **Real on Ethereum Sepolia**, signed via **browser wallet** (injected / MetaMask) |
| x402 | Behind a feature flag, **mocked by default**; real Base Sepolia round-trip later |
| Seed agents | **RoboDoc (real) + 5 fictional** = 6 marketplace cards |

The ERC-8004 registry **is** the marketplace: each card = a registered on-chain agent. That makes
"register our dog as a hireable agent" the centerpiece, not a side feature.

---

## 1. What already exists (reuse, don't rebuild)

Backend is solid (`app/`, Bun/Turbo monorepo). Keep all of it:

- **server** (Hono): token-guarded robot ingest (`/api/robot/frame|map|splat|trajectory`,
  `/frame/:id/analysis`), `commands.send` → DimOS agent over tunnel, presigned object storage.
- **api** (oRPC): `frames.{list,embedded,scans,scansHeaders,analyses}`, `map.latest`,
  `trajectory.latest`, `splats.list`, `commands.send`, `messages.{list,add}`.
- **db** (Drizzle/Postgres): `frames` (run/position/angle/pose/label/embedding), `frame_analyses`
  + `frame_analysis_objects` (Falcon segmentation crops/masks), `maps`, `trajectories`, `splats`,
  `auth`, `messages`.
- **web** (Next 16 + shadcn + Tailwind 4): working but dev-grade pages `/frames /scans /map
  /splats /control`. The **ScansBrowser** (segmentation lightbox), **SplatViewer** (Spark 3D
  walkthrough), **MapView** (costmap + trajectory) components are the reusable gold.

The gap is **presentation + framing + on-chain glue**, not capability.

---

## 2. New data model

Two new Drizzle tables (+ migration). Keep existing tables untouched.

### `agents` — the marketplace / registry rows
```
id            text pk           // agent_xxx
slug          text unique       // "robodoc", "patrol-pup", ...
name          text
tagline       text
description    text
avatarKey     text null         // object-storage key (or static /public asset)
services      jsonb             // [{ key, name, desc, priceUsd, durationHint }]
basePriceUsd  numeric
isReal        boolean           // RoboDoc=true; fictional=false
status        text              // "live" | "coming_soon"
chain         text default 'eip155:11155111'   // Ethereum Sepolia
agentId       text null         // ERC-8004 NFT id once registered (real reads use this)
agentWallet   text null         // address bound via setAgentWallet
registerTx    text null         // identity register() tx hash
capabilities  jsonb             // tags: ["segmentation","gaussian-splat","mapping"]
createdAt     timestamptz
```

### `jobs` — one hire, ties results + payment + reputation together
```
id              text pk          // job_xxx
agentSlug       text fk agents.slug
requesterAddr   text null        // connected wallet
requesterUserId text null        // better-auth user (optional)
service         text             // services[].key
status          text             // booked|dispatched|scanning|reconstructing|done|failed|cancelled
priceUsd        numeric
paid            boolean default false
paymentMode     text             // "mock" | "x402"
paymentTx       text null        // x402 settlement tx (real mode)
command         text null        // NL command dispatched to the dog
run             text null        // links to frames.run — the scan sweep this job produced
splatId         text null fk splats.id   // the 3D-tour deliverable
rating          integer null     // 1..5 the requester gave
feedbackTx      text null        // ERC-8004 giveFeedback() tx hash
createdAt       timestamptz
dispatchedAt    timestamptz null
completedAt     timestamptz null
```

**Pinning results to a job (the key mechanic):** `job.run` links the job to a scan sweep, and
`job.splatId` to its 3D tour. Because the robot/DimOS agent currently names its own `run`, the
robust MVP approach is **claim-latest**:

1. On hire → dispatch, set `status=scanning`, `dispatchedAt=now`, store `command`.
2. Poll `frames.scansHeaders`; when a run appears with `capturedAt > dispatchedAt` and the job has
   no `run` yet, attach it (`job.run = newest.run`, `status=reconstructing` once a splat lands, then
   `done`). Expose a manual "attach this run" override for demo safety.

(Stretch: pre-assign `run = job.id` and pass it in the command so the skill tags frames with it —
needs a robot-side change, so keep claim-latest as the default.)

---

## 3. ERC-8004 integration (viem + browser wallet)

**Chain:** Ethereum Sepolia (`eip155:11155111`). **Identity Registry (verified):**
`0xf66e7CBdAE1Cb710fee7732E4e1f173624e137A7`.

### Phase-0 lookups (do first — don't fabricate)
- [ ] Reputation Registry Sepolia address (from `ChaosChain/trustless-agents-erc-ri` deployments /
      broadcast).
- [ ] ABIs for `register`, `setAgentWallet`, `giveFeedback`, `getSummary` (extract from the
      reference repo; or hand-write minimal ABI fragments for just these 4 calls).

### Client setup (`web/src/lib/chain.ts`)
```ts
import { createPublicClient, createWalletClient, custom, http } from "viem";
import { sepolia } from "viem/chains";

export const publicClient = createPublicClient({ chain: sepolia, http: http() });
export function walletClient() {
  return createWalletClient({ chain: sepolia, transport: custom(window.ethereum) });
}
```
A `useWallet()` hook: connect (eth_requestAccounts), expose `address`, `chainId`, a
`switchToSepolia()` guard, and disconnect. Small "Connect wallet" button in the marketplace header.

### Three on-chain flows
1. **Register agent (one-time, operator action).** On RoboDoc's profile, a dev-gated "Register
   on-chain" button: `agentURI` = our hosted card JSON (`GET /api/agents/:slug/card.json` describing
   the agent + services), `walletClient.writeContract(identity.register, [agentURI])` → read the
   `agentId` from logs → `PATCH` it back onto the `agents` row (+ `setAgentWallet`). Persist
   `registerTx`.
2. **Reputation read (everywhere).** `publicClient.readContract(reputation.getSummary, [agentId,
   [], tag, ""])` → `(count, summaryValue, decimals)` → render "★ 4.8 over N jobs" on the card and
   profile. Poll every ~10s.
3. **Give feedback (per completed job).** When a job hits `done`, the requester's connected wallet
   signs `giveFeedback(agentId, 5, 0, "taskRating", "", endpoint, ...)`. Store `feedbackTx` +
   `rating` on the job. Show optimistic "+1 job" immediately; reconcile when the tx confirms.

Fictional agents: show a **mock** identity badge + reputation (so the registry looks full) and a
"Coming soon" hire button. Only RoboDoc is wired to real chain reads/writes for the MVP (others can
be registered later with the same button).

---

## 4. x402 (flagged, mock-by-default)

`NEXT_PUBLIC_X402_ENABLED` (default off). Hire flow always shows a "Pay 0.10 USDC" step:

- **Mock mode (default):** resolves after a short faux-settlement animation → `paid=true`,
  `paymentMode="mock"`. Demo can't be broken by network.
- **Real mode (later):** browser x402 client signs USDC on **Base Sepolia** (`eip155:84532`, USDC
  `0x036CbD53842c5426634e7929541eC2318f3dCF7e`, facilitator `https://x402.org/facilitator`) against
  a paywalled `POST /jobs/:id/pay` on the server. Note the deliberate **two-chain split**: identity/
  reputation on Ethereum Sepolia, payment on Base Sepolia — the wallet handles both.

Keep payment a thin gate in front of dispatch; never block the dog's action on settlement finality.

---

## 5. UI / page spec

Restyle into a real product. New shared chrome: a top nav (robomoo wordmark, Marketplace, My jobs,
Connect wallet) + a polished card/badge system. Keep `/frames /map` as "raw/debug" links in a footer.

### `/` — Marketplace (replaces the message board)
- Hero strip: "robomoo — hire autonomous robot agents. On-chain identity, real reputation."
- Grid of **6 agent cards** (RoboDoc + 5 fictional). Each card: avatar, name, tagline, capability
  tags, price-from, **reputation badge** (★ + job count), an on-chain "verified identity" pill
  (agentId / etherscan link), and a Hire / Coming-soon CTA.
- Connect-wallet button in header.

### `/agents/[slug]` — Agent profile
- Header: name, tagline, **ERC-8004 identity panel** (agentId, bound wallet, etherscan links,
  register tx), **live reputation** (getSummary counter + recent ratings).
- Services list with prices. Primary **Hire** CTA opens the hire flow.
- "Past work" = this agent's completed jobs (deliverable thumbnails).
- RoboDoc = fully live; fictional = mock badges + disabled hire.

### Hire flow (modal or `/agents/[slug]/hire`)
Stepper: **1 Pick service → 2 Connect wallet → 3 Pay (mock/x402) → 4 Dispatch.**
On confirm: create `job` (status=booked→paid→dispatched), call `commands.send` with the service's
command (e.g. "scan the room for VR"), route to the live job view.

### `/jobs/[id]` — Live job view (the "watch it work" beat)
- Status header with a progress stepper (booked → scanning → reconstructing → done).
- Live panels (poll existing RPCs): **MapView** (costmap + trajectory drawing as the dog moves) +
  incoming **frames** stream + "scanning…" state. Reuse `map.latest`, `trajectory.latest`,
  `frames.scansHeaders` filtered to the claimed `run`.
- When `run` attaches → embed the **ScansBrowser** segmentation view scoped to that run.
- When a splat lands → "Your virtual tour is ready" → **SplatViewer** (Spark walkthrough).
- On `done` → **Rate this agent ★★★★★** → triggers `giveFeedback` (browser wallet) → reputation
  ticks up on the profile/marketplace.

### `/jobs` — My jobs
List of the connected wallet's (or session's) jobs with status + deliverable links.

### Deliverable = the existing components, re-skinned
- Segmentation: `ScansBrowser` (run-scoped) — masks/crops/object list already work.
- 3D tour: `SplatViewer` — already streams `.spz/.ply` via Spark.

---

## 6. API additions (oRPC + server)

New router `agents`:
- `agents.list()` → marketplace cards (+ cached reputation if we store it; else client reads chain).
- `agents.get({ slug })` → profile.
- `agents.setOnchain({ slug, agentId, agentWallet, registerTx })` — protected; called after the
  browser-wallet register tx confirms.

New router `jobs`:
- `jobs.create({ agentSlug, service })` → `job` (booked).
- `jobs.dispatch({ id })` → sets dispatched, calls `commands.send`, returns job.
- `jobs.get({ id })` / `jobs.listMine()` — with claim-latest run/splat attachment logic.
- `jobs.attachRun({ id, run })` / `jobs.complete({ id })` — manual overrides for demo.
- `jobs.setFeedback({ id, rating, feedbackTx })` — records the on-chain rating.

Server: `GET /api/agents/:slug/card.json` (the ERC-8004 `agentURI` metadata document).
Seed: insert RoboDoc + 5 fictional agents on boot (idempotent, like the existing message seed).

### The 5 fictional agents (seed)
1. **RoboDoc** *(real)* — Go2 room scanner → segmentation + 3D virtual tour.
2. **Patrol Pup** — autonomous security patrol that narrates anomalies (coming soon).
3. **Fetch** — delivery dog, brings you an item (coming soon).
4. **Surveyor** — outdoor site mapping & inspection (coming soon).
5. **Maestro** — swarm-choreography performer (coming soon).
6. **Greeter** — concierge / guided-tour host (coming soon).

---

## 7. Build phases

- **P0 — Facts & scaffold:** look up Reputation registry address + ABIs; add viem + wallet hook;
  add `agents`/`jobs` tables + migration + seed; `agents.card.json` route.
- **P1 — Marketplace + profile:** nav chrome, card system, `/` grid, `/agents/[slug]`, mock badges
  for all, real `getSummary` read for RoboDoc.
- **P2 — Hire → dispatch → live job:** hire stepper (mock pay), `jobs.create/dispatch`, `/jobs/[id]`
  live view reusing MapView/frames, claim-latest run attachment.
- **P3 — Deliverable:** run-scoped ScansBrowser + SplatViewer in the job view; `/jobs` list.
- **P4 — On-chain real:** browser-wallet `register` (RoboDoc) + `giveFeedback` on job done; reputation
  counter live across marketplace/profile.
- **P5 — Polish:** loading/empty states, etherscan links, demo seed data, mobile-friendly layout.
- **P6 (stretch):** real x402 Base Sepolia payment behind the flag.

---

## 8. Open questions / risks

- **Reputation registry address + ABIs** — must confirm from the reference repo (P0 blocker for real
  reads/writes; mock works without it).
- **Two chains** (8004 on Eth Sepolia, x402 on Base Sepolia) — fine for viem, but the wallet UX must
  prompt the right network per action. Keep x402 mocked to sidestep for now.
- **Run↔job binding** — claim-latest is robust but races if two jobs dispatch back-to-back; for a
  single-dog demo this is fine. Manual override is the safety net.
- **China-floor constraint** (from `X402_ERC8004.md`): on-chain stablecoins/NFTs are a public-floor
  liability — the on-chain pieces are the private "global mode" on **testnet only**; never put a
  mainnet key on the venue laptop.
- **Wallet on a demo device** — judges/visitors may not have a wallet; pre-connect a burner, or keep
  feedback as an operator action so the reputation tick-up always lands.
