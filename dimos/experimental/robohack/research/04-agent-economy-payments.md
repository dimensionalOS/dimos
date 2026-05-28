# Agent-Economy Payments for a DimOS Robot (x402 / MPP / ERC-8004)

> Research note for the DIMENSIONAL/DimOS hackathon (Shanghai, May 26–28 2026; Unitree Go2).
> Goal: establish the **technical reality** of making the Go2 an *economic agent* — it pays for
> services it consumes and/or charges for services it provides, with optional on-chain identity +
> reputation — and find prior art at the robotics × crypto × machine-economy intersection.
> All code shapes below are taken from the **actual current repos** (May 2026), not from memory.

---

## 0. TL;DR for the build

- **x402 is the right rail.** It is HTTP-402 + on-chain USDC, has a **first-class Python SDK** (`pip install x402`),
  ships a **FastAPI middleware** and — critically — a **native MCP integration with a `@paid` decorator**.
  DimOS Skills are already MCP tools, so an x402 paywall maps onto a DimOS Skill with *one decorator*.
- **Testnet is permissionless and free.** Base Sepolia (`eip155:84532`), public facilitator at
  `https://x402.org/facilitator`, free testnet USDC from Circle / CDP / LearnWeb3 faucets. **No merchant
  credentials needed** — unlike Alipay this can ship tonight.
- **ERC-8004 is live and our home turf.** Reference contracts are deployed on Sepolia; an agent registers an
  ERC-721 identity (`register`) and earns reputation (`giveFeedback` / `getSummary`). Wire it in as the *trust
  spine*, not the long pole.
- **Prior art exists and is almost exactly our demo:** **OpenMind + Circle (Feb 2026)** showed a robot dog
  ("Bits") that detected low battery, found a charger, plugged in, and **autonomously paid for electricity in
  USDC via an x402 module** on its OM1 OS. We are doing the DimOS-native version of this, plus the *inverse*
  (robot charges per task) and the 8004 trust layer they didn't show.

---

## 1. x402 — the technical reality (verified against `coinbase/x402`)

### What it is
x402 revives HTTP status **402 Payment Required**. Flow:

1. Client `GET`s a resource.
2. Server replies `402` with a `PAYMENT-REQUIRED` header describing accepted payment options (network, asset, price, pay-to address).
3. Client signs a stablecoin payment payload (EIP-3009 transfer authorization for USDC) and retries with a `PAYMENT-SIGNATURE` header.
4. Server hands the payload to a **facilitator**, which verifies and settles on-chain, then returns the resource (+ a settlement receipt header).

The **facilitator** abstracts away RPC/gas/chain plumbing. For testnet the public one is `https://x402.org/facilitator`; CDP runs a hosted one for mainnet (Base, Polygon, Arbitrum, World, Solana; 1,000 free tx/mo).

### The Python SDK is real and current
`coinbase/x402` (now under the **x402 Foundation**, a Linux Foundation effort) has a `python/` package published as `pip install x402`, with submodules: `client`, `server`, `facilitator`, `http` (incl. `http/middleware/fastapi`), `mechanisms/evm` + `mechanisms/svm`, and an **`mcp/`** package. Examples live in `examples/python/{servers,clients}` and include `fastapi`, `flask`, **`mcp`**, and `mcp-chatbot`.

### Server seam — a paid FastAPI endpoint (verbatim shape from `examples/python/servers/fastapi/main.py`)

```python
from fastapi import FastAPI
from x402.http import FacilitatorConfig, HTTPFacilitatorClient, PaymentOption
from x402.http.middleware.fastapi import PaymentMiddlewareASGI
from x402.http.types import RouteConfig
from x402.mechanisms.evm.exact import ExactEvmServerScheme
from x402.schemas import Network
from x402.server import x402ResourceServer

EVM_NETWORK: Network = "eip155:84532"                 # Base Sepolia
FACILITATOR_URL = "https://x402.org/facilitator"      # public testnet facilitator
EVM_ADDRESS = "0xYourRobotWallet"                     # where the robot gets paid

app = FastAPI()
server = x402ResourceServer(HTTPFacilitatorClient(FacilitatorConfig(url=FACILITATOR_URL)))
server.register(EVM_NETWORK, ExactEvmServerScheme())

routes = {
    "POST /fetch": RouteConfig(accepts=[PaymentOption(
        scheme="exact", pay_to=EVM_ADDRESS, price="$0.10", network=EVM_NETWORK)],
        description="Go2 fetches an object and brings it to you"),
}
app.add_middleware(PaymentMiddlewareASGI, routes=routes, server=server)

@app.post("/fetch")
async def fetch(job: FetchJob):
    result = await dimos.call_skill("NavigateToObject", target=job.target)  # <-- DimOS Skill
    return {"status": "delivered", "evidence_uri": result.frame_url}
```

That is the **entire seam for "robot gets paid per task"**: the paid endpoint is a thin FastAPI wrapper that, once payment settles, calls a DimOS Skill over the MCP server (`http://localhost:9990/mcp`) or via `dimos.call_skill`. On Base Sepolia USDC is at `0x036CbD53842c5426634e7929541eC2318f3dCF7e`.

### Client seam — the robot *pays* (verbatim shape from `examples/python/clients/httpx/main.py`)

```python
from eth_account import Account
from x402 import x402Client
from x402.http.clients import x402HttpxClient
from x402.mechanisms.evm import EthAccountSigner
from x402.mechanisms.evm.exact.register import register_exact_evm_client

client = x402Client()
account = Account.from_key(os.environ["ROBOT_PRIVATE_KEY"])
register_exact_evm_client(client, EthAccountSigner(account))

async with x402HttpxClient(client) as http:           # auto-handles 402 → sign → retry
    resp = await http.get(f"{PAID_API}/vision/describe")   # robot pays for a VLM call
    await resp.aread()
```

The client transparently intercepts the `402`, signs from the robot's key, retries — **the calling code looks like a normal HTTP GET**. This is "robot pays per API call" with ~3 added lines.

### The MCP integration — *this is the DimOS killer seam*
`python/x402/mcp` exposes `create_payment_wrapper(...)` returning a **`@paid` decorator** that wraps an MCP tool handler so calling the tool *requires* an x402 payment; and on the client side `create_x402_mcp_client_from_config(...)` / `wrap_mcp_client_with_payment(...)` make an MCP client auto-pay for paid tools (with an `on_payment_requested` approval hook).

Because **DimOS Skills are surfaced to the agent as MCP tools**, this means:
- **Robot-as-seller:** decorate a DimOS Skill's MCP handler with `@paid` → the Skill is now paywalled; any agent must pay USDC to invoke `fetch()` / `patrol()` / `deliver()`.
- **Robot-as-buyer:** wrap the DimOS agent's *outbound* MCP client so when it calls a paid external tool (a premium perception/VLM/map service), it auto-pays from the robot's wallet.

```python
from x402.mcp import create_payment_wrapper, PaymentWrapperConfig
paid = create_payment_wrapper(resource_server,
        PaymentWrapperConfig(accepts=accepts))   # accepts built for eip155:84532, $0.10

@mcp_server.tool("deliver", "Go2 delivers an item to a person", schema)
@paid
def deliver(args, ctx):
    return run_dimos_skill("NavigateToObject", **args)
```

---

## 2. MPP (Stripe + Tempo) — the alternative rail

**MPP (Machine Payments Protocol)**, co-authored by **Stripe and Tempo**, is the same HTTP-402 idea but
**method-agnostic** (crypto on-chain *or* fiat card/wallet via Shared Payment Tokens) and **back-compatible
with x402**. Docs: `docs.stripe.com/payments/machine` and `/payments/machine/mpp`. Its standout feature is
**Sessions**: a client pre-authorizes a *budget*, then streams many micropayments against it — a natural fit
for a robot doing dozens of tiny paid actions in one run.

**48h reality:** doable in **Stripe test mode** with the PaymentIntents API, but the crypto leg is gated to
US legal entities and the ecosystem/tooling is newer and smaller than x402's. **Recommendation: skip for the
hackathon, mention as the "production, multi-rail" answer.** x402 gives the same 402 story with zero account
setup. If a judge asks "what about fiat?", the answer is "MPP — same protocol shape, swap the rail."

---

## 3. ERC-8004 — the trust spine (verified against `ChaosChain/trustless-agents-erc-ri`)

ERC-8004 ("Trustless Agents", **mainnet live Jan 29 2026**) is **not payments** — it is the on-chain trust
layer that makes payments *meaningful*: who is this agent, and is it any good. Three registries:

| Registry | Purpose | Key calls |
|---|---|---|
| **Identity** (ERC-721) | Agent = an NFT with a metadata URI + verified payment wallet | `register(agentURI) → agentId`, `setAgentWallet(agentId, wallet, deadline, sig)` |
| **Reputation** | Signed fixed-point feedback per agent | `giveFeedback(agentId, value:int128, valueDecimals:uint8, tag1, tag2, endpoint, ...)`, `getSummary(agentId, clients[], tag1, tag2) → (count, summaryValue, decimals)` |
| **Validation** | Independent verification of work (URI evidence + `responseHash`) | validator hooks |

Reference impl: Solidity 0.8.19, Foundry, **74/74 tests passing**, deployed & verified on **Ethereum Sepolia**
(Identity Registry `0xf66e7CBdAE1Cb710fee7732E4e1f173624e137A7`; other testnets — Base Sepolia, Linea Sepolia,
Hedera — also have reference deployments). Reputation values are flexible signed fixed-point: e.g. `uptime`
99.77% = `value=9977, decimals=2`; a 5-star task rating = `value=5, decimals=0`.

### Minimal 8004 demo loop (robot earns reputation per completed task)
1. **One-time:** robot calls `register(agentURI)` where `agentURI` (IPFS/HTTP JSON) describes the Go2 agent + its Skills → gets `agentId` (an NFT). `setAgentWallet` binds the same wallet it pays/receives with on x402.
2. **Per task:** customer pays via x402 → robot executes the DimOS Skill → on success the customer (or a validator) calls `giveFeedback(agentId, value=5, decimals=0, tag1="taskRating", endpoint="deliver", ...)`.
3. **Demo readout:** dashboard calls `getSummary(agentId, [], "taskRating", "")` → live "robot reputation: 4.8 over 12 jobs" counter that ticks up on stage. This is the single most legible on-chain artifact for a 90s video.

We have **prior ERC-8004 experience** — treat the registration + feedback wiring as a head start, write it Day-1, and make the reputation counter the visual spine.

---

## 4. Prior art — robotics × crypto × machine economy

- **OpenMind + Circle (Feb 2026) — the closest prior art.** Robot dog "Bits" detected low battery, navigated to a
  charger, plugged in, and **autonomously paid for electricity in USDC via an x402 module** on OpenMind's OM1 OS;
  Circle supplied USDC rails (nanopayments to $0.000001, zero gas). This validates the *exact* narrative — and
  gives us the differentiator: we do it **DimOS-native**, add the **inverse** (robot *charges* per task), and add
  the **ERC-8004 identity + reputation** layer OpenMind didn't show.
- **Scale signal for the pitch:** x402 has processed **$600M+** volume / **~500k active AI wallets**; agents moved
  **$73M across 176M tx** May 2025–Apr 2026. The machine economy is a real, not hypothetical, narrative.
- **Amazon enabling AI bots to pay in USDC via Coinbase x402** — agentic commerce is going mainstream.
- **Cloudflare** runs an x402 facilitator and co-founded the x402 Foundation; **x402 on Stellar/Solana** ports exist.
- **Adjacent agent-commerce ecosystems** (mention, don't build): **Coinbase AgentKit** (wallet + onchain tools for
  agents), **Skyfire**, **Nevermined**, **Catena**, **Virtuals**, **Olas/Autonolas** (autonomous agent services).
  None are needed for our demo; x402 + 8004 is the lean stack.

---

## 5. The recommended architecture (lowest-risk, most demo-able)

**"Pay-the-robot-dog in USDC, with an on-chain reputation that ticks up live."**

```
 Customer / judge phone (x402 client, browser or tiny CLI)
        │  POST /jobs/fetch   (HTTP 402 → sign USDC → retry)
        ▼
 FastAPI gateway  ── x402 PaymentMiddlewareASGI (Base Sepolia, x402.org/facilitator)
        │  on settle: call DimOS Skill over MCP (localhost:9990/mcp)
        ▼
 DimOS agent  ── Skill: fetch()/deliver()/patrol()  ── Unitree Go2 (WebRTC/ROS2)
        │  on success
        ▼
 ERC-8004 ReputationRegistry.giveFeedback(agentId, 5, …)   (Sepolia)
        ▲
 Dashboard polls getSummary(agentId) → "Reputation: 4.8 ★ over N jobs"  (the hero visual)
```

**Why this wins:** permissionless (ships tonight, no creds), the payment + the reputation are both *real
on-chain artifacts* a judge can verify on a block explorer, and the physical action (dog does a thing) is the
emotional payoff. It plugs straight into **Direction A** (the paywalled Skill is `patrol`/`fetch`) and naturally
extends to **Direction B** (each dog has its own 8004 identity; dogs pay each other for subtasks — the
"trustless swarm economy," our most ambitious swing).

**Optional inverse for extra wow:** in the same demo, the robot *spends* — before fetching, its agent pays
$0.01 via x402 for a "premium vision" API call (which can be your *own* second FastAPI paid endpoint). Now the
robot both **earns and spends** on stage: a closed economic loop.

---

## 6. Pitfalls & what to mock vs. make real

**Make real (these are cheap and judge-verifiable):**
- The **x402 402→pay→settle** round trip on Base Sepolia. It's the whole point and it works with free testnet USDC.
- The **ERC-8004 `register` + `giveFeedback` + `getSummary`** calls on Sepolia. On-chain reputation counter = the hero artifact.

**Mock / pre-stage (for a reliable 90 seconds):**
- **Testnet flakiness / latency.** Base Sepolia confirmations are usually seconds but can spike. **Don't block the
  robot's physical action on confirmation.** Optimistically execute the Skill the instant the facilitator returns
  `verify=ok`, and let settlement finalize in the background; poll the explorer link for the dashboard. Pre-fund
  wallets and **pre-warm** one tx before filming so RPC/nonce is hot.
- **Wallet/key management.** Use throwaway burner keys in env vars for the demo; **never** put a mainnet key on the
  venue laptop. Pre-fund: testnet ETH (gas) from CDP/Base faucet (0.1/24h) **and** testnet USDC from
  `faucet.circle.com` / CDP / LearnWeb3 *the day before* — faucets rate-limit and the venue Wi-Fi will fight you.
- **The facilitator dependency.** `x402.org/facilitator` is a public service; if it's down mid-demo you're stuck.
  Mitigation: have a **mock facilitator** (the SDK lets you point `FacilitatorConfig.url` at a local stub that
  always returns `valid/settled`) behind an env flag, so a network hiccup can't kill the 90s run.
- **8004 write latency.** `giveFeedback` is a Sepolia tx (seconds). Fire it async *after* the dog finishes; the
  dashboard can show an optimistic "+1 job" immediately and reconcile when the tx confirms.
- **Don't over-scope MPP/Alipay.** One rail, end-to-end, on testnet. Mention the others verbally.

**Robustness pattern to demo "graceful failure" (hire signal):** if payment verification fails, the Skill should
*refuse the job politely* ("payment not received, standing by") rather than crash — judges explicitly reward
plan/execute/**fail/recover**.

---

## Sources
- [coinbase/x402 (x402 Foundation) repo](https://github.com/coinbase/x402) — `python/x402/`, `examples/python/{servers/fastapi,clients/httpx,servers/mcp}`, `python/x402/mcp` (`@paid` decorator)
- [x402 docs (CDP)](https://docs.cdp.coinbase.com/x402/welcome) · [x402 explainer guide](https://simplescraper.io/blog/x402-payment-protocol) · [awesome-x402](https://github.com/xpaysh/awesome-x402)
- [Cloudflare x402 Foundation launch](https://blog.cloudflare.com/x402/) · [Cloudflare Agents x402 docs](https://developers.cloudflare.com/agents/agentic-payments/x402/)
- [Stripe MPP docs](https://docs.stripe.com/payments/machine/mpp) · [Stripe MPP announcement](https://stripe.com/blog/machine-payments-protocol) · [Cloudflare Agents MPP docs](https://developers.cloudflare.com/agents/agentic-payments/mpp/)
- [EIP-8004](https://eips.ethereum.org/EIPS/eip-8004) · [ChaosChain reference impl](https://github.com/ChaosChain/trustless-agents-erc-ri) · [erc-8004/erc-8004-contracts](https://github.com/erc-8004/erc-8004-contracts) · [awesome-erc8004](https://github.com/sudeepb02/awesome-erc8004)
- [OpenMind + Circle robot USDC payments (Feb 2026)](https://blockeden.xyz/blog/2026/03/04/openmind-machine-economy-usdc-robot-payments/) · [Circle M2M micropayments + USDC](https://www.circle.com/blog/enabling-machine-to-machine-micropayments-with-gateway-and-usdc) · [Amazon AI bots pay USDC via x402](https://crypto.news/amazon-lets-ai-bots-pay-in-usdc-via-coinbase-x402/)
- Faucets: [Circle testnet faucet](https://faucet.circle.com/) · [Base network faucets](https://docs.base.org/base-chain/network-information/network-faucets) · [LearnWeb3 Base Sepolia USDC](https://learnweb3.io/faucets/base_sepolia_usdc/)
</content>
</invoke>
