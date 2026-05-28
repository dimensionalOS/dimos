import {
  createPublicClient,
  createWalletClient,
  custom,
  type EIP1193Provider,
  http,
  parseEventLogs,
  stringToHex,
} from "viem";
import { sepolia } from "viem/chains";

// ─── ERC-8004 (Trustless Agents) on Ethereum Sepolia ────────────────────────
//
// Identity Registry is verified-deployed on Sepolia (reference impl
// `ChaosChain/trustless-agents-erc-ri`). The Reputation Registry address must be
// supplied via NEXT_PUBLIC_ERC8004_REPUTATION_ADDRESS — see PLAN.md / the PR
// description. Every on-chain call here is defensive: reads return null on
// failure (the UI falls back to job-derived stats) and the demo never breaks if
// the chain is unreachable or the address is unset.

export const SEPOLIA_CHAIN_ID = 11155111;

export const IDENTITY_REGISTRY = (process.env
  .NEXT_PUBLIC_ERC8004_IDENTITY_ADDRESS ??
  "0xf66e7CBdAE1Cb710fee7732E4e1f173624e137A7") as `0x${string}`;

// Empty string = not configured → on-chain reputation reads/writes are skipped.
export const REPUTATION_REGISTRY = (process.env
  .NEXT_PUBLIC_ERC8004_REPUTATION_ADDRESS ?? "") as string;

export const reputationConfigured = REPUTATION_REGISTRY.length > 0;

const ZERO_TAG = `0x${"00".repeat(32)}` as `0x${string}`;
const TASK_RATING_TAG = stringToHex("taskRating", { size: 32 });

export const IDENTITY_ABI = [
  {
    type: "function",
    name: "register",
    stateMutability: "nonpayable",
    inputs: [{ name: "agentURI", type: "string" }],
    outputs: [{ name: "agentId", type: "uint256" }],
  },
  {
    type: "function",
    name: "setAgentWallet",
    stateMutability: "nonpayable",
    inputs: [
      { name: "agentId", type: "uint256" },
      { name: "wallet", type: "address" },
      { name: "deadline", type: "uint256" },
      { name: "signature", type: "bytes" },
    ],
    outputs: [],
  },
  {
    type: "event",
    name: "Transfer",
    inputs: [
      { name: "from", type: "address", indexed: true },
      { name: "to", type: "address", indexed: true },
      { name: "tokenId", type: "uint256", indexed: true },
    ],
  },
] as const;

export const REPUTATION_ABI = [
  {
    type: "function",
    name: "giveFeedback",
    stateMutability: "nonpayable",
    inputs: [
      { name: "agentId", type: "uint256" },
      { name: "value", type: "int128" },
      { name: "valueDecimals", type: "uint8" },
      { name: "tag1", type: "bytes32" },
      { name: "tag2", type: "bytes32" },
      { name: "endpoint", type: "string" },
    ],
    outputs: [],
  },
  {
    type: "function",
    name: "getSummary",
    stateMutability: "view",
    inputs: [
      { name: "agentId", type: "uint256" },
      { name: "clients", type: "address[]" },
      { name: "tag1", type: "bytes32" },
      { name: "tag2", type: "bytes32" },
    ],
    outputs: [
      { name: "count", type: "uint64" },
      { name: "summaryValue", type: "int128" },
      { name: "decimals", type: "uint8" },
    ],
  },
] as const;

export const publicClient = createPublicClient({
  chain: sepolia,
  transport: http(process.env.NEXT_PUBLIC_SEPOLIA_RPC_URL || undefined),
});

function walletClient() {
  if (typeof window === "undefined" || !window.ethereum) {
    throw new Error("No browser wallet found");
  }
  return createWalletClient({
    chain: sepolia,
    transport: custom(window.ethereum as unknown as EIP1193Provider),
  });
}

// Register an agent on the Identity Registry. Returns the tx hash and the minted
// agentId (parsed from the ERC-721 Transfer event; null if it couldn't be read).
export async function registerAgentOnchain(
  account: `0x${string}`,
  agentURI: string,
): Promise<{ txHash: `0x${string}`; agentId: string | null }> {
  const wallet = walletClient();
  const txHash = await wallet.writeContract({
    account,
    address: IDENTITY_REGISTRY,
    abi: IDENTITY_ABI,
    functionName: "register",
    args: [agentURI],
  });
  const receipt = await publicClient.waitForTransactionReceipt({ hash: txHash });
  let agentId: string | null = null;
  try {
    const logs = parseEventLogs({
      abi: IDENTITY_ABI,
      logs: receipt.logs,
      eventName: "Transfer",
    });
    const minted = logs.find((l) => l.args && "tokenId" in l.args);
    if (minted) agentId = (minted.args.tokenId as bigint).toString();
  } catch {
    /* couldn't decode — caller can read it from the tx on etherscan */
  }
  return { txHash, agentId };
}

// Live reputation read. Returns null when not configured or on any failure, so
// callers transparently fall back to off-chain stats.
export async function getReputationSummary(
  agentId: string,
): Promise<{ count: number; avg: number } | null> {
  if (!reputationConfigured) return null;
  try {
    const res = (await publicClient.readContract({
      address: REPUTATION_REGISTRY as `0x${string}`,
      abi: REPUTATION_ABI,
      functionName: "getSummary",
      args: [BigInt(agentId), [], ZERO_TAG, ZERO_TAG],
    })) as readonly [bigint, bigint, number];
    const count = Number(res[0]);
    if (count === 0) return { count: 0, avg: 0 };
    const decimals = Number(res[2]);
    const avg = Number(res[1]) / count / 10 ** decimals;
    return { count, avg };
  } catch {
    return null;
  }
}

// Submit a 1..5 star rating for a completed job. Throws if the reputation
// registry isn't configured (caller still persists the rating off-chain).
export async function giveFeedbackOnchain(
  account: `0x${string}`,
  agentId: string,
  rating: number,
): Promise<`0x${string}`> {
  if (!reputationConfigured) {
    throw new Error("Reputation registry not configured");
  }
  const wallet = walletClient();
  return wallet.writeContract({
    account,
    address: REPUTATION_REGISTRY as `0x${string}`,
    abi: REPUTATION_ABI,
    functionName: "giveFeedback",
    args: [BigInt(agentId), BigInt(rating), 0, TASK_RATING_TAG, ZERO_TAG, ""],
  });
}

declare global {
  interface Window {
    ethereum?: {
      request: (args: { method: string; params?: unknown[] }) => Promise<unknown>;
      on?: (event: string, handler: (...args: unknown[]) => void) => void;
      removeListener?: (
        event: string,
        handler: (...args: unknown[]) => void,
      ) => void;
    };
  }
}
