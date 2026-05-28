import { type AgentService, agents, type Database } from "@robomoo/db";
import { newId } from "@robomoo/shared";

interface AgentSeed {
  slug: string;
  name: string;
  tagline: string;
  description: string;
  emoji: string;
  services: AgentService[];
  basePriceUsd: number;
  isReal: boolean;
  status: "live" | "coming_soon";
  capabilities: string[];
}

// The marketplace = the ERC-8004 registry. RoboDoc is the one real, hireable
// agent (the Go2 scan → segmentation → 3D-tour pipeline); the rest are fictional
// "coming soon" listings that make the registry feel populated for the demo.
const SEED: AgentSeed[] = [
  {
    slug: "robodoc",
    name: "RoboDoc",
    tagline: "Comes to your home, scans every room, hands you a 3D virtual tour.",
    description:
      "RoboDoc is an autonomous Unitree Go2 that you can hire to survey a space. It walks each room, photographs it from every angle, runs vision-language segmentation to label the objects it sees, and reconstructs a Gaussian-splat virtual tour you can walk through in the browser. Built on DimOS.",
    emoji: "🐕‍🦺",
    services: [
      {
        key: "room-scan",
        name: "Room Scan + 3D Tour",
        desc: "RoboDoc sweeps each room (360° panoramas), segments the objects it sees, and builds a walkable Gaussian-splat virtual tour.",
        priceUsd: 25,
        durationHint: "~10 min/room",
      },
    ],
    basePriceUsd: 25,
    isReal: true,
    status: "live",
    capabilities: ["room-scan", "segmentation", "gaussian-splat", "mapping"],
  },
  {
    slug: "patrol-pup",
    name: "Patrol Pup",
    tagline: "An autonomous security patrol that thinks out loud.",
    description:
      "Walks a set of waypoints, compares each scene to a learned baseline, and narrates anomalies — a door left open, an unfamiliar face, a misplaced package — logging each incident with a location and timestamp.",
    emoji: "🛡️",
    services: [
      {
        key: "patrol",
        name: "Night Patrol",
        desc: "Waypoint patrol with anomaly reasoning + incident log.",
        priceUsd: 40,
        durationHint: "per shift",
      },
    ],
    basePriceUsd: 40,
    isReal: false,
    status: "coming_soon",
    capabilities: ["patrol", "anomaly-detection", "spatial-memory"],
  },
  {
    slug: "fetch",
    name: "Fetch",
    tagline: "Tell it what you want; it finds it and brings it to you.",
    description:
      "Open-vocabulary object retrieval: describe an item or a person and Fetch navigates to it and delivers it. 'Bring me the red mug from the kitchen.'",
    emoji: "🦴",
    services: [
      {
        key: "fetch-item",
        name: "Fetch an item",
        desc: "Navigate to a described object/person and deliver it.",
        priceUsd: 8,
        durationHint: "per fetch",
      },
    ],
    basePriceUsd: 8,
    isReal: false,
    status: "coming_soon",
    capabilities: ["open-vocab-detection", "navigation", "delivery"],
  },
  {
    slug: "surveyor",
    name: "Surveyor",
    tagline: "Outdoor site mapping & inspection on four legs.",
    description:
      "Walks a construction site or facility, builds a 2D/3D map, and flags inspection points — cracks, leaks, blocked exits — into a structured report.",
    emoji: "📐",
    services: [
      {
        key: "site-survey",
        name: "Site Survey",
        desc: "Map a site and produce an inspection report.",
        priceUsd: 120,
        durationHint: "per site",
      },
    ],
    basePriceUsd: 120,
    isReal: false,
    status: "coming_soon",
    capabilities: ["mapping", "inspection", "reporting"],
  },
  {
    slug: "maestro",
    name: "Maestro",
    tagline: "A pack of robots choreographed as one performer.",
    description:
      "A conductor agent issues natural-language cues — 'form a line', 'wave down the row', 'scatter and regroup' — and each robot interprets them relative to its pose and neighbors. One behavior file, many dogs.",
    emoji: "🎭",
    services: [
      {
        key: "performance",
        name: "Live Performance",
        desc: "Choreographed multi-robot routine for an event.",
        priceUsd: 500,
        durationHint: "per show",
      },
    ],
    basePriceUsd: 500,
    isReal: false,
    status: "coming_soon",
    capabilities: ["swarm", "choreography", "coordination"],
  },
  {
    slug: "greeter",
    name: "Greeter",
    tagline: "A concierge that welcomes guests and gives guided tours.",
    description:
      "Greets visitors, answers questions about the space, and leads guided tours — narrating points of interest as it walks. A friendly front-of-house robot.",
    emoji: "🤝",
    services: [
      {
        key: "guided-tour",
        name: "Guided Tour",
        desc: "Lead a narrated tour of a venue.",
        priceUsd: 30,
        durationHint: "per hour",
      },
    ],
    basePriceUsd: 30,
    isReal: false,
    status: "coming_soon",
    capabilities: ["concierge", "tour-guide", "tts"],
  },
];

// Insert the seed agents if missing. Idempotent on `slug` — re-running on every
// deploy is a no-op, and editing an existing agent's metadata here won't clobber
// on-chain fields (agentId/agentWallet/registerTx) already written by a register.
export async function seedAgents(db: Database): Promise<void> {
  for (const a of SEED) {
    await db
      .insert(agents)
      .values({
        id: newId("agent"),
        slug: a.slug,
        name: a.name,
        tagline: a.tagline,
        description: a.description,
        emoji: a.emoji,
        services: a.services,
        basePriceUsd: a.basePriceUsd,
        isReal: a.isReal,
        status: a.status,
        capabilities: a.capabilities,
      })
      .onConflictDoNothing({ target: agents.slug });
  }
  console.log(`seeded ${SEED.length} marketplace agents (idempotent)`);
}
