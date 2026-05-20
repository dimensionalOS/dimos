// LCM URL Parser (vendored from @dimos/lcm@0.2.0)

import type { ParsedUrl } from "./types.ts";

export const DEFAULT_MULTICAST_GROUP = "239.255.76.67";
export const DEFAULT_PORT = 7667;
export const DEFAULT_TTL = 0;

/**
 * Parse an LCM URL into its components.
 *
 * Supported formats:
 * - "udpm://239.255.76.67:7667?ttl=1"
 * - "udpm://239.255.76.67:7667"
 * - "udpm://" (uses defaults)
 * - "" (uses defaults)
 */
export function parseUrl(url?: string): ParsedUrl {
  if (!url || url === "" || url === "udpm://") {
    return {
      scheme: "udpm",
      host: DEFAULT_MULTICAST_GROUP,
      port: DEFAULT_PORT,
      ttl: DEFAULT_TTL,
    };
  }

  const match = url.match(/^(\w+):\/\/([^:/?#]+)?(?::(\d+))?(?:\?(.*))?$/);
  if (!match) {
    throw new Error(`Invalid LCM URL: ${url}`);
  }

  const [, scheme, host, portStr, queryStr] = match;

  if (scheme !== "udpm") {
    throw new Error(`Unsupported LCM scheme: ${scheme} (only "udpm" is supported)`);
  }

  const port = portStr ? parseInt(portStr, 10) : DEFAULT_PORT;

  // Parse query parameters
  let ttl = DEFAULT_TTL;
  let iface: string | undefined;

  if (queryStr) {
    const params = new URLSearchParams(queryStr);
    const ttlStr = params.get("ttl");
    if (ttlStr) {
      ttl = parseInt(ttlStr, 10);
    }
    iface = params.get("iface") ?? undefined;
  }

  return {
    scheme,
    host: host || DEFAULT_MULTICAST_GROUP,
    port,
    ttl,
    iface,
  };
}
