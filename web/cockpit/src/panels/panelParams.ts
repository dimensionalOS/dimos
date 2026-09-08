// Role -> channel lookup for the cockpit panels whose manifest params carry
// the mapping (chat/navmap/control, see dimos/web/cockpit.py). The params are
// authoritative; the positional slot in spec.channels is the fallback for a
// bridge that predates the params, so the frontend never index-guesses when
// it does not have to.

import type { PanelSpec } from "@dimos/shared";

export function paramChannel(spec: PanelSpec, role: string, index: number): string | undefined {
  const named = spec.params[role];
  if (typeof named === "string" && named !== "") return named;
  return spec.channels[index];
}

export function readNumber(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

export function readString(v: unknown): string | null {
  return typeof v === "string" ? v : null;
}
