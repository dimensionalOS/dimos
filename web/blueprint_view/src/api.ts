/// <reference types="vite/client" />
// Paths are the gateway's, relative to <base>/blueprint_view ("active" → <base>/active); "../…" is another gateway (Desktop's at ../desktop/), "../../apps/…" an app.
const BASE = import.meta.env.DEV ? "/gateways/dimos/" : "";

export class ApiError extends Error {
  constructor(message: string, readonly status: number) {
    super(message);
  }
}

export async function getJson<T = unknown>(path: string, options?: RequestInit): Promise<T> {
  const response = await fetch(path.startsWith("../") ? path : BASE + path, options);
  const data = await response.json().catch(() => null);
  if (!response.ok) {
    throw new ApiError(data?.error ?? `${response.status} ${response.statusText}`, response.status);
  }
  return data as T;
}

export const send = <T = unknown>(method: string, path: string, body?: unknown): Promise<T> =>
  getJson<T>(path, {
    method,
    headers: { "content-type": "application/json" },
    body: JSON.stringify(body ?? {}),
  });

export const errorText = (error: unknown): string =>
  String((error as Error | null)?.message ?? error);
