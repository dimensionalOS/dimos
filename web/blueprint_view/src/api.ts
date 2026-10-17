/// <reference types="vite/client" />
// The gateway's API, relative to the page: the view is at <base>/blueprint_view (Desktop's /dimos-gateway, or nothing
// on the gateway's own port), so "runs" is <base>/runs. vite's dev server proxies /dimos-gateway/ to the gateway, as Desktop does.
const BASE = import.meta.env.DEV ? "/dimos-gateway/" : "";

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
