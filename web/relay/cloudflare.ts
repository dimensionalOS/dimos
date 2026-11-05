// Cloudflare Realtime SFU client (rtc.ts drives it). Error shapes: a non-2xx
// status, a 2xx whose body carries an errorCode, and per-track failures in
// tracks[].errorCode; 410 or a "disconnected" session_error means the session
// is gone for good and must be re-provisioned, never retried. Response bodies
// are never echoed into errors: an SDP carries ICE credentials.
import type { IceServer } from "@dimos/shared";

export interface RtcConfig {
  /** Realtime SFU app (dashboard: Realtime -> SFU): id and secret. */
  appId: string;
  appSecret: string;
  /** Optional TURN key (dashboard: Realtime -> TURN), both or neither.
   * Without it peers get STUN only and must reach the SFU over UDP. */
  turnKeyId?: string;
  turnToken?: string;
}

export const CF_API_BASE = "https://rtc.live.cloudflare.com/v1";
export const STUN_ONLY: IceServer[] = [{ urls: ["stun:stun.cloudflare.com:3478"] }];
/** TURN credential lifetime in seconds; the hub refreshes at half-life. */
export const TURN_TTL_S = 7200;
/** Cloudflare collects a track after 30 s without media. A published track
 * nobody watches (the bridge encodes lazily) must then be declared again. */
export const TRACK_GC_MS = 30_000;
const SESSION_TIMEOUT_MS = 10_000;
const TRACKS_TIMEOUT_MS = 30_000;

export interface TrackRef {
  location: "local" | "remote";
  trackName: string;
  /** Local only: the transceiver (m-section) in the session's own SDP. */
  mid?: string;
  /** Remote only: the session publishing the track. */
  sessionId?: string;
}

export interface TrackResult {
  mid?: string;
  trackName?: string;
  errorCode?: string;
  errorDescription?: string;
}

export interface TracksResponse {
  tracks: TrackResult[];
  /** The SFU's offer (after a pull), when the call changed the SDP. */
  sdp: string | null;
}

export class CloudflareError extends Error {
  /** `sessionId` is the session the failed call addressed (null for
   * sessions/new and the TURN mint). */
  constructor(
    readonly status: number,
    readonly code: string | null,
    message: string,
    readonly sessionId: string | null = null,
  ) {
    super(message);
    this.name = "CloudflareError";
  }
}

/** The session no longer exists at the SFU: re-provision, never retry. */
export class SessionGoneError extends CloudflareError {
  constructor(
    status: number,
    code: string | null,
    message: string,
    sessionId: string | null = null,
  ) {
    super(status, code, message, sessionId);
    this.name = "SessionGoneError";
  }
}

export type FetchFn = typeof fetch;

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function isIceServer(value: unknown): value is IceServer {
  return isRecord(value) && Array.isArray(value.urls) &&
    value.urls.every((u) => typeof u === "string") &&
    (value.username === undefined || typeof value.username === "string") &&
    (value.credential === undefined || typeof value.credential === "string");
}

/** The per-track results of a tracks/new or tracks/close body. */
function parseTrackResults(body: Record<string, unknown>): TrackResult[] {
  const results: TrackResult[] = [];
  for (const t of Array.isArray(body.tracks) ? body.tracks : []) {
    if (!isRecord(t)) continue;
    const result: TrackResult = {};
    if (typeof t.mid === "string") result.mid = t.mid;
    if (typeof t.trackName === "string") result.trackName = t.trackName;
    if (typeof t.errorCode === "string" && t.errorCode !== "") {
      result.errorCode = t.errorCode;
      if (typeof t.errorDescription === "string") result.errorDescription = t.errorDescription;
    }
    results.push(result);
  }
  return results;
}

export class CloudflareClient {
  readonly #config: RtcConfig;
  readonly #fetch: FetchFn;
  readonly #base: string;

  /** `fetchFn` and `base` are test seams. */
  constructor(config: RtcConfig, fetchFn: FetchFn = fetch, base: string = CF_API_BASE) {
    this.#config = config;
    this.#fetch = fetchFn;
    this.#base = base;
  }

  get hasTurn(): boolean {
    return this.#config.turnKeyId !== undefined && this.#config.turnToken !== undefined;
  }

  async newSession(offerSdp: string): Promise<{ sessionId: string; answerSdp: string }> {
    const body = await this.#call(
      "POST",
      `/apps/${this.#config.appId}/sessions/new`,
      { sessionDescription: { type: "offer", sdp: offerSdp } },
      SESSION_TIMEOUT_MS,
    );
    const sessionId = body.sessionId;
    const description = body.sessionDescription;
    const sdp = isRecord(description) ? description.sdp : undefined;
    if (typeof sessionId !== "string" || typeof sdp !== "string") {
      throw new CloudflareError(200, null, "sessions/new: unexpected response shape");
    }
    return { sessionId, answerSdp: sdp };
  }

  /** A pull returns the SFU's offer, which the peer answers through
   * renegotiate(). Per-track failures are returned, not thrown. */
  async addTracks(sessionId: string, tracks: TrackRef[]): Promise<TracksResponse> {
    const body = await this.#call(
      "POST",
      `/apps/${this.#config.appId}/sessions/${sessionId}/tracks/new`,
      { tracks },
      TRACKS_TIMEOUT_MS,
      sessionId,
    );
    const description = body.sessionDescription;
    const sdp = isRecord(description) && typeof description.sdp === "string"
      ? description.sdp
      : null;
    return { tracks: parseTrackResults(body), sdp };
  }

  /** force: no renegotiation, the peer's transceivers stay, inactive. An
   * HTTP 200 can still fail single mids: those are returned, not thrown. */
  async closeTracks(sessionId: string, mids: string[]): Promise<TrackResult[]> {
    const body = await this.#call(
      "PUT",
      `/apps/${this.#config.appId}/sessions/${sessionId}/tracks/close`,
      { tracks: mids.map((mid) => ({ mid })), force: true },
      TRACKS_TIMEOUT_MS,
      sessionId,
    );
    return parseTrackResults(body);
  }

  async renegotiate(sessionId: string, answerSdp: string): Promise<void> {
    await this.#call(
      "PUT",
      `/apps/${this.#config.appId}/sessions/${sessionId}/renegotiate`,
      { sessionDescription: { type: "answer", sdp: answerSdp } },
      TRACKS_TIMEOUT_MS,
      sessionId,
    );
  }

  async iceServers(): Promise<IceServer[]> {
    const { turnKeyId, turnToken } = this.#config;
    if (turnKeyId === undefined || turnToken === undefined) return STUN_ONLY;
    const res = await this.#fetch(
      `${this.#base}/turn/keys/${turnKeyId}/credentials/generate-ice-servers`,
      {
        method: "POST",
        headers: { authorization: `Bearer ${turnToken}`, "content-type": "application/json" },
        body: JSON.stringify({ ttl: TURN_TTL_S }),
        signal: AbortSignal.timeout(SESSION_TIMEOUT_MS),
      },
    );
    if (!res.ok) {
      throw new CloudflareError(res.status, null, `turn credentials: HTTP ${res.status}`);
    }
    const body: unknown = await res.json();
    const servers = isRecord(body) ? body.iceServers : undefined;
    if (!Array.isArray(servers) || servers.length === 0 || !servers.every(isIceServer)) {
      throw new CloudflareError(res.status, null, "turn credentials: unexpected response shape");
    }
    return servers;
  }

  async #call(
    method: "POST" | "PUT",
    path: string,
    body: unknown,
    timeoutMs: number,
    sessionId: string | null = null,
  ): Promise<Record<string, unknown>> {
    const res = await this.#fetch(`${this.#base}${path}`, {
      method,
      headers: {
        authorization: `Bearer ${this.#config.appSecret}`,
        "content-type": "application/json",
      },
      body: JSON.stringify(body),
      signal: AbortSignal.timeout(timeoutMs),
    });
    const text = await res.text();
    if (res.status === 410) {
      throw new SessionGoneError(410, null, `${path}: session gone`, sessionId);
    }
    if (!res.ok) {
      throw new CloudflareError(res.status, null, `${path}: HTTP ${res.status}`, sessionId);
    }
    let data: unknown;
    try {
      data = JSON.parse(text);
    } catch {
      throw new CloudflareError(res.status, null, `${path}: non-JSON response`, sessionId);
    }
    if (!isRecord(data)) {
      throw new CloudflareError(res.status, null, `${path}: non-object response`, sessionId);
    }
    const code = data.errorCode;
    if (typeof code === "string" && code !== "") {
      const description = typeof data.errorDescription === "string" ? data.errorDescription : "";
      const message = `${path}: ${code}: ${description.slice(0, 200)}`;
      if (code === "session_error" && description.toLowerCase().includes("disconnected")) {
        throw new SessionGoneError(res.status, code, message, sessionId);
      }
      throw new CloudflareError(res.status, code, message, sessionId);
    }
    return data;
  }
}

const RTC_FILE_KEYS = new Set(["appId", "appSecret", "turnKeyId", "turnToken"]);

/** Errors name keys, never values: V8's JSON error message quotes file text. */
export function parseRtcFile(text: string): RtcConfig {
  let data: unknown;
  try {
    data = JSON.parse(text);
  } catch {
    throw new Error("rtc file: not valid JSON");
  }
  if (!isRecord(data)) {
    throw new Error("rtc file: the top level must be an object with appId and appSecret");
  }
  for (const key of Object.keys(data)) {
    if (!RTC_FILE_KEYS.has(key)) throw new Error(`rtc file: unknown key "${key}"`);
  }
  const str = (key: string): string => {
    const value = data[key];
    if (typeof value !== "string" || value === "") {
      throw new Error(`rtc file: "${key}" must be a non-empty string`);
    }
    return value;
  };
  const config: RtcConfig = { appId: str("appId"), appSecret: str("appSecret") };
  const hasKey = "turnKeyId" in data;
  if (hasKey !== "turnToken" in data) {
    throw new Error("rtc file: turnKeyId and turnToken must be given together");
  }
  if (hasKey) {
    config.turnKeyId = str("turnKeyId");
    config.turnToken = str("turnToken");
  }
  return config;
}

export async function loadRtcFile(path: string): Promise<RtcConfig> {
  return parseRtcFile(await Deno.readTextFile(path));
}
