// Static secrets for a relay that can be exposed (--auth-file): robot keys
// bound to robot ids, viewer tokens, and topic source keys bound to source
// ids. Robots, viewers and topic providers present them in hello; /api/stats
// takes a viewer token as a bearer. All-or-nothing: a viewer token can do
// everything on every robot, and on every topic source when the relay runs
// with --open-topics. Compares are constant-time, and error messages name
// sections, ids, and names, never a secret.
import { timingSafeEqual } from "@std/crypto/timing-safe-equal";
import { MAX_TOKEN_LEN, MAX_TOPIC_SOURCES, topicSourceOk } from "@dimos/shared";

export const MIN_SECRET_LEN = 16;

const encoder = new TextEncoder();

/** Constant-time string equality. Different lengths are false at once (std's
 * timingSafeEqual throws on them), which leaks only the length, and the
 * secret's format gives that away anyway. */
export function secretsEqual(a: string, b: string): boolean {
  const x = encoder.encode(a);
  const y = encoder.encode(b);
  return x.byteLength === y.byteLength && timingSafeEqual(x, y);
}

export class Auth {
  readonly #robots: Map<string, string>;
  readonly #viewers: Map<string, string>;
  readonly #topicSources: Map<string, string>;

  constructor(
    robots: Map<string, string>,
    viewers: Map<string, string>,
    topicSources: Map<string, string> = new Map(),
  ) {
    this.#robots = robots;
    this.#viewers = viewers;
    this.#topicSources = topicSources;
  }

  /** True when `key` is the key bound to robot `id`. */
  robotKeyOk(id: string, key: string | undefined): boolean {
    const expected = this.#robots.get(id);
    return expected !== undefined && key !== undefined && secretsEqual(expected, key);
  }

  /** The viewer name owning `token`, or null. */
  viewerName(token: string | undefined): string | null {
    if (token === undefined) return null;
    for (const [name, expected] of this.#viewers) {
      if (secretsEqual(expected, token)) return name;
    }
    return null;
  }

  /** True when `key` is the key bound to topic source `source`. */
  topicSourceKeyOk(source: string, key: string | undefined): boolean {
    const expected = this.#topicSources.get(source);
    return expected !== undefined && key !== undefined && secretsEqual(expected, key);
  }

  /** An `Authorization: Bearer <viewer token>` header value. */
  bearerOk(header: string | null): boolean {
    const scheme = "Bearer ";
    if (header === null || !header.startsWith(scheme)) return false;
    return this.viewerName(header.slice(scheme.length)) !== null;
  }
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function section(
  data: Record<string, unknown>,
  key: "robots" | "viewers" | "topicSources",
): Map<string, string> {
  const what = key === "robots" ? "robot id" : key === "viewers" ? "viewer name" : "topic source";
  const value = data[key];
  if (!isRecord(value)) {
    throw new Error(`auth file: "${key}" must be an object of ${what} -> secret`);
  }
  const out = new Map<string, string>();
  for (const [name, secret] of Object.entries(value)) {
    if (name === "") throw new Error(`auth file: empty ${what} in "${key}"`);
    if (typeof secret !== "string" || secret.length < MIN_SECRET_LEN) {
      throw new Error(
        `auth file: ${key}["${name}"] must be a string of at least ${MIN_SECRET_LEN} characters`,
      );
    }
    if (secret.length > MAX_TOKEN_LEN) {
      throw new Error(
        `auth file: ${key}["${name}"] must be at most ${MAX_TOKEN_LEN} characters`,
      );
    }
    out.set(name, secret);
  }
  return out;
}

function topicSources(data: Record<string, unknown>): Map<string, string> {
  if (data.topicSources === undefined) return new Map();
  const sources = section(data, "topicSources");
  if (sources.size > MAX_TOPIC_SOURCES) throw new Error("auth file: too many topic sources");
  // The same rule the provider hello is checked against, so an id the file
  // accepts can always register.
  for (const name of sources.keys()) {
    if (!topicSourceOk(name)) {
      throw new Error(
        "auth file: topic source ids start with a letter or digit, then up to 63 letters, digits, _, . or -",
      );
    }
  }
  return sources;
}

/** Parses and validates the auth file text (see the file's shape in
 * docs/web/relay.md). A JSON error is reported without V8's message, which
 * quotes file text. */
export function parseAuthFile(text: string): Auth {
  let data: unknown;
  try {
    data = JSON.parse(text);
  } catch {
    throw new Error("auth file: not valid JSON");
  }
  if (!isRecord(data)) {
    throw new Error("auth file: the top level must be an object with robots and viewers");
  }
  const robots = section(data, "robots");
  const viewers = section(data, "viewers");
  const sources = topicSources(data);
  // One secret, one owner: a shared secret would let a robot act as a viewer
  // (or two robots swap identities) by accident.
  const owners = new Map<string, string>();
  for (
    const [key, entries] of [
      ["robots", robots],
      ["viewers", viewers],
      ["topicSources", sources],
    ] as const
  ) {
    for (const [name, secret] of entries) {
      const owner = `${key}["${name}"]`;
      const other = owners.get(secret);
      if (other !== undefined) throw new Error(`auth file: ${owner} reuses the secret of ${other}`);
      owners.set(secret, owner);
    }
  }
  return new Auth(robots, viewers, sources);
}

export async function loadAuthFile(path: string): Promise<Auth> {
  return parseAuthFile(await Deno.readTextFile(path));
}
