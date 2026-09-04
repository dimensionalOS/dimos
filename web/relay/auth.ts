// Static secrets for a relay that can be exposed (--auth-file): robot keys
// bound to robot ids, and viewer tokens. Robots and viewers present them in
// hello; /api/stats takes a viewer token as a bearer. All-or-nothing: a
// viewer token can do everything on every robot. Compares are constant-time,
// and error messages name sections, ids, and names, never a secret.
import { timingSafeEqual } from "@std/crypto/timing-safe-equal";
import { MAX_TOKEN_LEN } from "@dimos/shared";

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

  constructor(robots: Map<string, string>, viewers: Map<string, string>) {
    this.#robots = robots;
    this.#viewers = viewers;
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

function section(data: Record<string, unknown>, key: "robots" | "viewers"): Map<string, string> {
  const what = key === "robots" ? "robot id" : "viewer name";
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

/** Parses and validates the auth file text (see the file's shape in
 * web/README.md). A JSON error is reported without V8's message, which
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
  // One secret, one owner: a shared secret would let a robot act as a viewer
  // (or two robots swap identities) by accident.
  const owners = new Map<string, string>();
  for (const [key, entries] of [["robots", robots], ["viewers", viewers]] as const) {
    for (const [name, secret] of entries) {
      const owner = `${key}["${name}"]`;
      const other = owners.get(secret);
      if (other !== undefined) throw new Error(`auth file: ${owner} reuses the secret of ${other}`);
      owners.set(secret, owner);
    }
  }
  return new Auth(robots, viewers);
}

export async function loadAuthFile(path: string): Promise<Auth> {
  return parseAuthFile(await Deno.readTextFile(path));
}
