import { assert, assertEquals, assertFalse, assertThrows } from "@std/assert";
import { MAX_TOKEN_LEN } from "@dimos/shared";
import { Auth, MIN_SECRET_LEN, parseAuthFile, secretsEqual } from "./auth.ts";

const ROBOT_KEY = "robot-key-0123456789abcdef";
const VIEWER_TOKEN = "viewer-token-0123456789abcdef";
const FILE = JSON.stringify({
  robots: { "go2-lab": ROBOT_KEY },
  viewers: { paul: VIEWER_TOKEN },
});

Deno.test("secretsEqual: equal, different, prefix, empty", () => {
  assert(secretsEqual("abcdefghijklmnop", "abcdefghijklmnop"));
  assertFalse(secretsEqual("abcdefghijklmnop", "abcdefghijklmnoq"));
  assertFalse(secretsEqual("abcdefghijklmnop", "abcdefghijklmno"));
  assertFalse(secretsEqual("", "a"));
});

Deno.test("a robot key is bound to its id", () => {
  const auth = parseAuthFile(FILE);
  assert(auth.robotKeyOk("go2-lab", ROBOT_KEY));
  assertFalse(auth.robotKeyOk("go2-lab", "wrong-key-0123456789abcdef"));
  assertFalse(auth.robotKeyOk("go2-lab", undefined));
  assertFalse(auth.robotKeyOk("g1-office", ROBOT_KEY)); // the right key for another id
  assertFalse(auth.robotKeyOk("go2-lab", VIEWER_TOKEN)); // a viewer token is not a robot key
});

Deno.test("a viewer token resolves to its name", () => {
  const auth = parseAuthFile(FILE);
  assertEquals(auth.viewerName(VIEWER_TOKEN), "paul");
  assertEquals(auth.viewerName("wrong-token-0123456789abcdef"), null);
  assertEquals(auth.viewerName(undefined), null);
  assertEquals(auth.viewerName(ROBOT_KEY), null);
});

Deno.test("bearerOk accepts exactly `Bearer <viewer token>`", () => {
  const auth = parseAuthFile(FILE);
  assert(auth.bearerOk(`Bearer ${VIEWER_TOKEN}`));
  assertFalse(auth.bearerOk(null));
  assertFalse(auth.bearerOk(VIEWER_TOKEN));
  assertFalse(auth.bearerOk(`Basic ${VIEWER_TOKEN}`));
  assertFalse(auth.bearerOk(`Bearer ${ROBOT_KEY}`));
  assertFalse(auth.bearerOk("Bearer "));
});

Deno.test("parseAuthFile names the problem and never a secret", () => {
  const SECRET = "leaky-secret-0123456789abcdef";
  const cases: [string, string][] = [
    ["{not json", "not valid JSON"],
    ["[]", "top level must be an object"],
    [JSON.stringify({ robots: {} }), '"viewers" must be an object'],
    [JSON.stringify({ robots: [], viewers: {} }), '"robots" must be an object'],
    [JSON.stringify({ robots: { "": SECRET }, viewers: {} }), 'empty robot id in "robots"'],
    [JSON.stringify({ robots: {}, viewers: { paul: 7 } }), 'viewers["paul"] must be a string'],
    [
      JSON.stringify({ robots: {}, viewers: { paul: "short" } }),
      `at least ${MIN_SECRET_LEN} characters`,
    ],
    [
      JSON.stringify({ robots: { a: SECRET }, viewers: { paul: SECRET } }),
      'viewers["paul"] reuses the secret of robots["a"]',
    ],
    [
      JSON.stringify({ robots: { a: SECRET, b: SECRET }, viewers: {} }),
      'robots["b"] reuses the secret of robots["a"]',
    ],
  ];
  for (const [text, expected] of cases) {
    const err = assertThrows(() => parseAuthFile(text), Error, expected);
    assert(err.message.startsWith("auth file: "), err.message);
    assertFalse(err.message.includes(SECRET), `secret leaked: ${err.message}`);
  }
});

Deno.test("parseAuthFile accepts empty sections", () => {
  const auth = parseAuthFile(JSON.stringify({ robots: {}, viewers: {} }));
  assertFalse(auth.robotKeyOk("go2-lab", ROBOT_KEY));
  assertEquals(auth.viewerName(VIEWER_TOKEN), null);
  const viewersOnly = new Auth(new Map(), new Map([["ops", VIEWER_TOKEN]]));
  assert(viewersOnly.bearerOk(`Bearer ${VIEWER_TOKEN}`));
});

Deno.test("parseAuthFile rejects secrets that cannot fit in hello", () => {
  const secret = "x".repeat(MAX_TOKEN_LEN + 1);
  const err = assertThrows(
    () => parseAuthFile(JSON.stringify({ robots: {}, viewers: { paul: secret } })),
    Error,
    `at most ${MAX_TOKEN_LEN} characters`,
  );
  assertFalse(err.message.includes(secret));
});
