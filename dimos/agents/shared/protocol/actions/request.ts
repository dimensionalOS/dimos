import type { UserRequestId } from "../conversation/input.ts";

/**
 * Chosen by the client for one action whose catalog entry retries `by_request_id`, and reused when it retries after the
 * link dropped, so the retry returns the first call's result instead of acting twice.
 */
export type ActionRequestId = string & { readonly __brand: "ActionRequestId" };

/** `crypto.randomUUID()`, a global in browsers and in Node, so `shared/protocol/` needs no `node:` import. */
export function newActionRequest(): ActionRequestId {
  return crypto.randomUUID() as ActionRequestId;
}

/** A typed input's request id; InputGate answers a repeated one with the first submission. */
export function newUserRequest(): UserRequestId {
  return crypto.randomUUID() as UserRequestId;
}
