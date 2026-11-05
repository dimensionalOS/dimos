import type { ChordError } from "@earendil-works/chord";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { Refusal } from "#dimos/agents/shared/protocol/outcome.ts";
import { isChordError } from "@earendil-works/chord";

export class RefusalError extends Error {
  readonly refusal: Refusal;

  constructor(refusal: Refusal) {
    super(`the request was refused: ${refusal.code}`);
    this.refusal = refusal;
  }
}

/** A ChordError's message without the `code/reason: ` its constructor adds; NULs between address parts as spaces. */
function problemOf(error: ChordError): string {
  const prefix = `${error.code}${error.reason === undefined ? "" : `/${error.reason}`}: `;
  const message = error.message.startsWith(prefix) ? error.message.slice(prefix.length) : error.message;
  return message.replaceAll("\0", " ");
}

/**
 * A Chord call failure as a refusal: `not_in_variant` (the variant changed under the caller) or `forbidden` is
 * `viewing`, `invalid` (arguments or input the gateway rejected before anything ran) is `invalid` with Chord's reason
 * as the field, `contract_mismatch` or `upgrade_required` is `other_build`, and anything else is `unknown_outcome`:
 * `unavailable`, `aborted`, `released`, `gone` and `superseded` can end a call that already ran. Chord's wire errors
 * carry only code, message and reason (`chord/dist/wire.js:156-163`).
 */
export function refusalOfFailure(error: unknown, since: EpochMs): Refusal {
  if (error instanceof RefusalError) return error.refusal;
  if (isChordError(error)) {
    switch (error.code) {
      case "not_in_variant":
      case "forbidden":
        return { code: "viewing", conversation: null };
      case "invalid":
        return { code: "invalid", field: error.reason ?? "arguments", problem: problemOf(error) };
      case "contract_mismatch":
      case "upgrade_required":
        return { code: "other_build", gatewayBuild: null };
      default:
        return { code: "unknown_outcome", since };
    }
  }
  return { code: "unknown_outcome", since };
}

/** Whether a failure leaves the call's outcome unknown (the link dropped mid-call), so a `by_request_id` action may be sent again. */
export function outcomeUnknown(error: unknown): boolean {
  return isChordError(error) && (error.code === "unavailable" || error.code === "aborted");
}

function isOwner<R extends { readonly variant: string }>(ref: R): ref is Extract<R, { readonly variant: "owner" }> {
  return ref.variant === "owner";
}

/** Narrows the live Chord variant without wrapping or retaining the reference. */
export function owner<R extends { readonly variant: string }>(ref: R): Extract<R, { readonly variant: "owner" }> {
  if (!isOwner(ref)) throw new RefusalError({ code: "viewing", conversation: null });
  return ref;
}
