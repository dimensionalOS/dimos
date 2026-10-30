import type { AuthCheck, AuthEvent, AuthPrompt } from "@earendil-works/pi-ai";
import type { EpochMs } from "./time.ts";

/** A terminal or page identity, stable for the life of one client process. */
export type TerminalId = string & { readonly __brand: "TerminalId" };

/** A model a terminal may pick; `contextWindow` is what "context left" divides by. */
export type ModelChoice = {
  readonly provider: string;
  readonly id: string;
  readonly name: string;
  readonly available: boolean;
  readonly reasoning: boolean;
  readonly contextWindow: number;
};

/** One way to log in to one provider. */
export type ProviderChoice = {
  readonly provider: string;
  readonly name: string;
  readonly method: "api_key" | "oauth";
  /** Whether the OAuth sign-in is backed by a subscription. */
  readonly subscription: boolean;
  /** How the provider is configured now, or null. */
  readonly status: AuthCheck | null;
};

export type Accounts = { readonly providers: readonly ProviderChoice[]; readonly models: readonly ModelChoice[] };

/** One login prompt sent to one terminal. */
export type DialogId = string & { readonly __brand: "DialogId" };

/** A login prompt without its AbortSignal. */
export type DialogPrompt = AuthPrompt extends infer P ? (P extends unknown ? Omit<P, "signal"> : never) : never;

/** This terminal's login flow; only the principal that started it sees it. */
export type LoginState =
  | { readonly phase: "idle" }
  | {
      readonly phase: "running";
      readonly provider: string;
      readonly method: "api_key" | "oauth";
      readonly prompt: { readonly dialog: DialogId; readonly prompt: DialogPrompt } | null;
      /** The newest five progress events, so a device code stays visible while other state changes. */
      readonly info: readonly AuthEvent[];
    }
  | { readonly phase: "done"; readonly provider: string; readonly at: EpochMs }
  | { readonly phase: "failed"; readonly provider: string; readonly error: string };
