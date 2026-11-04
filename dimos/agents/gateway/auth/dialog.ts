import type { AuthInteraction, AuthPrompt, Models } from "@earendil-works/pi-ai";
import type { DialogId, LoginState } from "#dimos/agents/shared/protocol/auth.ts";
import type { ModelCatalog } from "#dimos/agents/gateway/auth/models.ts";
import type { Outcome } from "#dimos/agents/shared/protocol/outcome.ts";
import type { Live } from "#dimos/agents/shared/protocol/status.ts";
import type { Principal } from "#dimos/agents/gateway/auth/principal.ts";
import { Status } from "#dimos/agents/shared/protocol/status.ts";
import { done, refused } from "#dimos/agents/shared/protocol/outcome.ts";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
import { principalKey } from "#dimos/agents/gateway/auth/principal.ts";
import { randomUUID } from "node:crypto";

const KEPT_INFO = 5;

type OpenDialog = {
  readonly id: DialogId;
  readonly resolve: (value: string) => void;
  readonly reject: (error: Error) => void;
};

type Flow = {
  readonly status: Status<LoginState>;
  open: OpenDialog | null;
  /** The newest login's; aborting it ends that login and unblocks pi's credential queue for its provider. */
  login: AbortController | null;
};

function messageOf(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

/**
 * pi-ai's login flows, one per principal key at a time. A prompt is that principal's LoginState, which its gateway view
 * feeds into a cell, so the device code stays on screen whatever else changes and survives a reconnect of the same
 * terminal. Chord never awaits a person inside a method: `start` returns once the flow runs, `answer` resolves the open
 * prompt, and the result reaches the client through `loginFlow` and `accounts`. A login ends when its prompt waits
 * `promptMs` or `cancel` is called, as pi runs a provider's next login or logout only after it.
 */
export class LoginFlows {
  private readonly models: Models;
  private readonly catalog: ModelCatalog;
  private readonly report: (error: unknown) => void;
  private readonly promptMs: number;
  private readonly flows = new Map<string, Flow>();

  constructor(models: Models, catalog: ModelCatalog, report: (error: unknown) => void, promptMs: number) {
    this.models = models;
    this.catalog = catalog;
    this.report = report;
    this.promptMs = promptMs;
  }

  of(principal: Principal): Live<LoginState> {
    return this.flowOf(principal).status;
  }

  /** Starts `models.login` with this principal's interaction; refreshes the catalog when it ends either way. */
  async start(principal: Principal, provider: string, method: "api_key" | "oauth"): Promise<Outcome<null>> {
    const flow = this.flowOf(principal);
    if (flow.status.value.phase === "running")
      return refused({
        code: "invalid",
        field: "provider",
        problem: `a login to ${flow.status.value.provider} is in progress`,
      });
    if (this.models.getProvider(provider) === undefined)
      return refused({ code: "not_found", what: "provider", name: provider });
    if (provider === "anthropic" && method === "oauth")
      return refused({ code: "invalid", field: "method", problem: "Anthropic OAuth is not offered; use an API key" });
    const login = new AbortController();
    flow.login = login;
    flow.status.set({ phase: "running", provider, method, prompt: null, info: [] });
    this.models
      .login(provider, method, this.interaction(flow, login.signal))
      .then(
        () => flow.status.set({ phase: "done", provider, at: epochMs(Date.now()) }),
        (error: unknown) => flow.status.set({ phase: "failed", provider, error: messageOf(error) }),
      )
      .then(() => this.catalog.refresh())
      .catch(this.report);
    return done(null);
  }

  /** Ends the principal's running login, if any: nobody of that principal is left to answer its prompt. */
  cancel(principal: Principal): void {
    const flow = this.flows.get(principalKey(principal));
    if (flow !== undefined) this.abort(flow, new Error("The terminal that started the login disconnected"));
  }

  /** null cancels. `not_found` when no login runs for the principal, `invalid` when the dialog is not its open prompt. */
  async answer(principal: Principal, dialog: DialogId, value: string | null): Promise<Outcome<null>> {
    const flow = this.flows.get(principalKey(principal));
    if (flow === undefined || flow.status.value.phase !== "running")
      return refused({ code: "not_found", what: "provider", name: dialog });
    const { open } = flow;
    if (open === null || open.id !== dialog)
      return refused({ code: "invalid", field: "dialog", problem: `${dialog} is not the open prompt` });
    if (value === null) open.reject(new Error("Cancelled"));
    else open.resolve(value);
    return done(null);
  }

  private flowOf(principal: Principal): Flow {
    const key = principalKey(principal);
    const known = this.flows.get(key);
    if (known !== undefined) return known;
    const created: Flow = {
      status: new Status<LoginState>({ phase: "idle" }, (error) => {
        throw error;
      }),
      open: null,
      login: null,
    };
    this.flows.set(key, created);
    return created;
  }

  private abort(flow: Flow, reason: Error): void {
    flow.login?.abort(reason);
    flow.open?.reject(reason);
  }

  /** Its callbacks are own properties, because pi-ai copies an interaction with object spread. */
  private interaction(flow: Flow, signal: AbortSignal): AuthInteraction {
    return {
      signal,
      prompt: (prompt) => this.prompt(flow, prompt),
      notify: (event) => {
        if (flow.status.value.phase !== "running") return;
        flow.status.set({ ...flow.status.value, info: [...flow.status.value.info, event].slice(-KEPT_INFO) });
      },
    };
  }

  private prompt(flow: Flow, prompt: AuthPrompt): Promise<string> {
    if (flow.status.value.phase !== "running") return Promise.reject(new Error("No login is running"));
    if (flow.open !== null) return Promise.reject(new Error("Answer the open prompt first"));
    const { signal, ...wire } = prompt;
    if (signal?.aborted || flow.login?.signal.aborted) return Promise.reject(new Error("Interaction cancelled"));
    const id = randomUUID() as DialogId;
    return new Promise<string>((resolve, reject) => {
      const aborted = () => flow.open?.id === id && flow.open.reject(new Error("Interaction cancelled"));
      const unanswered = setTimeout(
        () => this.abort(flow, new Error("Nobody answered the login prompt")),
        this.promptMs,
      );
      const settle = () => {
        clearTimeout(unanswered);
        signal?.removeEventListener("abort", aborted);
        flow.open = null;
        if (flow.status.value.phase === "running") flow.status.set({ ...flow.status.value, prompt: null });
      };
      flow.open = {
        id,
        resolve: (value) => {
          settle();
          resolve(value);
        },
        reject: (error) => {
          settle();
          reject(error);
        },
      };
      signal?.addEventListener("abort", aborted, { once: true });
      if (flow.status.value.phase === "running")
        flow.status.set({ ...flow.status.value, prompt: { dialog: id, prompt: wire } });
    });
  }
}
