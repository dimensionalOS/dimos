import type { AgentModuleName } from "../../shared/protocol/runtime/event.ts";
import type { ConversationId, JsonObject } from "@earendil-works/pi-durable";
import type { DimosConnection } from "./connection.ts";
import type { Facts } from "../../shared/protocol/runtime/context.ts";
import type { ImageContent, TextContent } from "@earendil-works/pi-ai";
import type { Inventory } from "../../shared/protocol/runtime/inventory.ts";
import type { ModuleOutcome } from "../../shared/protocol/runtime/module.ts";
import type { Session } from "@dimos/sdk";
import type { Step } from "../trajectory/steps.ts";
import type { Live } from "../../shared/protocol/status.ts";
import type { StreamRef } from "../../shared/protocol/streams.ts";
import { Status } from "../../shared/protocol/status.ts";
import { AgentModuleRecordSchema } from "../../shared/protocol/runtime/module.ts";
import { Type } from "typebox";
import { openRelay, relayStreams } from "../../sdk/relay.ts";
import { inventoryDecoder, inventoryFactsDecoder } from "./wire/inventory.ts";
import { moduleOutcomeDecoder } from "./wire/module.ts";
import { wireDecoder } from "./wire/decoder.ts";

const moduleListDecoder = wireDecoder(
  "agent_module list",
  Type.Object({ agent_modules: Type.Array(AgentModuleRecordSchema) }),
);

const publishedStepsDecoder = wireDecoder("publish_steps", Type.Object({ published: Type.Integer({ minimum: 0 }) }));

/** The `structuredContent` McpServer sends for a SkillResult; HarnessRuntime's data is its `metadata`. */
const skillResultDecoder = wireDecoder("SkillResult", Type.Object({ metadata: Type.Unknown() }));

function textOf(content: readonly (TextContent | ImageContent)[]): string {
  return content.flatMap((block) => (block.type === "text" ? [block.text] : [])).join("\n");
}

/**
 * The HarnessRuntime skills the gateway calls for itself, typed and decoded. Every MCP call throws
 * `DimosUnavailable` at once while the link is down. Stream consumers share the SDK session for this process.
 */
export class RuntimeClient {
  private readonly connection: DimosConnection;
  private currentRelay: { key: string; session: Promise<Session>; stop: () => void; inventory: Inventory } | null =
    null;
  private readonly streamStatus: Status<readonly StreamRef[]>;
  readonly streams: Live<readonly StreamRef[]>;
  private closed = false;

  constructor(connection: DimosConnection, report: (error: unknown) => void = () => {}) {
    this.connection = connection;
    this.streamStatus = new Status<readonly StreamRef[]>([], report);
    this.streams = this.streamStatus;
  }

  /** `inventory compact=true`: the system's keyed facts. */
  async snapshot(signal?: AbortSignal): Promise<Facts> {
    return inventoryFactsDecoder.decode(await this.call("inventory", { compact: true }, 10_000, signal));
  }

  /** `inventory` with no arguments, for the `/runtime` view. */
  async inventory(signal?: AbortSignal): Promise<Inventory> {
    return inventoryDecoder.decode(await this.call("inventory", {}, 10_000, signal));
  }

  async publishSteps(steps: readonly Step[]): Promise<void> {
    const result = await this.call(
      "publish_steps",
      {
        steps: steps.map((step) => ({
          conversation: step.conversation,
          entry: String(step.entry),
          kind: step.kind,
          text: step.text,
          ts: step.at / 1000,
        })),
      },
      10_000,
      undefined,
    );
    const { published } = publishedStepsDecoder.decode(result);
    if (published !== steps.length) throw new Error(`publish_steps published ${published} of ${steps.length} steps`);
  }

  /** `agent_module op=create`, sent with `caller: owner` so HarnessRuntime records the owning conversation. */
  async createModule(name: AgentModuleName, path: string, owner: ConversationId | null): Promise<ModuleOutcome> {
    return this.moduleChange("create", name, { op: "create", name, path }, owner);
  }

  async moduleOp(op: "restart" | "unload" | "delete", name: AgentModuleName): Promise<ModuleOutcome> {
    return this.moduleChange(op, name, { op, name }, null);
  }

  async relay(inventory?: Inventory, signal?: AbortSignal): Promise<Session> {
    const currentInventory = inventory ?? (await this.inventory(signal));
    const endpoint = currentInventory.environment.relay;
    if (this.closed) throw new Error("Runtime client is closed");
    if (endpoint === null) {
      this.disconnectRelay();
      throw new Error("The DimOS relay is not running; run dimos agent load");
    }
    const key = JSON.stringify(endpoint);
    if (this.currentRelay?.key !== key) {
      this.disconnectRelay();
      const current = { key, session: openRelay(endpoint), stop: () => {}, inventory: currentInventory };
      this.currentRelay = current;
      void current.session.then(
        (session) => {
          if (this.currentRelay !== current) return;
          const changed = () => this.streamStatus.set(relayStreams(session, current.inventory));
          current.stop = session.status.subscribe(changed);
          changed();
        },
        () => {
          if (this.currentRelay === current) this.disconnectRelay();
        },
      );
    }
    const current = this.currentRelay;
    current.inventory = currentInventory;
    const session = await current.session;
    if (this.currentRelay === current) this.streamStatus.set(relayStreams(session, current.inventory));
    return session;
  }

  close(): void {
    this.closed = true;
    this.disconnectRelay();
  }

  disconnectRelay(): void {
    this.currentRelay?.stop();
    void this.currentRelay?.session.then(
      (session) => session.close(),
      () => {},
    );
    this.currentRelay = null;
    this.streamStatus.set([]);
  }

  /**
   * An op HarnessRuntime ran answers with its outcome, failed or not. One it refused before running (an unknown name,
   * a create of a module that exists) is an MCP error with text only; that outcome takes the module's state from `list`.
   */
  private async moduleChange(
    op: ModuleOutcome["op"],
    name: AgentModuleName,
    args: JsonObject,
    caller: ConversationId | null,
  ): Promise<ModuleOutcome> {
    const outcome = await this.connection.callTool("agent_module", args, { caller, timeoutMs: 120_000 });
    if (outcome.structured !== undefined)
      return moduleOutcomeDecoder.decode(skillResultDecoder.decode(outcome.structured).metadata);
    if (!outcome.isError) throw new Error(`agent_module ${op} answered without an outcome: ${textOf(outcome.content)}`);
    const listed = moduleListDecoder.decode(await this.call("agent_module", { op: "list" }, 10_000, undefined));
    const module = listed.agent_modules.find((row) => row.name === name);
    return {
      name,
      op,
      ok: false,
      state: module === undefined ? "absent" : module.state,
      path: module === undefined ? null : module.path,
      revision: module === undefined ? null : module.revision,
      error: textOf(outcome.content),
      traceback: null,
    };
  }

  /** The call's SkillResult metadata; an MCP error or a result without structured content throws. */
  private async call(
    name: string,
    args: Parameters<DimosConnection["callTool"]>[1],
    timeoutMs: number,
    signal: AbortSignal | undefined,
  ): Promise<unknown> {
    const outcome = await this.connection.callTool(name, args, { caller: null, timeoutMs, signal });
    if (outcome.isError || outcome.structured === undefined)
      throw new Error(`${name} failed: ${textOf(outcome.content)}`);
    return skillResultDecoder.decode(outcome.structured).metadata;
  }
}
