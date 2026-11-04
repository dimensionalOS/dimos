import type { Context } from "@earendil-works/chord/context";
import type { DimosConnection, RemoteTool } from "#dimos/agents/harness/dimos/connection.ts";
import type { Harness } from "@earendil-works/pi-durable";
import type { Live } from "#dimos/agents/shared/protocol/status.ts";
import type { RuntimeToolName } from "#dimos/agents/shared/protocol/tools/details.ts";
import { Status } from "#dimos/agents/shared/protocol/status.ts";
import { defineDoc } from "@earendil-works/pi-durable";
import { isRuntimeToolName } from "#dimos/agents/harness/tools/details.ts";

export type RuntimeToolSchemas = { readonly [N in Exclude<RuntimeToolName, "stream_get">]?: RemoteTool };

const RuntimeToolsDoc = defineDoc<{ tools: RuntimeToolSchemas }>({
  kind: "dimos.runtime_tools",
  version: 1,
  scope: "session",
  initial: () => ({ tools: {} }),
});

function sameSchemas(a: RuntimeToolSchemas, b: RuntimeToolSchemas): boolean {
  return JSON.stringify(a) === JSON.stringify(b);
}

/** The HarnessRuntime tool schemas McpServer last advertised, kept in the store so the tools are registered before McpServer answers. */
export class RuntimeToolCatalog {
  private readonly harness: Harness;
  private readonly current: Status<RuntimeToolSchemas>;
  readonly schemas: Live<RuntimeToolSchemas>;

  private constructor(harness: Harness, schemas: RuntimeToolSchemas) {
    this.harness = harness;
    this.current = new Status(schemas, (error) => {
      throw error;
    });
    this.schemas = this.current;
  }

  static async load(harness: Harness, context: Context): Promise<RuntimeToolCatalog> {
    const doc = await harness.snapshot(RuntimeToolsDoc, context);
    return new RuntimeToolCatalog(harness, doc?.tools ?? {});
  }

  /** Stores the remote runtime schemas. stream_get is implemented locally through the Web SDK. */
  async refresh(connection: DimosConnection, context: Context): Promise<"complete" | "missing"> {
    const listed: { -readonly [N in keyof RuntimeToolSchemas]: RuntimeToolSchemas[N] } = {};
    for (const tool of await connection.listTools()) {
      if (isRuntimeToolName(tool.name) && tool.name !== "stream_get") listed[tool.name] = tool;
    }
    if (!sameSchemas(listed, this.current.value)) {
      await this.harness.commit(async (tx) => {
        const doc = await tx.doc(RuntimeToolsDoc);
        doc.tools = { ...listed };
      }, context);
      this.current.set(listed);
    }
    return listed.inventory && listed.agent_module ? "complete" : "missing";
  }
}
