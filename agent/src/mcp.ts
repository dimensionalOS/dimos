import { createHash } from "node:crypto";
import {
  Client,
  StreamableHTTPClientTransport,
  type CallToolResult,
  type ProgressNotification,
} from "@modelcontextprotocol/client";
import type { ImageContent, TextContent } from "@earendil-works/pi-ai";
import type {
  ExtensionFactory,
  ToolDefinition,
} from "@earendil-works/pi-coding-agent";
import { z } from "zod";
import { Type, type TUnsafe } from "typebox";
import type { Endpoint } from "./config.js";

export type McpDetails = {
  endpoint: string;
  tool: string;
  result: Omit<CallToolResult, "content">;
};
export function resultContent(
  content: CallToolResult["content"],
): Array<TextContent | ImageContent> {
  return content.map((block) => {
    if (block.type === "text") return { type: "text", text: block.text };
    if (block.type === "image")
      return { type: "image", data: block.data, mimeType: block.mimeType };
    if (block.type === "resource" && "text" in block.resource)
      return { type: "text", text: block.resource.text };
    return { type: "text", text: JSON.stringify(block) };
  });
}
export const toolName = (endpoint: string, name: string): string =>
  "dimos_" +
  endpoint.slice(0, 12) +
  "_" +
  name.replace(/[^a-zA-Z0-9_-]/g, "_").slice(0, 28) +
  "_" +
  createHash("sha256")
    .update(endpoint + "\0" + name)
    .digest("hex")
    .slice(0, 10);

type McpTool = ToolDefinition<TUnsafe<Record<string, unknown>>, McpDetails>;

export class McpTools {
  private readonly clients = new Map<string, Client>();
  private readonly urls = new Map<string, string>();
  private readonly progress = new Map<
    string | number,
    (value: ProgressNotification["params"]) => void
  >();
  constructor(
    private readonly endpoints: () => Promise<readonly Endpoint[]>,
    private readonly report: (message: string) => void = () => {},
  ) {}
  async tools(): Promise<McpTool[]> {
    const tools: McpTool[] = [];
    const endpoints = await this.endpoints();
    for (const [name, client] of this.clients)
      if (
        !endpoints.some(
          (endpoint) =>
            endpoint.name === name && endpoint.url === this.urls.get(name),
        )
      ) {
        await client.close();
        this.clients.delete(name);
      }
    for (const endpoint of endpoints) {
      let client = this.clients.get(endpoint.name);
      try {
        if (!client) {
          client = new Client({ name: "dimcode", version: "0.1.0" });
          client.onerror = (error) =>
            this.report(endpoint.name + ": " + error.message);
          const connected = client;
          client.onclose = () => {
            if (this.clients.get(endpoint.name) === connected)
              this.clients.delete(endpoint.name);
            this.report(endpoint.name + ": disconnected");
          };
          client.setNotificationHandler(
            "notifications/message",
            (notification) =>
              this.report(
                endpoint.name + ": " + JSON.stringify(notification.params.data),
              ),
          );
          client.setNotificationHandler(
            "notifications/progress",
            (notification) => {
              const progress = notification.params;
              const update = this.progress.get(progress.progressToken);
              if (update) update(progress);
              else
                this.report(
                  endpoint.name +
                    " background " +
                    progress.progressToken +
                    ": " +
                    (progress.message ?? progress.progress),
                );
            },
          );
          client.fallbackNotificationHandler = async (notification) =>
            this.report(endpoint.name + ": " + JSON.stringify(notification));
          await client.connect(
            new StreamableHTTPClientTransport(new URL(endpoint.url)),
          );
          this.clients.set(endpoint.name, client);
          this.urls.set(endpoint.name, endpoint.url);
        }
        const connection = client;
        const remoteTools = [];
        let cursor: string | undefined;
        do {
          const listing = await connection.listTools({ cursor });
          remoteTools.push(...listing.tools);
          cursor = listing.nextCursor;
        } while (cursor);
        const names = new Set<string>();
        for (const remote of remoteTools) {
          if (names.has(remote.name))
            throw new Error("Duplicate remote tool name: " + remote.name);
          names.add(remote.name);
          const parameters = Type.Unsafe<Record<string, unknown>>(
            remote.inputSchema,
          );
          const tool: ToolDefinition<typeof parameters, McpDetails> = {
            name: toolName(endpoint.name, remote.name),
            label: endpoint.name + " / " + remote.name,
            description:
              (remote.description ?? remote.name) +
              "\nDimOS metadata: " +
              JSON.stringify(remote._meta ?? {}),
            parameters,
            execute: async (id, args, signal, update) => {
              if (this.clients.get(endpoint.name) !== connection)
                throw new Error("MCP endpoint disconnected; refresh tools.");
              this.progress.set(id, (progress) =>
                update?.({
                  content: [
                    {
                      type: "text",
                      text: progress.message ?? String(progress.progress),
                    },
                  ],
                  details: {
                    endpoint: endpoint.name,
                    tool: remote.name,
                    result: {},
                  },
                }),
              );
              try {
                const result = await connection.callTool(
                  {
                    name: remote.name,
                    arguments: args,
                    _meta: { progressToken: id },
                  },
                  { signal },
                );
                const { content, ...metadata } = result;
                return {
                  content: resultContent(content),
                  details: {
                    endpoint: endpoint.name,
                    tool: remote.name,
                    result: metadata,
                  },
                };
              } finally {
                this.progress.delete(id);
              }
            },
          };
          tools.push(tool);
        }
      } catch (error) {
        await client?.close();
        this.clients.delete(endpoint.name);
        this.report(endpoint.name + ": " + String(error));
      }
    }
    return tools;
  }
  extension(): ExtensionFactory {
    return async (pi) => {
      for (const tool of await this.tools()) pi.registerTool(tool);
      pi.on("tool_result", (event) => {
        const details = z
          .object({
            endpoint: z.string(),
            result: z.object({ isError: z.boolean().optional() }),
          })
          .safeParse(event.details);
        if (details.success && details.data.result.isError)
          return { isError: true };
        return undefined;
      });
      pi.on("session_shutdown", () => this.close());
    };
  }
  async close(): Promise<void> {
    await Promise.all(
      [...this.clients.values()].map((client) => client.close()),
    );
    this.clients.clear();
  }
}
