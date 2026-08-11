import {
  Client,
  StreamableHTTPClientTransport,
  type CallToolResult,
} from "@modelcontextprotocol/client";
import { highlightCode, type ExtensionAPI } from "@earendil-works/pi-coding-agent";
import { Text } from "@earendil-works/pi-tui";
import { Type } from "typebox";

const TOOL_NAME = "python_exec";
const DEFAULT_TIMEOUT_SECONDS = 600;

interface McpClient {
  listTools(): Promise<{ tools: Array<{ name: string; description?: string }> }>;
  callTool(
    params: { name: string; arguments: Record<string, unknown> },
    options?: { timeout?: number },
  ): Promise<CallToolResult>;
  close(): Promise<void>;
}

export function textFromResult(result: CallToolResult): string {
  return result.content
    .filter((item): item is Extract<typeof item, { type: "text" }> => item.type === "text")
    .map((item) => item.text)
    .join("\n");
}

async function connect(url: string): Promise<McpClient> {
  const client = new Client({ name: "dimos-pi-code-policy", version: "1.0.0" });
  await client.connect(new StreamableHTTPClientTransport(new URL(url)));
  return client;
}

export async function installPythonExec(
  pi: ExtensionAPI,
  mcpUrl: string,
  connectClient: (url: string) => Promise<McpClient> = connect,
): Promise<void> {
  const client = await connectClient(mcpUrl);
  const inventory = await client.listTools();
  if (inventory.tools.length !== 1 || inventory.tools[0]?.name !== TOOL_NAME) {
    await client.close();
    throw new Error("CodePolicy MCP server must expose exactly python_exec");
  }

  pi.registerTool({
    name: TOOL_NAME,
    label: "Execute Python",
    description:
      inventory.tools[0].description ??
      "Execute Python in a persistent trusted, unsandboxed session.",
    parameters: Type.Object(
      {
        code: Type.String({ minLength: 1 }),
        timeout_s: Type.Optional(
          Type.Number({ exclusiveMinimum: 0, maximum: DEFAULT_TIMEOUT_SECONDS }),
        ),
      },
      { additionalProperties: false },
    ),
    executionMode: "sequential",
    renderCall: (params, theme) => {
      const title = theme.fg("toolTitle", theme.bold("Python"));
      const code = highlightCode(params.code, "python").join("\n");
      return new Text(`${title}\n${code}`, 0, 0);
    },
    renderResult: (result, _options, theme, context) => {
      const output = result.content
        .filter((item): item is Extract<typeof item, { type: "text" }> => item.type === "text")
        .map((item) => item.text)
        .join("\n");
      const isError = context.isError;
      const title = theme.fg(
        isError ? "error" : "success",
        theme.bold(isError ? "Python error" : "Python result"),
      );
      const body = highlightCode(output || "(completed)", "python").join("\n");
      return new Text(`${title}\n${body}`, 0, 0);
    },
    execute: async (_id, params) => {
      const timeoutSeconds = params.timeout_s ?? DEFAULT_TIMEOUT_SECONDS;
      const result = await client.callTool(
        {
          name: TOOL_NAME,
          arguments: { code: params.code, timeout_s: timeoutSeconds },
        },
        { timeout: (timeoutSeconds + 10) * 1000 },
      );
      return { content: [{ type: "text", text: textFromResult(result) }], details: {} };
    },
  });

  pi.on("session_shutdown", async () => {
    await client.close();
  });
}

export default async function pythonExecExtension(pi: ExtensionAPI): Promise<void> {
  const mcpUrl = process.env.DIMOS_CODE_POLICY_MCP_URL;
  if (!mcpUrl) throw new Error("DIMOS_CODE_POLICY_MCP_URL is required");
  await installPythonExec(pi, mcpUrl);
}
