# DimOS MCP Server with OpenClaw

The OpenClaw plugin lives at `dimos/web/plugin_openclaw/`. It bridges DimOS MCP tools into the OpenClaw agent system.

## How It Works

1. DimOS starts a FastAPI MCP server on port 9990, exposing robot skills as JSON-RPC tools
2. The OpenClaw plugin discovers these tools on startup and registers them with the OpenClaw agent
3. User sends natural language commands via OpenClaw, which the agent translates into MCP tool calls
4. Flow: User → OpenClaw Agent → Plugin → HTTP/JSON-RPC → DimOS MCP → Robot

## Prerequisites

If you haven't cloned and installed DimOS yet, follow the [main README](../../README.md) first.

Install pnpm:

```bash
curl -fsSL https://get.pnpm.io/install.sh | sh -
```

## Terminal 1 — DimOS MCP server

Install dependencies:

```bash
uv sync --extra base --extra unitree
```

For hardware (set `ROBOT_IP` in your environment):

```bash
export ROBOT_IP=<YOUR_ROBOT_IP>
uv run dimos run unitree-go2-agentic-mcp
```

For simulation:

```bash
uv run dimos --simulation run unitree-go2-agentic-mcp
```

## Terminal 2 — OpenClaw gateway

First time only — install and configure the plugin:

```bash
cd dimos/web/plugin_openclaw
pnpm install
pnpm openclaw plugins install -l .
pnpm openclaw config set plugins.entries.dimos.enabled true
pnpm openclaw config set gateway.mode local
```

Set your API keys (first time only):

```bash
echo "ANTHROPIC_API_KEY=<YOUR_KEY>" >> ~/.openclaw/.env
echo "OPENCLAW_GATEWAY_TOKEN=<YOUR_TOKEN>" >> ~/.openclaw/.env  # can be any string, e.g. "test1"
```

Start the gateway:

```bash
cd dimos/web/plugin_openclaw
pnpm openclaw gateway run --port 18789 --verbose
```

You should see `dimos: discovered <N> tool(s)` confirming the plugin loaded.

## Terminal 3 — Send commands

```bash
cd dimos/web/plugin_openclaw
pnpm openclaw agent --session-id dimos-test --message "move forward 1 meter"
```

Or use the interactive TUI:

```bash
cd dimos/web/plugin_openclaw
pnpm openclaw tui
```

# DimOS MCP Server with Claude Code

## How It Works

1. DimOS starts a FastAPI MCP server on port 9990, exposing robot skills as JSON-RPC tools
2. Claude Code connects directly to the MCP server over HTTP
3. User sends natural language commands, which Claude translates into MCP tool calls

## Prerequisites

If you haven't cloned and installed DimOS yet, follow the [main README](../../README.md) first.

## Terminal 1 — DimOS MCP server

Install dependencies:

```bash
uv sync --extra base --extra unitree
```

For hardware (set `ROBOT_IP` in your environment):

```bash
export ROBOT_IP=<YOUR_ROBOT_IP>
uv run dimos run unitree-go2-agentic-mcp
```

For simulation:

```bash
uv run dimos --simulation run unitree-go2-agentic-mcp
```

## Terminal 2 — Claude Code

Add the MCP server (one-time):

```bash
claude mcp add --transport http --scope project dimos http://localhost:9990/mcp
```

Use robot skills:

```
> move forward 1 meter
> go to the kitchen
> tag this location as "desk"
```

# MCP Inspector

For manual inspection:

```bash
npx -y @modelcontextprotocol/inspector
```

Change **Transport Type** to "Streamable HTTP", **URL** to `http://localhost:9990/mcp`, and **Connection Type** to "Direct". Click "Connect".
