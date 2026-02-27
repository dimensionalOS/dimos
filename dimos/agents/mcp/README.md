# DimOS MCP Server

Expose DimOS robot skills to Claude Code via Model Context Protocol.

## Setup

```bash
uv sync --extra base --extra unitree
```

Add to Claude Code (one command)

```bash
claude mcp add --transport http --scope project dimos http://localhost:9990/mcp
```

Verify that it was added:

```bash
claude mcp list
```

## MCP Inspector

If you want to inspect the server manually, you can use MCP Inspector.

Install it:

```bash
npx -y @modelcontextprotocol/inspector
```

It will open a browser window.

Change **Transport Type** to "Streamable HTTP", change **URL** to `http://localhost:9990/mcp`, and **Connection Type** to "Direct". Then click on "Connect".

## Usage

**Terminal 1** - Start DimOS:
```bash
uv run dimos run unitree-go2-agentic-mcp
```

**Claude Code** - Use robot skills:
```
> move forward 1 meter
> go to the kitchen
> tag this location as "desk"
```

## How It Works

1. `McpServer` in the blueprint starts a FastAPI server on port 9990
2. Claude Code connects directly to `http://localhost:9990/mcp`
3. Skills are exposed as MCP tools (e.g., `relative_move`, `navigate_with_text`)
