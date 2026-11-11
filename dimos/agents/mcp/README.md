# DimOS MCP Server

Expose DimOS robot skills to coding agents (Claude Code, Codex, Cursor, and more) via Model Context Protocol.

## Setup

```bash
uv sync --extra base --extra unitree --inexact
```

Add to your coding agents (one command)

```bash
npx add-mcp http://localhost:9990/mcp --name dimos      # project-level: this directory only
npx add-mcp http://localhost:9990/mcp --name dimos -g   # global: all projects (user-level)
```

Verify that it was added:

```bash
npx add-mcp list-agents
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
uv run dimos run unitree-go2-agentic
```

**Your coding agent** - Use robot skills:
```
> move forward 1 meter
> go to the kitchen
> tag this location as "desk"
```

## How It Works

1. `McpServer` in the blueprint starts a FastAPI server on port 9990
2. Your coding agent connects directly to `http://localhost:9990/mcp`
3. Skills are exposed as MCP tools (e.g., `move_to`, `navigate_with_text`)
