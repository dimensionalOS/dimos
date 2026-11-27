# DimOS MCP Server

Expose DimOS robot skills to Claude Code via Model Context Protocol.

## Setup

```bash
uv sync --extra base --extra unitree --inexact
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

If you want to inspect the server manually, you can use MCP Inspector. It has to be told to use the revision the server speaks, which it calls the modern protocol era:

```bash
npx -y @modelcontextprotocol/inspector --cli http://localhost:9990/mcp --protocol-era modern --method tools/list
```

## Usage

**Terminal 1** - Start DimOS:
```bash
uv run dimos run unitree-go2-agentic
```

**Claude Code** - Use robot skills:
```
> move forward 1 meter
> go to the kitchen
> tag this location as "desk"
```

## How It Works

1. `McpServer` in the blueprint starts an aiohttp server on port 9990
2. Claude Code connects directly to `http://localhost:9990/mcp`
3. Skills are exposed as MCP tools (e.g., `move_to`, `navigate_with_text`)
