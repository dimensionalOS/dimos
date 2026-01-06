# DimOS MCP Server

Expose DimOS robot skills to Claude Code via Model Context Protocol.

## Setup

Add to Claude Code (one command):
```bash
claude mcp add --transport stdio dimos --scope project -- python -m dimos.protocol.mcp
```

## Usage

**Terminal 1** - Start DimOS:
```bash
dimos --replay run unitree-go2-agentic
```

**Claude Code** - Use robot skills:
```
> move forward 1 meter
> go to the kitchen
> tag this location as "desk"
```

## How It Works

1. `llm_agent(mcp_port=9990)` in the blueprint starts a TCP server
2. Claude Code spawns the bridge (`--bridge`) which connects to `localhost:9990`
3. Skills are exposed as MCP tools (e.g., `relative_move`, `navigate_with_text`)
