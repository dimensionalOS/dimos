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

## History compaction

`McpClient` keeps the full conversation and replays it on every turn, so a long
session eventually outgrows the model's context window and every subsequent
request fails. To prevent that, the history is compacted before each model call:
the oldest messages are dropped and replaced with a summary, keeping the recent
turns verbatim.

Compaction is on by default and is configured on `McpClientConfig`:

| Option | Default | Meaning |
| -- | -- | -- |
| `compaction_enabled` | `True` | Turn compaction off entirely |
| `context_window` | `None` | Window size in tokens; `None` resolves it from `model` |
| `compaction_trigger_ratio` | `0.8` | Compact once the history passes this fraction of the window |
| `compaction_keep_ratio` | `0.35` | Target size of the history after compaction |
| `compaction_summarize_with_model` | `True` | Summarise with the model; otherwise use an offline digest |

Notes:

- Tool results are never separated from the tool call that produced them, so a
  compacted history stays valid for the OpenAI API.
- A leading system message is never dropped.
- If summarisation fails, an offline digest is used instead; if compaction
  itself fails, the turn proceeds uncompacted. Compaction never fails a turn.
- The window for an unrecognised model falls back to a conservative default.
  Add known models to `MODEL_CONTEXT_WINDOWS` in `dimos/agents/compaction.py`.

