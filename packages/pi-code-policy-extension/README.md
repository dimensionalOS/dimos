# Pi CodePolicy extension

This package adds one `python_exec` tool to the stock Pi CLI. The tool connects
to the evaluator-owned MCP server named by `DIMOS_CODE_POLICY_MCP_URL`.

The Python evaluator launches Pi with all built-in tools and extension discovery
disabled, then loads only `dist/python-exec.js`. The Python REPL provides
`submit_policy(policy)`; it is not a second Pi tool.
