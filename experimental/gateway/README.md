# Desktop gateway (experimental)

`python -m experimental.gateway --socket <path> --dimos-dir <checkout>` serves the existing `/dimos` Desktop API. `--no-zenoh` disables bus publishing; `--write-openapi` refreshes `openapi.json`.

- `server/`: application state, assembly and process lifecycle.
- `utils/`: shared discovery, launch, configuration, codec and upload logic.
- `endpoints/`: one leaf module per literal HTTP path. `/dimos/blueprints/{name}/config` maps to `dimos/blueprints/{name}/config.py`; `/dimos/msgs.js` maps to `dimos/msgs.js.py`. Root `/` would use `index.py`. The checked inventory registers every leaf explicitly.
- `tests/`: gateway and Desktop contract regressions.
- `annotations.json`: generated robot/blueprint metadata. Its HTTP endpoint remains `/dimos/robots`.
- `assets/`: Desktop's blueprint details page, cloud login and message codecs.

The skills endpoints remain. The published API 1.18 RPC list/call endpoints are included too, so both installed and newer Portal clients work.

A launch transaction serializes checking, spawning and recording runs across threads and gateway processes. When Desktop sets `DESKTOP_URL`, config edits go through Desktop's `/api/config` owner; offline edits use a locked file transaction. Cleared overrides use null leaf markers, which both launch override merging and gateway reads remove.

Run:

```sh
python -m pytest experimental/gateway/tests
python -m mypy experimental/gateway/server experimental/gateway/utils
python -m experimental.gateway.utils.check_endpoint_types
```

Literal URL filenames are not valid Python package names for mypy. The last command checks each leaf as strict source with the same repository configuration; CI requires both type-check commands.
