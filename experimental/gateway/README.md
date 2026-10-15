# dimos gateway (experimental)

`python -m experimental.gateway --socket <path> --dimos-dir <checkout>` serves the HTTP API that dimOS Desktop,
apps and agents use: blueprints, config, runs and their logs, skills, and the extras
installs and blueprint scan (as jobs). `--port 8123` also serves it on `127.0.0.1:8123`; `--detach` starts it in the
background. Routes have no prefix: Desktop puts them under `/dimos-gateway`, so `/runs` here is `/dimos-gateway/runs`
through Desktop.

- `GET /openapi.json` lists every route (FastAPI's own document); `x-slow: true` marks the ones that import dimos.
- Events go on zenoh at `<ns>/dimos/events/<type>` (`launch`, `blueprints`); the namespace is `--zenoh-namespace` or `$DIMOS_ZENOH_NAMESPACE`, with no namespace there are no events.
- Saved GlobalConfig and module config live in `$XDG_STATE_HOME/dimos/gateway/settings.json`.
- `annotations.yaml` describes every robot and its blueprints (`GET /robots`).
- Anything that imports a blueprint runs in a child process (`introspect.py`), so a blueprint that hangs or crashes
  can't take the gateway down.

See [gateway-how-to.md](gateway-how-to.md) for examples.

```sh
python -m pytest experimental/gateway/tests
python -m mypy experimental/gateway
```
