# dimos gateway (experimental)

`python -m experimental.gateway --socket <path> --dimos-dir <checkout>` serves the `/dimos` HTTP API that dimOS Desktop,
apps and agents use: blueprints, config, runs and their logs, skills, Dimensional cloud uploads and the first-run
discovery. `--port 8123` also serves it on `127.0.0.1:8123`; `--detach` starts it in the background.

- `GET /dimos/openapi.json` lists every route (FastAPI's own document).
- Events go on zenoh at `<ns>/dimos/events/<type>` (`launch`, `blueprints`, `upload`, `uploads`, `upload-removed`,
  `cloud-login`); the namespace is `--zenoh-namespace` or `$DIMOS_ZENOH_NAMESPACE`, with no namespace there are no events.
- Saved GlobalConfig and module config live in `$XDG_STATE_HOME/dimos/gateway/settings.json`.
- `annotations.yaml` describes every robot and its blueprints (`GET /dimos/robots`).
- `routes/` holds the routers; the rest is what they call. Anything that imports a blueprint runs in a child process
  (`introspect.py`), so a blueprint that hangs or crashes can't take the gateway down.

See [gateway-how-to.md](gateway-how-to.md) for examples.

```sh
python -m pytest experimental/gateway/tests
python -m mypy experimental/gateway
```
