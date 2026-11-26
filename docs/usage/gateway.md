# The dimos gateway

The stable API for building apps on top of dimos: what could run (`/catalog`), what is running (`/active`) and what
ran (`/history`). Desktop starts it from dimos.yaml's `start:` and serves it at `/dimos-gateway/`; an app page at
`/apps/<name>/` reaches it as `../../dimos-gateway/`. `GET /openapi.json` has every route with its request and answer;
`GET /info` `api` is the API version (released as `desktop-gateway-v<version>` tags).

The catalog reads blueprints from the checkout it serves. It uses the default options for blueprint choices and also accepts blueprints from checkouts that predate choices.

## Rules

1. **Relative URLs only:** `../../dimos-gateway/active`, never a host or port.
2. **Declare every route you call** in your app's `dimos.yaml`: `uses: {"@dimos-gateway": [GET /active, POST /active/launch]}`.
3. **Config is per launch.** The gateway saves none: keep your defaults and send all of them in each
   `POST /active/launch` `{blueprint, config: {global, modules}}`. `GET /catalog/blueprints/{name}/config` is the schema.
   When supplied, `global` and `modules` must be objects and `secrets` must be a list of field paths.
   Packaged paths appear under their asset names. Omit an unchanged path to keep its lazy lookup, or supply a local file
   to replace it. A collection containing packaged paths is a read-only preview (`json_compatible: false`).
4. **Secrets** (keys like `*_key`, `*token*`) reach dimos through the environment, never its command line, and come
   back as `•••`.
5. **One blueprint runs at a time.** `/active` routes answer 409 when nothing runs; `POST /active/launch` answers 409
   when something does.
6. **A module instance's id** is its namespace and lowercase class joined by `:` (`robot0:go2connection`, or
   `go2connection` with no namespace).
7. **RPCs are for buttons.** `POST /active/modules/{id}/rpcs/{name}` calls the module directly, not through MCP, so a
   skill's capability holds don't apply; call one only from a user's action. Agents use the python API.
8. **A running module's config can be read.** `GET /active/modules/{id}/config` answers
   `{config: {field: value}, readonly: [field]}`: a readonly field is a secret (sent as `•••`) or one JSON can't carry
   (its text). `POST /active/modules/{id}/restart` with `{"config": {field: value}}` restarts it with those changed.
9. **The catalog is a cache.** It changes only when a `POST /catalog/scan` job finishes (`?timeout=` per blueprint);
   follow the job at `GET /jobs/{job}`. What is read from the code on demand (a blueprint's or module's detail, source
   and `config`, and `extras`) is reused for ten minutes, and read again once a scan is asked for or the checkout
   changes (an edit, a commit, an install).

For Relaunch with edited settings, send one launch request to `POST /active/replace`.
The gateway validates the replacement before stopping the current process. Discover this route in
the gateway manifest or OpenAPI document. On an older gateway, leave the run active.
A bodyless `POST /active/relaunch` repeats the previously saved launch.
Both answer 400 when the checkout rejects the settings and 500 (`could not validate the launch`) when its
check could not run; the current run keeps running either way. Validation does not hold up
`POST /active/stop`: after a stop, or any other change to what runs, during validation, the replacement
answers 409 and starts nothing.

## Routes

| | |
| -- | -- |
| `/catalog` | `blueprints`, `modules`, `rpcs`, `robots`, `config` (global schema), `scan`, each `{name}/config` schema and `{name}/source` (its .py file; a module's `?blueprint=` reads it from that blueprint's live run's checkout) |
| `/active` | the running blueprint; `launch`, `exec`, `relaunch`, `stop`, `log`, `blueprint`, `modules/{id}` (+ `config`, `restart`, `stop`, `rpcs/{name}`, `streams`), `skills`, `streams` (+ `{name}/peek`), `stats` |
| `/history` | every run with a log; `{runId}` (or `latest`, the newest across every registered checkout), `{runId}/logs`, `recordings` (its directory) |
| `/cloud` | `account`, `login` (a device login; `?after=<state>` waits for it to leave that state; DELETE cancels it, or logs out with none pending), `login/page` (a login page apps embed), `uploads` (queue a recording, retry, cancel) |
| views | `msgs.js` (decoders for dimos messages), `msgs.d.ts` (its types) |
| other | `info`, `extras` (+ `install`), `jobs/{job}`, `docs/search`, `server/stop` |

## Examples

```js
const gateway = "../../dimos-gateway"
const post = (path, body) =>
    fetch(`${gateway}/${path}`, { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify(body) })

await post("active/launch", { blueprint: "unitree-go2", config: { global: { replay: true } } }) // phase "starting"
const now = await (await fetch(`${gateway}/active`)).json() // { running, runId, blueprint, launch: { phase, output, ... } }
const { modules } = await (await fetch(`${gateway}/active/modules`)).json() // [{ id, class, rpcs, streams }]
await post("active/modules/unitreeskillcontainer/rpcs/execute_sport_command", { kwargs: { command_name: "FrontJump" } })
const page = await (await fetch(`${gateway}/active/log?level=warning&limit=200`)).json() // then ?after=${page.offset}
```

Live launch phases (a run started in a terminal starting or ending too; and added or removed blueprints, and upload progress) also arrive on zenoh at `<namespace>/dimos/events/<type>`. The types are `launch`, `blueprints`,
`upload`, `uploads`, `upload-removed`, `cloud-account`, `cloud-login`, `stats` and `log`.

`stats` and `log` say something changed and carry no data, so a page refetches instead of polling. `stats` fires on each
dtop sample (re-read `GET /active/stats`); `log` fires when a log grows, with `run` set to the run's id, or `launch` for
the gateway's launch log. To wait for a device login, `GET /cloud/login?after=<state>` answers once the login
leaves that state.

Decode dimos messages in a page with `msgs.js` (Deno and editors pick up its types, `msgs.d.ts`, from its
`X-TypeScript-Types` header):

```js
const { decodeChannel, geometry_msgs } = await import(`${gateway}/msgs.js`)
const odom = decodeChannel("dimos/odom/geometry_msgs.PoseStamped", bytes) // bytes: one message off zenoh or LCM
const twist = geometry_msgs.Twist.encode({ linear: { x: 0.3 }, angular: { z: 0.5 } }) // a Uint8Array to publish
```

## Running it yourself

```sh skip
.venv/bin/python -m experimental.gateway --detach      # in the background, on its unix socket
.venv/bin/python -m experimental.gateway --port 8123   # in the foreground, also on --host (default 127.0.0.1):8123
```

- Settings come from `DIMOS_APP` (`socket`, `recordingsDir`, `zenohNamespace`, `zenohConnect`), else `<home>/config.yaml`
  (`home` from `DIMOS_APP`, else `~/.dimos`): `dimos-gateway.socket`, `zenoh.namespace`, `zenoh.connect`,
  `zenoh.gossip`, `topics.enabled` and `recordings.dir`, read once at start: Desktop writes that file and restarts the
  gateway when they change.
- Its tuning numbers are in `experimental/gateway/constants.py`; robot and blueprint descriptions in
  `experimental/gateway/annotations.yaml`.
- Anything that imports a blueprint runs in a child process, so a blueprint that hangs or crashes can't take it down.
- Tests: `python -m pytest experimental/gateway/tests` and `python -m mypy experimental/gateway`.
