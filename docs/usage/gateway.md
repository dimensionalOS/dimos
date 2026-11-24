# The dimos gateway

The stable API for building apps on top of dimos: what could run (`/catalog`), what is running (`/active`) and what
ran (`/history`). Desktop starts it from dimos.yaml's `start:` and serves it at `/dimos-gateway/`; an app page at
`/apps/<name>/` reaches it as `../../dimos-gateway/`. `GET /openapi.json` has every route with its request and answer;
`GET /info` `api` is the API version (released as `desktop-gateway-v<version>` tags).

## Rules

1. **Relative URLs only:** `../../dimos-gateway/active`, never a host or port.
2. **Declare every route you call** in your app's `dimos.yaml`: `uses: {"@dimos-gateway": [GET /active, POST /active/launch]}`.
3. **Config is per launch.** The gateway saves none: keep your defaults and send all of them in each
   `POST /active/launch` `{blueprint, config: {global, modules}}`. `GET /catalog/blueprints/{name}/config` is the schema.
4. **Secrets** (keys like `*_key`, `*token*`) reach dimos through the environment, never its command line, and come
   back as `•••`.
5. **One blueprint runs at a time.** `/active` routes answer 409 when nothing runs; `POST /active/launch` answers 409
   when something does.
6. **A module instance's id** is its namespace and lowercase class joined by `:` (`robot0:go2connection`, or
   `go2connection` with no namespace).
7. **RPCs are for buttons.** `POST /active/modules/{id}/rpcs/{name}` calls the module directly, not through MCP, so a
   skill's capability holds don't apply; call one only from a user's action. Agents use the python API.
8. **The catalog is a cache.** It changes only when a `POST /catalog/scan` job finishes (`?timeout=` per blueprint);
   follow the job at `GET /jobs/{job}`. What is read from the code on demand (a blueprint's or module's detail, source
   and `config`, and `extras`) is reused for ten minutes, and read again once a scan is asked for or the checkout
   changes (an edit, a commit, an install).

## Routes

| | |
| -- | -- |
| `/catalog` | `blueprints`, `modules`, `rpcs`, `robots`, `config` (global schema), `scan`, each `{name}/config` schema and `{name}/source` (its .py file; a module's `?blueprint=` reads it from that blueprint's live run's checkout) |
| `/active` | the running blueprint; `launch`, `exec`, `relaunch`, `stop`, `log`, `blueprint`, `modules/{id}` (+ `restart`, `stop`, `rpcs/{name}`, `streams`), `skills`, `streams` (+ `{name}/peek`), `stats` |
| `/history` | every run with a log; `{runId}` (or `latest`), `{runId}/logs`, `recordings` (its directory) |
| `/cloud` | `account`, `login` (a device login; `?after=<state>` waits for it to leave that state; DELETE cancels it, or logs out with none pending), `login/page` (a login page apps embed), `uploads` (queue a recording, retry, cancel) |
| `/agent` | On-robot DimOS Agent conversations, submissions, images, and scheduled jobs |
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

## On-robot DimOS Agent

From the checkout the gateway serves, build the pi-durable agent with `dimos agent version` and start it with
`dimos agent gateway --daemon`.
Desktop apps declare these HTTP operations in `uses["@dimos-gateway"]`. The gateway connects to
the agent's private Unix socket. An offline agent returns 503.

| Method | Path | Request or result |
| --- | --- | --- |
| GET, POST | `/agent/conversations` | List summaries or create with `{request, cwd?}` and receive `{conversation}`. |
| GET | `/agent/conversations/{id}` | Read `transcript.items`, its `omitted` count, and `turn` with the active run and partial text. |
| POST | `/agent/conversations/{id}/submissions` | Send `{request, text, images, whenBusy}` and receive `{submission}`. Each image has `type: "image"`, `mimeType`, and base64 `data`. Use `steer` or `followUp` for `whenBusy`. |
| GET | `/agent/conversations/{id}/submissions/{submission}` | The durable `record` and its `answer` entry when done. An unanswered record includes its reason. |
| POST | `/agent/conversations/{id}/abort` | Send `{request}` to stop the turn. The result is the user's withdrawn `{mode, text}` items in queue order. |
| GET | `/agent/conversations/{id}/images/{entry}/{message}/{block}` | Use the address in a transcript image reference to fetch its stored image. |
| GET, POST | `/agent/jobs` | List schedules or create one with `{name, conversation, cron, timeZone, action}`. Rows include `nextDue`, `upcoming`, and recent `runs`. |
| PUT, DELETE | `/agent/jobs/{name}` | Replace the definition or delete the schedule. |
| POST | `/agent/jobs/{name}/runs` | Send `{request}` to run now and receive the job outcome. |

Generate a request UUID per action and reuse it for retries. A schedule action is either
`{kind: "notify", text}` or `{kind: "skill", skill, arguments}`. A null `timeZone` uses the agent's zone.

Subscribe to `<namespace>/dimos/events/agent/*` to receive all agent events, or select one topic.

| Topic under `<namespace>/dimos/events/` | Client action |
| --- | --- |
| `agent/chat` | Filter on the payload’s `conversation` ID. Replace the displayed partial reply with `turn.text`, merge committed `entries` by entry ID, and refresh the conversation when `turn` is null. |
| `agent/conversations` | Refresh `GET /agent/conversations`. |
| `agent/jobs` | Refresh `GET /agent/jobs`. |

On open or reconnect, subscribe first, then read `GET /agent/conversations/{id}` for the current transcript and turn.

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
