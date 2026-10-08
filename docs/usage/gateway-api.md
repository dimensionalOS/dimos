# The dimos gateway's HTTP API

How do I launch a blueprint, read its logs, call a skill...? See [gateway-how-to.md](/docs/usage/gateway-how-to.md).

`python -m dimos.gateway` serves the `/dimos` HTTP API that dimOS Desktop uses: blueprints, global config, runs and
their logs, events, and Dimensional cloud uploads. Desktop starts it on a unix socket and forwards `/dimos/...` to it
unchanged.

## Starting it: `DIMOS_GATEWAY`

Desktop runs [`dimos.yaml`](/dimos.yaml#L89)'s `start:` (`.venv/bin/python -m dimos.gateway --detach`) in the checkout with
one variable, `DIMOS_GATEWAY`, a JSON object; every field is optional:

| Field             | What                                                                                     |
| ----------------- | ---------------------------------------------------------------------------------------- |
| `socket`          | the unix socket to serve on (default `<dimos state>/gateway/dimos-gateway.sock`)         |
| `dimosDir`        | the checkout runs are launched from (default: the one this code is in)                   |
| `zenoh.namespace` | Desktop's `<ns>`: events go on `<ns>/dimos/events/<type>`                                |
| `zenoh.connect`   | the zenoh endpoints to dial, a list (default: dimos's `zenoh_connect`)                   |
| `desktopUrl`      | where Desktop answers (its shell tool runs extras installs)                              |
| `recordingsDir`   | the recordings folder Desktop gives its apps                                             |
| `dimosRange`      | the dimos versions Desktop works with (`/dimos/info`'s `inRange`; launches outside fail) |

`socket:` (`--print-socket`) prints where it serves. `--no-zenoh` keeps events on SSE only.

## What it offers: `provides:`

[`dimos.yaml`](/dimos.yaml#L27)'s `provides:` lists every endpoint, the way an app declares its own: method, path
relative to `/dimos/` and a one-line description. Desktop reads it per tag to check apps' `uses: "@dimos-gateway"`.
It's generated from the routes (each `route_doc` summary is its description): after adding or changing a route, run

```sh skip
python -m dimos.gateway --write-provides
```

A test fails while it's stale. Request and answer models live in [`dimos/gateway/models.py`](/dimos/gateway/models.py);
the gateway serves no OpenAPI document or Swagger page.

## Groups

| Tag             | What                                                                                 |
| --------------- | ------------------------------------------------------------------------------------ |
| `server`        | liveness, the checkout it serves, where things are, stopping it                      |
| `blueprints`    | the blueprint list, a blueprint's modules and config, the catalog of modules/skills  |
| `global-config` | dimos's GlobalConfig schema and defaults, and Desktop's saved overrides               |
| `runs`          | launching and stopping blueprints, and the live runs                                  |
| `logs`          | a run's structured log, with filters and tailing                                     |
| `cloud`         | the Dimensional cloud login (device flow) and account                                |
| `uploads`       | the upload queue, and which recordings are already in the cloud                      |
| `events`        | the event payloads, and the deprecated SSE stream                                    |
| `discovery`     | the discovery cache: blueprints, modules, their config, message types, robot ranking |
| `docs`          | the guide to adding your own robot, and links into the docs site                     |
| `extras`        | dimos's optional extras, which are installed, and installing more                    |
| `jobs`          | long jobs (an extras install): their output, live on zenoh and as a snapshot         |
| `skills`        | the running blueprint's skills: list them, call one, and an MCP server for agents    |

## Launch diagnostics

A launch's `steps` and `problems` are stable codes with data: clients own the wording. The gateway reads them from the
run's structured log (`main.jsonl`), not from the console, and needs nothing added to dimos for it:

- **Steps:** the messages dimos logs as it starts: its startup line, `Building the blueprint`, `Starting the
  modules`, one `Deployed module.` per module and `Blueprint started` (`test_diagnose.py` checks dimos still logs
  each). dimos doesn't log how many modules it will start, so `starting_modules` counts only the deployed ones.
- **Problems:** an exception is logged with its traceback; the gateway reads the exception classes and an errno or
  SQLite "can't open" from it and maps them to a problem code in
  [`dimos/gateway/diagnose.py`](/dimos/gateway/diagnose.py). Refusals dimos only prints (bad arguments, an unknown
  blueprint, an unmet requirement) have no record: the launch's `error` is then its output's last line.

The gateway starts `dimos run` with `DIMOS_RUN_LOG_DIR` set, so even the records from before the run has an id are in a
file it can read; after that it follows the run to `LOG_DIR/<run id>`.

## Events

The gateway publishes its events on zenoh at `<ns>/dimos/events/<type>` (`<ns>` is Desktop's namespace): `launch`,
`log`, `upload`, `uploads`, `upload-removed`, `cloud-login`, `discovery`, `job` and `blueprints` (the blueprint list changed: `added`, `removed`; the gateway watches dimos/robot and site-packages itself). Each payload is a component schema in the document
(`LaunchEvent`, ... ; `DimosEvent` is any of them) with its zenoh key in `x-zenoh-key`. `GET /dimos/events` streams
the same events as server-sent events, and is deprecated.

A job's output lines are on `<ns>/dimos/jobs/<job>` (`{type: "line", n, line}`, then `{type: "done", ok, error,
failure, lines}`), the same shape as Desktop's own jobs; `GET /dimos/jobs/{job}/log?after=<n>` is the snapshot.

## Discovery

When the gateway starts it imports every blueprint in dimos's registry ([`dimos/robot/all_blueprints.py`](/dimos/robot/all_blueprints.py))
and every registry module once, in child processes ([`dimos/gateway/discover.py`](/dimos/gateway/discover.py)): a blueprint that
hangs or crashes its process costs only itself. The answer (whether each blueprint imports and why not, its modules
and their streams and topics, every module's config fields, the message types) is saved under
`<state>/dimos/gateway/discovery/`, keyed by the checkout's commit, its dirty files and the installed packages, so a
restart answers at once. The key is checked every 30 s and after an extras install; when it changes the old answer is
served (`stale: true`) until the new one is in. See [`dimos/gateway/discovery.py`](/dimos/gateway/discovery.py#L25).

`GET /dimos/robots/{robot}/modules` ranks the modules of a robot's blueprints by how specific they are to it:

```
score = (robot's blueprints using the module / robot's blueprints) * ln(robots / robots using the module)
```

so the robot's own connection module comes first and a module every robot uses scores 0. Robots are
[`dimos/gateway/robots.json`](/dimos/gateway/robots.json)'s, and only blueprints that import count.

## Topic rates

`GET /dimos/topics/rates` lists every topic the gateway has heard on the bus since it started: rate and throughput
over the last 2 s, messages so far, seconds since the last one. It listens to zenoh `dimos/**` from its start, so a
topic published once (at a blueprint's startup) is listed, and one gone quiet stays (0 Hz). RPC calls (zenoh queries)
and LCM-only traffic aren't there. The blueprint view's side panel shows it, with the blueprint's own topics that
nothing has published yet.

## Skills

`GET /dimos/skills` lists the skills (a module's `@skill` methods) of the running blueprint, agent or no agent: name,
module, docstring, params as JSON Schema (the one McpServer gives an agent), `lifecycle` and the capabilities it
`uses`. The gateway reads them over dimos's module RPC, as `dimos.porcelain` does: `Coordinator/list_modules`, then
each module's `get_skills`; a module that doesn't answer is in `errors`, and nothing running is an empty list with
`run: null`. `POST /dimos/skills/call {skill, args, module?, runId?}` checks `args` against `params`, calls
`<module>/<skill>` over the same RPC (as the coordinator calls a module's `start`), and answers with its text once it
returns (`via: rpc`). A skill that holds a capability goes through the run's McpServer instead when one answers
(`via: mcp`), so the capability locks its agent goes by cover the call too. It acts on the robot: the gateway calls a
skill only when asked to.

```sh skip
sock=$(.venv/bin/python -m dimos.gateway --print-socket)
curl -s --unix-socket "$sock" http://gateway/dimos/skills | jq '.skills[] | {name, module}'
curl -s --unix-socket "$sock" -X POST http://gateway/dimos/skills/call -H 'content-type: application/json' \
    -d '{"skill": "execute_sport_command", "args": {"command_name": "FrontJump"}}'
```

`POST /dimos/mcp` is an MCP server (Streamable HTTP, JSON answers) with two tools, `list_skills` and `call_skill`,
for an agent: they don't change with what runs, so a session that connected before the blueprint started still reaches
its skills. dimcode's `dimcode desktop --mcp-url <Desktop>/mcp` connects it too, as the MCP endpoint `skills`.

## Dimos's python, for an agent

`GET /dimos/python` says which python dimos runs with, so an agent (dimcode) can use dimos's python API: `python`
(absolute, usually `<checkout>/.venv/bin/python`), `command` (the argv to start it), `env` (what to set for `import
dimos` to find the checkout; usually empty, else `PYTHONPATH`), `dimosDir`, `version`, `dimosVersion` and a runnable
`example`. The gateway checks it once, by running `import dimos` in it from another folder, and caches the answer.
Run one-liners as `<python> -c '...'` and scripts as `<python> script.py`, with `env` set:

```sh skip
py=$(curl -s --unix-socket "$sock" http://gateway/dimos/python | jq -r .python)
"$py" -c 'import dimos; print(dimos.__file__)'
```

## The blueprint view

`GET /dimos/blueprint_view?name=<blueprint>` is dimOS Desktop's whole blueprint Details modal: its top bar (phase,
Relaunch, Stop, Configure, Show code, Logs, close), Topic rates, the modules and the module graph, all plain JS and CSS
in `dimos/gateway/blueprint_view/`, so it changes with the dimos checkout, not with Desktop. Framed by Desktop, it posts
`dimos:chrome`; a Desktop that then shows only the frame answers `dimos:chrome-ok` and the page shows its bar (an older
Desktop keeps its own bar, so there's never two).

## Decoding messages in a page

`GET /dimos/msgs.js` is an ES module that decodes and encodes every dimos message, for pages and apps with no build
step; `GET /dimos/msgs.ts` is the same module as TypeScript (an interface per message), for Deno and TypeScript. Both
are generated from the message classes under dimos/msgs and their dimos_lcm schemas
([`dimos/gateway/msgs/codegen.py`](/dimos/gateway/msgs/codegen.py)), and a test fails while they're stale:

```sh skip
python -m dimos.gateway.msgs           # check, and list the messages without a schema
python -m dimos.gateway.msgs --write   # regenerate msgs.ts, and msgs.js from it (needs deno)
```

With the [zenoh-gateway](https://github.com/jeff-hykin/zenoh-gateway) browser client, where dimos publishes each
topic on the zenoh key `dimos/<topic>/<package>.<Type>`:

```js
import { connect } from "./zenoh_gateway.js" // zenoh-gateway's client/zenoh_gateway.ts, however your app ships it
import { decodeMessage, geometry_msgs } from "../../dimos/msgs.js"

const z = await connect(url)
// the type in the key picks the decoder
z.subscribe("dimos/odom/**", {}, (message) => console.log(decodeMessage(message).pose.position))
// publish a Twist: fields left out are zero
await z.put(geometry_msgs.Twist.zenohKey("dimos/cmd_vel"), geometry_msgs.Twist.encode({ linear: { x: 0.3 } }))
```

- `decode(bytes)` decodes by the frame's 8-byte fingerprint; `decodeChannel(channelOrKey, bytes)` lets the type an
  LCM channel (`/odom#nav_msgs.Odometry`) or zenoh key names win over it; `decodeMessage(message)` does that for a
  zenoh-gateway message (`undefined` for a delete).
- `geometry_msgs.PoseStamped` and the like: `.decode(bytes)`, `.encode(value)`, `.zenohKey(topic)`,
  `.lcmChannel(topic)`. `getTypeNames()` lists them. Values are plain objects in wire order; `int64_t` is a bigint and
  `byte[]` a view into the frame.
- A few dimos messages are hand-written, with no LCM schema (`sensor_msgs.JointCommand`, `trajectory_msgs.JointTrajectory`
  and others: `getMissingTypes()`, and the warnings the check prints). Decoding one throws until a page adds it with
  `register(name, fingerprint, decode, encode?)`.
