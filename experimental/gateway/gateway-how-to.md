# Dimos gateway: how do I...?

Snippets run in an app page served by Desktop at `/apps/<name>/`, so `../../dimos/` is the gateway. Each lists the
line its `dimos.yaml` needs under `uses: "@dimos-gateway":` (paths relative to `/dimos`). Full reference:
[gateway-api.md](/docs/usage/gateway-api.md).

```yaml
uses:
    "@dimos-gateway":
        - GET /blueprints # one line per endpoint the app calls
```

## How do I list blueprints?

```js
const { blueprints } = await (await fetch("../../dimos/blueprints")).json() // [{ name, kind, importable }]
```

`- GET /blueprints`

## How do I launch a blueprint?

```js
await fetch("../../dimos/runs", {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ blueprint: "unitree-go2", replay: true, overrides: { global: { n_workers: 4 } } }),
}) // answers at once, phase "starting"
```

`- POST /runs`

## How do I see what's running?

```js
const { runs, launch } = await (await fetch("../../dimos/runs")).json() // runs: [{ run_id, blueprint, pid }], launch.phase
getZenoh().subscribeDimos("launch", (launch) => show(launch.phase)) // live phase changes; getZenoh from ./dim-app/mod.js
```

`- GET /runs`

## How do I stop a blueprint?

```js
await fetch("../../dimos/runs/stop", { method: "POST" }) // or body { runId } for a specific run
```

`- POST /runs/stop`

## How do I relaunch?

```js
await fetch("../../dimos/runs/restart", { method: "POST" }) // the last launch again, with the config saved now
```

`- POST /runs/restart`

## How do I read a run's logs?

```js
const page = await (await fetch("../../dimos/runs/latest/log?level=warning&limit=200")).json() // { records, offset }
const newer = await (await fetch(`../../dimos/runs/latest/log?after=${page.offset}`)).json() // tail
```

`- GET /runs/{runId}/log`

## How do I list topics and their rates?

```js
const { topics } = await (await fetch("../../dimos/topics/rates")).json() // [{ topic, type, hz, bps, lastSeen }]
```

`- GET /topics/rates`

## How do I list a blueprint's modules, their streams and RPCs?

```js
const { modules } = await (await fetch("../../dimos/blueprints/unitree-go2")).json()
// modules[i]: { name, rpcs, skills, streams: [{ name, type, direction, topic }] }
```

`- GET /blueprints/{name}`

## How do I list every module's inputs and outputs?

```js
const { modules } = await (await fetch("../../dimos/catalog")).json() // [{ name, inputs, outputs, skills }]; slow the first time
```

`- GET /catalog`

## How do I list skills?

```js
const { skills } = await (await fetch("../../dimos/skills")).json() // the running blueprint's: [{ name, module, params }]
```

`- GET /skills`

## How do I call a skill?

```js
const result = await (await fetch("../../dimos/skills/call", {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ skill: "execute_sport_command", args: { command_name: "FrontJump" } }),
})).json() // { ok, text }
```

`- POST /skills/call` (acts on the robot: only from a user's action)

## How do I decode dimos messages in a page?

```js
import { DimApp } from "./dim-app/mod.js"

const app = new DimApp({ msgDecodeEndpoint: "../../dimos/msgs.js" })
app.subscribe("odom", (odom) => console.log(odom.pose.pose.position)) // decoded
const twist = app.msgs.geometry_msgs.Twist.encode({ linear: { x: 0.3 } }) // bytes
```

`- GET /msgs.js`

## How do I decode them in a Deno backend?

```js
import { dimContext } from "./dim-app/source/backend.js"

const msgs = await import(new URL("/dimos/msgs.ts", dimContext().desktopUrl).href)
const odom = msgs.decodeChannel("dimos/odom/nav_msgs.Odometry", bytes)
```

`- GET /msgs.ts`

## How do I list robots and their recommended blueprints?

```js
const { robots } = await (await fetch("../../dimos/robots")).json()
robots.go2.recommended // ["unitree-go2-basic", ...]
robots.go2.blueprints["unitree-go2"] // { title, description, starter, recommended_config }
```

`- GET /robots`

## How do I get dimos's python?

```js
const { python, env, example } = await (await fetch("../../dimos/python")).json() // run `${python} -c '...'` with env set
```

`- GET /python`

## How do I read a blueprint's config?

```js
const { modules } = await (await fetch("../../dimos/blueprints/unitree-go2/config")).json()
// modules[i]: { module, args: [{ name, type, default, value }] }
```

`- GET /blueprints/{name}/config`

## How do I save a blueprint's config?

```js
await fetch("../../dimos/blueprints/unitree-go2/config", {
    method: "PUT",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ overrides: { go2connection: { lidar: false } } }), // replaces the saved config; null drops a field
}) // applies at the next launch
```

`- PUT /blueprints/{name}/config`
