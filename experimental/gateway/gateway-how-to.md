# Dimos gateway: how do I...?

Snippets run in an app page served by Desktop at `/apps/<name>/`, so `../../dimos-gateway/` is the gateway
(Desktop serves it under `/dimos-gateway`; the gateway's own routes have no prefix). An app's `dimos.yaml` lists
each gateway route it calls:

```yaml
uses:
    "@dimos-gateway":
        - GET /blueprints
        - POST /runs
```

## How do I list blueprints?

```js
const { blueprints, scan } = await (await fetch("../../dimos-gateway/blueprints")).json()
// blueprints: [{ name, kind, ...once a scan finished: importable, import_error, suggested_extras, robot, modules }]
// scan: { job, state, scanned, finished_at, age_seconds, stale }: the scan is cached (on disk too) and only a POST
// catalog/scan changes it; stale says the checkout or venv changed since
```

To scan (the first time, or after a change) and follow it:

```js
const { job } = await (await fetch("../../dimos-gateway/catalog/scan", { method: "POST" })).json() // 202
const { done, ok, progress } = await (await fetch(`../../dimos-gateway/jobs/${job}`)).json() // progress: { done, total, current }
```

## How do I launch a blueprint?

```js
await fetch("../../dimos-gateway/runs", {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ blueprint: "unitree-go2", replay: true, overrides: { global: { n_workers: 4 } } }),
}) // answers at once, phase "starting"
```

## How do I see what's running?

```js
const { runs, launch } = await (await fetch("../../dimos-gateway/runs")).json() // runs: [{ run_id, blueprint, pid }], launch.phase
getZenoh().subscribeDimos("launch", (launch) => show(launch.phase)) // live phase changes; getZenoh from ./dim-app/mod.js
```

## How do I stop a blueprint?

```js
await fetch("../../dimos-gateway/runs/stop", { method: "POST", body: JSON.stringify({ runId }) }) // launch.runId is set from the start
```

## How do I relaunch?

```js
await fetch("../../dimos-gateway/runs/restart", { method: "POST", body: JSON.stringify({ runId }) }) // the gateway's launch again with the config saved now; another run with its own command line
```

## How do I read a run's logs?

```js
const page = await (await fetch("../../dimos-gateway/runs/latest/log?level=warning&limit=200")).json() // { records, offset }
const newer = await (await fetch(`../../dimos-gateway/runs/latest/log?after=${page.offset}`)).json() // tail
```

## How do I list a blueprint's modules, their streams and RPCs?

```js
const { modules } = await (await fetch("../../dimos-gateway/blueprints/unitree-go2")).json()
// modules[i]: { name, rpcs, skills, streams: [{ name, type, direction, topic }] }
```

## How do I list every module's inputs and outputs?

```js
const { modules } = await (await fetch("../../dimos-gateway/catalog")).json() // [{ name, inputs, outputs, skills }], from the last POST catalog/scan; its scan.stale says if the checkout changed since
```

## How do I list skills?

```js
const { skills } = await (await fetch("../../dimos-gateway/skills")).json() // the running blueprint's: [{ name, module, params }]
```

## How do I call a skill?

```js
const result = await (await fetch("../../dimos-gateway/skills/call", {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ skill: "execute_sport_command", args: { command_name: "FrontJump" } }),
})).json() // { ok, text }; this acts on the robot, so call it only from a user's action
```

## How do I list robots and their recommended blueprints?

```js
const { robots } = await (await fetch("../../dimos-gateway/robots")).json()
robots.go2.recommended // ["unitree-go2-basic", ...]
robots.go2.blueprints["unitree-go2"] // { title, description, starter, recommended_config }
```

## How do I get dimos's python?

```js
const { python } = await (await fetch("../../dimos-gateway/info")).json() // run `${python} -c 'import dimos'`
```

## How do I read a blueprint's config?

```js
const { config: { modules } } = await (await fetch("../../dimos-gateway/blueprints/unitree-go2")).json()
// modules[i]: { module, args: [{ name, type, default, value }] }
```

## How do I save a blueprint's config?

```js
await fetch("../../dimos-gateway/blueprints/unitree-go2/config", {
    method: "PUT",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ overrides: { go2connection: { lidar: false } } }), // replaces the saved config; null drops a field
}) // applies at the next launch
```
