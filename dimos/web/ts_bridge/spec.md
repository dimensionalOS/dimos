## Web side usage

```ts
import { Dimos } from "/dimos.js" // will be on JSR
import { decode } from "https://esm.sh/jsr/@dimos/msgs@0.1.4"

const app = await Dimos.connect({
    decode, // `decode` doesn't know of some datatype it will be a raw `Uint8Array`
    dimosWs: {
        host: location.hostname,
        port: Number(location.port),
        // OPTIONAL:
        // whitelist: ["/tf"],
        // blacklist: ["/global_map"],
    },
})
```


```ts
//
// subscribe
//
const unsub = app.subscribe("/tf", (message) => console.log(message.data, message.ts))
unsub()

// set QoS dynamically
app.setQos("/tf", {
    rate: 30, // tells backend to rate limit (not global limit, just this frontend)
    reliability: "best_effort",     // or "reliable" — buffer in order instead of dropping
    durability: "transient_local",  // new subscribers get the last value right away
    depth: 1,                       // outbox buffer size
})

// get everything (debug stream-name typos)
app.subscribeAll((m) => console.log(m.stream, m.data))

// just grab one (resolves null after timeoutMs)
const tf = await app.peek("/tf", { timeoutMs: 1000 })

//
// Rpc
//
await app.modules.GO2Connection.standup()
const pose = await app.modules.Localization.get_pose()
```

## Module Side Usage

Any module can provide a frontend without starting its own server

```py
from dimos.core.module import web_module, web_init

@web_module
class MyRobotThing(Module):
    # ONLY THIS method is needed for a frontend
    @web_init
    def _web_init(self) -> dict[str, str | Path]:
        return {
            # will be available at http://localhost:9669/MyRobotThing/index.html
            "index.html": Path(__file__).parent / "index.html",   # served at /MyRobotThing
            "icon":       Path(__file__).parent / "logo.svg",     # favicon — any format (SVG/PNG = high-res)
        }

    #
    # normal blah-blah module stuff
    #
    color_image: Out[Image]

    @rpc
    def start(self):
        super().start()
        logger.log("blah blah blah")

```

## Dimos Core

### One Websocket Server (not specific to JS/TS) `DimosWebsocket`

```py
DimosWebsocket.blueprint(
    port=9669, # pulls from global_config if not given
    host="12.01.20.102", # pulls from global_config if not given
    whitelist=[
        "global_map",
    ],
    # alternative:
    # blacklist=[
    #
    # ],
)
```

- This subscribes-all like rerun bridge
- Backend global blacklist/whitelist
- Per-connection QoS/blacklist/whitelist (to avoid flooding frontend)
- Is a general static webpage server

Wire it up alongside the server and your robot, then open the page:

```py
blueprint = autoconnect(
    MyRobot.blueprint(),
    DimosWebsocket.blueprint(),
    MyRobotThing.blueprint()
)
# → http://localhost:9669/MyRobotThing   (it loads /dimos.js and connects same-origin)
```
