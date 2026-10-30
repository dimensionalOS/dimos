# `dimos host`

| Command | Does |
|---|---|
| `dimos host start` | Start this machine's Host in the background (`-f`: here, in the foreground) |
| `dimos host stop` | Stop it |
| `dimos host restart` | Restart it, e.g. after changing its settings |
| `dimos host sync` | Check out this machine's commit on every other Host and restart them into it |
| `dimos host status` | Whether it runs, and what it advertises |
| `dimos host ls` | Every Host this one can see (`-p tcp/host:7447`: probe one more, `--json`) |
| `dimos host describe <host>` | One Host's tags, revision and active runs |
| `dimos host doctor` | Check the setup; `--fix` repairs what it can |
| `dimos host config` | Show this Host's settings; `key=value` sets one, `key=` removes it |
| `dimos host install` | Install the `dimos-host` systemd user unit, so `start` uses it (`--config key=value`: set first, `--start`: and start it) |
| `dimos host uninstall` | Remove the unit |
| `dimos host id` | This machine's Host ID |

To start a Host at boot: `dimos host install --start --config listen=iface/<name>`, then
`sudo loginctl enable-linger $USER`.

## Settings

`dimos host config key=value` writes `HOST__<KEY>` to the checkout's `.env`, which the Host
reads; `dimos host config` alone shows them. Run `dimos host restart` after changing them.

| Setting | Default | |
|---|---|---|
| `HOST__NAME` | hostname | shown in `ls`, usable as `hosted(host=...)` |
| `HOST__TAGS` | | extra tags, `go2,site=athens` |
| `HOST__LISTEN` | `tcp/127.0.0.1:7447` | router endpoints, or `iface/<name>[:<port>]` |
| `HOST__CONNECT` | | routers to always link to |
| `HOST__AUTODISCOVERY` | `false` | scout for other Hosts and link to them |
| `HOST__GOSSIP` | `true` | link directly to the Hosts a linked Host knows of |
| `HOST__STARTUP_TIMEOUT` | `180` | seconds a fragment may take to start |
| `HOST__STOP_TIMEOUT` | `15` | seconds a fragment may take to stop |
| `HOST__RUN_LOSS_GRACE` | `30` | seconds without the blueprint root before a run stops |

By default a Host listens only on this machine. Open it with zenoh endpoints or interfaces:

```sh skip
dimos host config listen=iface/wlan0                     # wlan0's addresses, port 7447
dimos host config listen=tcp/127.0.0.1:7447,iface/eth0   # loopback first, then eth0
```

The Host's own fragments dial the first one.

On a Go2, `go2-nav` and `go2-dds` dial the Host router on `tcp/127.0.0.1:7447`. Keep it first and
add the Jetson's robot-side interface, which the Go2 forwards 7447 to:

```sh skip
dimos host install --start --config listen=tcp/127.0.0.1:7447,iface/enP8p1s0
```
