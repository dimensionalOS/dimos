# Host discovery with Docker Compose

The two Compose files start 10 independent `dimos host serve` replicas and use
the repository source to build `dimos-host-discovery:compose` locally.

## With a Zenoh router

This scenario has two isolated leaf networks. Five Hosts connect only to
Router A on `network-a`, and five connect only to Router B on `network-b`.
The routers communicate over a separate `router-transit` network. Discovery,
remote execution, and the sandbox are attached only to `network-a`, so seeing
all 10 Hosts proves that Router A and Router B are routing across the network
boundary.

```mermaid
flowchart LR
    subgraph network_a["network-a · internal leaf network"]
        sandbox["sandbox<br/>discovery / controller"]
        hosts_a["Host A × 5<br/>tag: network-a"]
    end

    subgraph router_a["Router A"]
        router_a_leaf["network-a interface"]
        router_a_transit["transit interface"]
        router_a_leaf --- router_a_transit
    end

    subgraph transit["router-transit · routers only"]
        router_link["Zenoh router link<br/>TCP :7447"]
    end

    subgraph router_b["Router B"]
        router_b_transit["transit interface"]
        router_b_leaf["network-b interface"]
        router_b_transit --- router_b_leaf
    end

    subgraph network_b["network-b · internal leaf network"]
        hosts_b["Host B × 5<br/>tag: network-b"]
    end

    sandbox -->|"client → router-a:7447"| router_a_leaf
    hosts_a -->|"client → router-a:7447"| router_a_leaf
    router_a_transit -->|"-e tcp/router-b:7447"| router_link
    router_link --> router_b_transit
    hosts_b -->|"client → router-b:7447"| router_b_leaf
```

```bash
docker compose -f experimental/hosted/compose.router.yaml up -d --build
docker compose -f experimental/hosted/compose.router.yaml run --rm discovery
```

## Without a router (multicast scouting)

This scenario puts the sandbox and all 10 Hosts on one shared Compose network.
There is no Router or configured connect endpoint. Every process runs in peer
mode and uses multicast on `eth0` to discover the others; Zenoh traffic and
gossip then flow through the discovered peer fabric.

```mermaid
flowchart TB
    subgraph lan["Compose default network · shared multicast-capable LAN"]
        sandbox_m["sandbox<br/>discovery / controller<br/>mode: peer"]
        multicast["multicast scouting<br/>interface: eth0"]
        hosts_m["Host peers × 10<br/>mode: peer"]

        sandbox_m <-.->|"scout / hello"| multicast
        hosts_m <-.->|"scout / hello"| multicast
        sandbox_m <-->|"Zenoh peer fabric<br/>data + gossip"| hosts_m
    end
```

```bash
docker compose -f experimental/hosted/compose.multicast.yaml up -d --build
docker compose -f experimental/hosted/compose.multicast.yaml run --rm discovery
```

Each discovery command should return 10 available hosts with distinct
`host_id` values. The multicast scenario requires Docker networking and the
host environment to allow multicast traffic.

## Real remote execution

The remote-execution probe runs in a separate controller container. It selects
one available Host, sends a serialized Blueprint fragment, calls the deployed
module over Zenoh, and then stops the deployment. Run it against either
scenario:

```bash
docker compose -f experimental/hosted/compose.router.yaml run --rm remote-execution
docker compose -f experimental/hosted/compose.multicast.yaml run --rm remote-execution
```

A successful result shows different controller and remote hostnames, a remote
PID, `running` lifecycle states, and a final Host state of `available`.

This MVP transfers a Blueprint description with Python pickle. It assumes a
trusted Zenoh network and the same application code already installed on the
Host; it is not source-code distribution or a sandbox for untrusted code.

## Interactive sandbox

Each scenario starts a persistent sandbox using the same image, network, and
Zenoh configuration. Open a shell with:

```bash
docker compose -f experimental/hosted/compose.router.yaml exec sandbox bash
docker compose -f experimental/hosted/compose.multicast.yaml exec sandbox bash
```

For example, inside the shell:

```bash
dimos host list --timeout 5
python -m experimental.hosted.run_remote_execution
```

The sandbox is an interactive test container, not a security boundary for
untrusted code.

Stop both scenarios with:

```bash
docker compose -f experimental/hosted/compose.router.yaml down
docker compose -f experimental/hosted/compose.multicast.yaml down
```
