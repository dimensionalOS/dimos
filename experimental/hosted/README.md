# Host discovery with Docker Compose

The two Compose files start 10 independent `dimos host serve` replicas and use
the repository source to build `dimos-host-discovery:compose` locally.

## With a Zenoh router

```bash
docker compose -f experimental/hosted/compose.router.yaml up -d --build
docker compose -f experimental/hosted/compose.router.yaml run --rm discovery
```

## Without a router (multicast scouting)

```bash
docker compose -f experimental/hosted/compose.multicast.yaml up -d --build
docker compose -f experimental/hosted/compose.multicast.yaml run --rm discovery
```

Each discovery command should return 10 available hosts with distinct
`host_id` values. The multicast scenario requires Docker networking and the
host environment to allow multicast traffic.

Stop both scenarios with:

```bash
docker compose -f experimental/hosted/compose.router.yaml down
docker compose -f experimental/hosted/compose.multicast.yaml down
```
