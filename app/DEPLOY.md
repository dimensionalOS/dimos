# Deploy — Railway (robomoo)

Three app services + managed Postgres + S3 bucket, one project. Public traffic
enters through the **Caddy gateway** (the only service with a domain); `web` and
`server` are internal-only on `*.railway.internal`. Object storage is **Railway
Storage** (S3-compatible) accessed with Bun's native `S3Client`.

```
Browser ──HTTPS──> gateway (Caddy, public)
            /rpc/* + /api/auth/* + /api/upload/* ──> server (Bun+Hono) :4471 ──> Postgres
            everything else ───────────────────────> web (Next.js std) :4472    └──> bucket (S3)
```

## Railway IDs (workspace: "Kris's Projects")

| Resource | Name | ID |
|---|---|---|
| Project | `robomoo` | `eb47f62c-4513-45e9-92ed-c3fa43b9fc4a` |
| Environment | `production` | `c7037515-52c9-4e8b-968e-a00b828a9cb0` |
| Service | `server` | `cb366a87-0312-44d8-9067-4f832b612218` |
| Service | `web` | `7ed3ed67-e355-4b17-81b5-1c512e272670` |
| Service | `gateway` | `396febf6-e05a-4b34-a85a-dfd80e6b912b` |
| Database | `Postgres` | `9d853b73-b59f-4fc7-a7d8-4adfdadeb6c8` |
| Bucket | `robomoo-images` (`robomoo-images-u9vvr8xti3`) | `ee4dd85f-e354-4164-a98b-c359674ebaa7` |

- Public URL: **https://gateway-production-94e2.up.railway.app**
- Dashboard: https://railway.com/project/eb47f62c-4513-45e9-92ed-c3fa43b9fc4a

## Build model

Each service builds from **its own Dockerfile** with the **monorepo root as build
context** (so `workspace:*` resolves). Railway picks the Dockerfile via each
service's **config-as-code path** (set with `update_service railway_config_file`,
or the MCP equivalent), pointing at `apps/<svc>/railway.toml`:

| Service | Config file | Dockerfile | Start | Healthcheck |
|---|---|---|---|---|
| server | `apps/server/railway.toml` | `apps/server/Dockerfile` | `bun run src/server.ts` | `/health` |
| web | `apps/web/railway.toml` | `apps/web/Dockerfile` | `bun apps/web/server.js` | `/` |
| gateway | `apps/gateway/railway.toml` | `apps/gateway/Dockerfile` | (Caddy) | `/` |

Root directory stays `/` for all three. The server runs Drizzle migrations on
boot (`packages/db/drizzle/`), so a fresh DB is provisioned automatically.

## Environment variables (per service)

**server** — owns all secrets:

| Var | Value |
|---|---|
| `DATABASE_URL` | `${{Postgres.DATABASE_URL}}` (reference) |
| `BETTER_AUTH_SECRET` | random 32 bytes (`openssl rand -base64 32`) |
| `APP_URL` | the gateway public URL (Better Auth baseURL/trustedOrigin) |
| `S3_ENDPOINT` | `https://t3.storageapi.dev` |
| `S3_REGION` | `auto` |
| `S3_ACCESS_KEY_ID` / `S3_SECRET_ACCESS_KEY` | from `railway bucket credentials --bucket robomoo-images --json` |
| `S3_BUCKET` | `robomoo-images-u9vvr8xti3` (the real bucket name, not the alias) |
| `PORT` | `4471` |

**web**: `RPC_INTERNAL_URL=http://server.railway.internal:4471`, `PORT=4472`.

**gateway**: `SERVER_URL=http://server.railway.internal:4471`,
`WEB_URL=http://web.railway.internal:4472`, `PORT=4470`.

> Bucket credentials are issued per bucket and are **not committed**. Re-fetch
> them any time with `railway bucket credentials --bucket robomoo-images --json`.

## Redeploy (from this directory)

The CLI is already linked to the project (`railway link --project eb47f62c-…`).
Deploy a single service (build context = repo root, picks the service's
config-as-code Dockerfile):

```bash
# via Railway CLI
railway up --service server --detach
railway up --service web --detach
railway up --service gateway --detach

# or via the Railway MCP `deploy` tool: path=<this dir>, service_id=<id above>
```

Check status / logs:

```bash
railway deployment list --json
railway logs --service server --lines 100
```

## First-time provisioning (reproduce from scratch)

1. `create_project robomoo` (workspace `22a4b765-…`).
2. `railway link --project <id> --environment production && railway add --database postgres`.
3. `create_bucket robomoo-images` (region `ams`); `railway bucket credentials …`.
4. `create_service` × 3 (server, web, gateway); set each `railway_config_file`.
5. Set variables (above); `DATABASE_URL` as a reference var.
6. `generate_domain` on gateway (port 4470) → set that URL as the server's `APP_URL`.
7. `deploy` each service (path = repo root). Watch logs until healthy.

## Local dev

`bun install`, then `bun run db:start` (Postgres :54325 + MinIO :9000/:9001),
then `bun run dev` (server + web) and `bun run dev:gateway` (Caddy on :4470).
Local `.env` files live in `apps/server/.env` and `apps/web/.env` (gitignored;
see `.env.example`). MinIO is the local stand-in for the Railway bucket — same
Bun S3 code, path-style URLs.
