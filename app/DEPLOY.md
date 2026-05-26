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

## Build model — GitHub auto-deploy

All three services are connected to **`grmkris/robohack` @ `master`** and
**auto-deploy on every push**. Each builds from **its own Dockerfile** with the
**`app/` monorepo root as build context** (so `workspace:*` resolves). Two
service settings make this work:

- **Root Directory = `app`** → Railway uses `app/` as the build context.
- **Config-as-code path** → points at each `railway.toml`. ⚠️ **This path is
  absolute from the repo root and does NOT follow Root Directory**, so it must be
  `/app/apps/<svc>/railway.toml` (not `apps/<svc>/railway.toml`). Getting this
  wrong fails the build instantly with no Docker output. Inside the `railway.toml`,
  `dockerfilePath` *is* relative to Root Directory (`apps/<svc>/Dockerfile`).

| Service | Config file (repo-root absolute) | Dockerfile (root-dir relative) | Start | Healthcheck |
|---|---|---|---|---|
| server | `/app/apps/server/railway.toml` | `apps/server/Dockerfile` | `bun run src/server.ts` | `/health` |
| web | `/app/apps/web/railway.toml` | `apps/web/Dockerfile` | `bun apps/web/server.js` | `/` |
| gateway | `/app/apps/gateway/railway.toml` | `apps/gateway/Dockerfile` | (Caddy) | `/gateway-health` |

The server runs Drizzle migrations on boot (`packages/db/drizzle/`), so a fresh
DB is provisioned automatically.

> **Note:** the `robohack` repo previously had two private submodules
> (`perception-watch`, `gs-pot`) which blocked Railway from cloning. They were
> removed so CI/CD works; the upstream repos are unaffected.

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

## Deploying

**Normal path: just push to `master`.** Railway redeploys every connected
service automatically.

Manual redeploy of one service (pulls latest `master`, no code change needed):

```bash
railway redeploy --service server --from-source -y
```

Check status / logs:

```bash
railway deployment list --service server --json
railway logs --service server --lines 100
```

> The CLI is linked to the project (`railway link --project eb47f62c-…`).
> `railway up --service <svc>` still works for an ad-hoc upload deploy that
> bypasses GitHub.

## First-time provisioning (reproduce from scratch)

1. `create_project robomoo` (workspace `22a4b765-…`).
2. `railway link --project <id> --environment production && railway add --database postgres`.
3. `create_bucket robomoo-images` (region `ams`); `railway bucket credentials …`.
4. `create_service` × 3 (server, web, gateway). Connect each to the GitHub repo
   (`serviceConnect` → `grmkris/robohack`@`master`), set **Root Directory = `app`**
   and **config file = `/app/apps/<svc>/railway.toml`** (absolute from repo root).
5. Set variables (above); `DATABASE_URL` as a reference var.
6. `generate_domain` on gateway (port 4470) → set that URL as the server's `APP_URL`.
7. Push to `master` (or `railway redeploy --from-source`). Watch logs until healthy.

## Local dev

`bun install`, then `bun run db:start` (Postgres :54325 + MinIO :9000/:9001),
then `bun run dev` (server + web) and `bun run dev:gateway` (Caddy on :4470).
Local `.env` files live in `apps/server/.env` and `apps/web/.env` (gitignored;
see `.env.example`). MinIO is the local stand-in for the Railway bucket — same
Bun S3 code, path-style URLs.
