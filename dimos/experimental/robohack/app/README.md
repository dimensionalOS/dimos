# robomoo

A small, deployable full-stack sample on the AIS/Sonara stack — a monorepo
inside the `robohack` repo (lives in `app/`).

**Stack:** Bun workspaces + Turborepo · Bun + Hono server · Next.js 16 +
shadcn + Tailwind v4 web · oRPC (end-to-end type-safe RPC) · Drizzle + Postgres
· Better Auth (email + password) · object storage via Bun's native S3 client.

**Topology (Railway):** a Caddy **gateway** is the only public service; it
path-routes `/rpc`, `/api/auth`, `/api/upload` to the **server** and everything
else to the **web** app, both internal-only. Postgres + an S3 bucket back the
server.

```
apps/
  server/   Bun + Hono + Better Auth + oRPC; Drizzle migrations on boot; S3 upload
  web/      Next.js 16 (standalone) + shadcn; oRPC client; auth + image posting
  gateway/  Caddy reverse proxy (public edge)
packages/
  shared/   zod schemas + id helper
  db/       Drizzle schema (messages + Better Auth) + migrator
  api/      oRPC router, context, procedures
```

## Local dev

```bash
bun install
bun run db:start          # Postgres :54325 + MinIO :9000/:9001 (docker)
bun run dev               # server :4471 + web :4472 (turbo)
bun run dev:gateway       # Caddy on :4470
# open http://localhost:4470
```

Copy `.env.example` → `apps/server/.env` and `apps/web/.env` for local config.

## Deploy

Pushing to `master` auto-deploys all three services on Railway. See
[DEPLOY.md](./DEPLOY.md) for IDs, the env-var matrix, and the config-as-code
gotchas.
