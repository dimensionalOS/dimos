# syntax=docker/dockerfile:1.7
#
# apps/server — Bun + Hono + Better Auth + oRPC.
# Build from the repo root (context must include the full workspace so bun can
# resolve workspace:* deps):
#   docker build -f apps/server/Dockerfile -t robomoo-server .
# Railway injects $PORT at runtime; Bun.serve picks it up via env.PORT.

# ---- builder ----------------------------------------------------------------
FROM oven/bun:1.3.14 AS builder
WORKDIR /app

# Copy every workspace manifest + the lockfile first so `bun install` is
# cacheable across source edits.
COPY package.json bun.lock turbo.json tsconfig.base.json ./
COPY apps/server/package.json ./apps/server/
COPY apps/web/package.json ./apps/web/
COPY apps/gateway/package.json ./apps/gateway/
COPY packages/api/package.json ./packages/api/
COPY packages/db/package.json ./packages/db/
COPY packages/shared/package.json ./packages/shared/

RUN bun install --no-summary

# Source the server actually needs at runtime. packages/db carries the
# drizzle/ migrations folder applied on boot.
COPY apps/server ./apps/server
COPY packages/api ./packages/api
COPY packages/db ./packages/db
COPY packages/shared ./packages/shared

# ---- runtime ----------------------------------------------------------------
FROM oven/bun:1.3.14-slim AS runtime
WORKDIR /app

ENV NODE_ENV=production

# Bun runs TypeScript directly — no build step. Copy the whole tree so
# workspace:* resolution keeps working via the root node_modules symlinks.
COPY --from=builder /app /app

WORKDIR /app/apps/server

EXPOSE 4471

HEALTHCHECK --interval=30s --timeout=5s --start-period=20s --retries=3 \
  CMD bun -e "fetch('http://localhost:'+(process.env.PORT||4471)+'/health').then(r=>process.exit(r.ok?0:1)).catch(()=>process.exit(1))"

CMD ["bun", "run", "src/server.ts"]
