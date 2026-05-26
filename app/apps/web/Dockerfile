# syntax=docker/dockerfile:1.7
#
# apps/web — Next.js (standalone output) on Bun.
# Build from the repo root (workspace deps need the full context):
#   docker build -f apps/web/Dockerfile -t robomoo-web .
# Reached internally by the gateway at web.railway.internal:PORT.

# ---- builder ----------------------------------------------------------------
FROM oven/bun:1.3.14 AS builder
WORKDIR /app

COPY package.json bun.lock turbo.json tsconfig.base.json ./
COPY apps/server/package.json ./apps/server/
COPY apps/web/package.json ./apps/web/
COPY apps/gateway/package.json ./apps/gateway/
COPY packages/api/package.json ./packages/api/
COPY packages/db/package.json ./packages/db/
COPY packages/shared/package.json ./packages/shared/

RUN bun install --no-summary

# Sources to build web: apps/web plus the workspace packages it imports
# (api + shared). db is a transitive TYPE dep of the api router type.
COPY apps/web ./apps/web
COPY packages/api ./packages/api
COPY packages/db ./packages/db
COPY packages/shared ./packages/shared

# Ensure the public dir exists (Next won't create it; the runtime COPY needs it).
RUN mkdir -p apps/web/public

# next.config.ts sets output: "standalone" → self-contained server at
# apps/web/.next/standalone.
RUN cd apps/web && bun run build

# ---- runtime ----------------------------------------------------------------
FROM oven/bun:1.3.14-slim AS runtime
WORKDIR /app

ENV NODE_ENV=production
ENV PORT=4472
ENV HOSTNAME=0.0.0.0

COPY --from=builder /app/apps/web/.next/standalone ./
COPY --from=builder /app/apps/web/.next/static ./apps/web/.next/static
COPY --from=builder /app/apps/web/public ./apps/web/public

EXPOSE 4472

HEALTHCHECK --interval=30s --timeout=5s --start-period=20s --retries=3 \
  CMD bun -e "fetch('http://localhost:'+(process.env.PORT||4472)+'/').then(r=>process.exit(r.ok?0:1)).catch(()=>process.exit(1))"

CMD ["bun", "apps/web/server.js"]
