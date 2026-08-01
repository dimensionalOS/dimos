FROM node:22.19.0-bookworm-slim AS build

WORKDIR /build
COPY package.json package-lock.json ./
RUN npm ci
COPY tsconfig.json tsconfig.build.json ./
COPY src ./src
RUN npm run build

FROM node:22.19.0-bookworm-slim

WORKDIR /opt/pi-spatial-adapter
COPY package.json package-lock.json ./
RUN npm ci --omit=dev
COPY --from=build /build/dist ./dist

ENV NODE_ENV=production
ENTRYPOINT ["node", "/opt/pi-spatial-adapter/dist/main.js"]
