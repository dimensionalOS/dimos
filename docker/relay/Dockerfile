# The hosted relay: web/ built into one image, run as the image's unprivileged
# deno user (uid 1993) with the certificate and auth file mounted at /etc/relay.
# Guide: docs/usage/relay_hosting.md; compose.yaml next to this file runs it.
# The Deno pin lives in dimos/utils/deno.py (CI greps it there); keep them equal.
FROM denoland/deno:2.6.10

# A server has no use for the daily version check against dl.deno.land.
ENV DENO_NO_UPDATE_CHECK=1
WORKDIR /app/web
# The build context is the repo root; Dockerfile.dockerignore next to this
# file narrows it to web/ (the root .dockerignore would drop package.json).
# WORKDIR created /app/web as root and --chown covers only the copied entries,
# so the directory itself needs the chown for deno to create node_modules.
COPY --chown=deno:deno web/ /app/web
RUN chown deno:deno /app/web
USER deno
RUN deno install --frozen && deno task -r build

EXPOSE 443/tcp 443/udp
# Docker lets the unprivileged user bind 443 (ip_unprivileged_port_start=0
# inside containers since 20.10). CMD holds only the file flags, so a compose
# `command:` can override those without repeating the rest.
ENTRYPOINT ["deno", "run", "--frozen", "--allow-net", "--allow-read=/app/web,/etc/relay", \
  "relay/main.ts", "--host", "0.0.0.0", "--port", "443", \
  "--cockpit-dir", "cockpit/dist", "--sdk-dir", "sdk/dist"]
CMD ["--cert", "/etc/relay/fullchain.pem", "--key", "/etc/relay/privkey.pem", \
  "--auth-file", "/etc/relay/auth.json"]
