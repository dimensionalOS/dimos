#!/bin/bash
set -euo pipefail

# ─── System setup ────────────────────────────────────────────────────

export DEBIAN_FRONTEND=noninteractive
apt-get update -y
apt-get install -y software-properties-common ca-certificates curl gnupg \
  debian-keyring debian-archive-keyring apt-transport-https

# Caddy is not in Ubuntu's default apt repos — add Cloudsmith's stable repo.
curl -1sLf 'https://dl.cloudsmith.io/public/caddy/stable/gpg.key' \
  | gpg --dearmor -o /usr/share/keyrings/caddy-stable-archive-keyring.gpg
curl -1sLf 'https://dl.cloudsmith.io/public/caddy/stable/debian.deb.txt' \
  > /etc/apt/sources.list.d/caddy-stable.list

# Jammy ships python3.10 by default; only a 3.11 RC is in backports.
# Use deadsnakes for stable python3.11.
add-apt-repository -y ppa:deadsnakes/ppa

apt-get update -y
apt-get install -y python3.11 python3.11-venv python3-pip git caddy

# ─── App setup ───────────────────────────────────────────────────────

APP_DIR=/opt/dimos-teleop
mkdir -p $APP_DIR
cd $APP_DIR

# Clone the repo (or copy from S3 — adjust as needed)
# For now, write the app inline from user_data
cat > .env << 'ENVEOF'
CF_TELEOP_APP_ID=${cf_teleop_app_id}
CF_TELEOP_APP_SECRET=${cf_teleop_app_secret}
CF_TURN_KEY_ID=${cf_turn_key_id}
CF_TURN_API_TOKEN=${cf_turn_api_token}
COGNITO_REGION=${cognito_region}
COGNITO_USER_POOL_ID=${cognito_user_pool_id}
COGNITO_CLIENT_ID=${cognito_client_id}
DATABASE_URL=sqlite+aiosqlite:///./teleop.db
HOST=127.0.0.1
PORT=${app_port}
ENVEOF

chmod 600 .env

# ─── Litestream: restore DB from S3, then replicate continuously ─────

curl -sL https://github.com/benbjohnson/litestream/releases/download/v0.3.13/litestream-v0.3.13-linux-amd64.deb \
  -o /tmp/litestream.deb
dpkg -i /tmp/litestream.deb

cat > /etc/litestream.yml << 'LSEOF'
dbs:
  - path: /opt/dimos-teleop/app/teleop.db
    replicas:
      - type: s3
        bucket: dimos-teleop-db-backup
        path: teleop.db
        region: us-east-2
LSEOF

# On a fresh instance, pull the latest replicated DB (no-op if none exists).
# Runs before the app ever starts, so API keys/sessions survive a re-pave.
mkdir -p $APP_DIR/app
litestream restore -if-replica-exists -if-db-not-exists \
  -o $APP_DIR/app/teleop.db s3://dimos-teleop-db-backup/teleop.db

systemctl enable litestream
systemctl start litestream

# ─── Python venv ─────────────────────────────────────────────────────

python3.11 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install \
  fastapi==0.115.12 \
  'uvicorn[standard]==0.34.3' \
  httpx==0.28.1 \
  pydantic==2.11.3 \
  pydantic-settings==2.9.1 \
  'python-jose[cryptography]==3.4.0' \
  sqlalchemy==2.0.41 \
  aiosqlite==0.21.0 \
  python-multipart==0.0.20

# ─── Systemd service ────────────────────────────────────────────────

cat > /etc/systemd/system/dimos-teleop.service << 'SVCEOF'
[Unit]
Description=dimos-teleop session microservice
After=network.target

[Service]
Type=simple
WorkingDirectory=/opt/dimos-teleop/app
EnvironmentFile=/opt/dimos-teleop/.env
ExecStart=/opt/dimos-teleop/.venv/bin/uvicorn main:app --host 127.0.0.1 --port ${app_port}
Restart=always
RestartSec=5
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
SVCEOF

# ─── Caddy reverse proxy (HTTPS) ────────────────────────────────────

# Caddy auto-provisions TLS via Let's Encrypt.
# Until DNS is pointed, it serves on :80/:443 with self-signed.
cat > /etc/caddy/Caddyfile << 'CADDYEOF'
:80, :443 {
    reverse_proxy 127.0.0.1:${app_port}
}
CADDYEOF

# Make /opt/dimos-teleop writable to the ubuntu user so scripts/deploy.sh
# can rsync app/ in without sudo. The systemd unit still runs as root, so
# it can read everything regardless of ownership.
chown -R ubuntu:ubuntu /opt/dimos-teleop

# ─── Start services ─────────────────────────────────────────────────

systemctl daemon-reload
systemctl enable dimos-teleop
# dimos-teleop will fail-and-restart until app code is rsynced in via
# scripts/deploy.sh; that's fine — systemd's Restart=always handles it.
systemctl start dimos-teleop || true
systemctl restart caddy

echo "dimos-teleop deployed successfully"
