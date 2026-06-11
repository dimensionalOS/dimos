#!/bin/bash
# Deploy app code to an existing dimos-teleop EC2 instance.
# Usage: ./scripts/deploy.sh <ip-address>
#
# Run from the repo root. rsyncs app/ into /opt/dimos-teleop/app/ on the
# instance and restarts the systemd unit. Assumes user_data has already
# bootstrapped Python/Caddy/systemd and chowned /opt/dimos-teleop to ubuntu.

set -euo pipefail

IP="${1:?Usage: deploy.sh <ip-address>}"
KEY="${SSH_KEY:-daneel-local.pem}"
SSH_OPTS="-i $KEY -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR"

echo "Deploying to $IP..."

rsync -avz --delete \
  --exclude __pycache__ --exclude '*.pyc' --exclude .venv --exclude '*.db' --exclude '*.db-wal' --exclude '*.db-shm' --exclude '.*-litestream' --exclude .env \
  -e "ssh $SSH_OPTS" \
  app/ ubuntu@$IP:/opt/dimos-teleop/app/

# Refresh deps in case requirements.txt changed
ssh $SSH_OPTS ubuntu@$IP \
  '/opt/dimos-teleop/.venv/bin/pip install --quiet -r /opt/dimos-teleop/app/requirements.txt'

ssh $SSH_OPTS ubuntu@$IP 'sudo systemctl restart dimos-teleop'
sleep 2

echo "--- service health (from inside the box) ---"
ssh $SSH_OPTS ubuntu@$IP '
  sudo systemctl is-active dimos-teleop
  curl -sf http://127.0.0.1:8450/health && echo
'
