# Deployment Guide

## Prerequisites

- AWS CLI configured with full access (EC2, VPC, EIP, Route53)
- Terraform installed
- `daneel-local.pem` SSH key (must match AWS key pair `daneel-local` in us-east-2)
- GitHub repo access to `dimensionalOS/dimos` (the teleop code lives under `hosted/`)

## Secrets

Set these as GitHub repository secrets (Settings → Secrets → Actions) for CI, or pass directly to Terraform:

| Secret | Description |
|--------|-------------|
| `CF_TELEOP_APP_ID` | Cloudflare Realtime SFU App ID |
| `CF_TELEOP_APP_SECRET` | Cloudflare Realtime SFU App Secret |
| `CF_TURN_KEY_ID` | Cloudflare TURN key ID ([Realtime → TURN](https://dash.cloudflare.com/?to=/:account/realtime/turn) → create key). Optional: empty = STUN-only, which fails for operators/robots on UDP-blocked networks |
| `CF_TURN_API_TOKEN` | Cloudflare TURN key API token (shown once at key creation) |
| — | Operator auth uses the Cognito pool created by terraform (no auth secret to manage) |

Find CF credentials in the Cloudflare dashboard: [Realtime SFU](https://dash.cloudflare.com/?to=/:account/realtime/sfu) → `hosted-teleop-dev-0` app.

## Step 1: Get the code + provision EC2

You deploy from a local checkout — the EC2 box gets code by rsync (Step 3), it
never clones the repo, and it needs nothing else from the monorepo. If you
already have `dimos` checked out, skip the clone. Otherwise clone it, either in
full or as a minimal checkout of just `hosted/` (no LFS models, no other
subsystems):

```bash
# Full clone (if you work in dimos anyway):
git clone https://github.com/dimensionalOS/dimos.git

# — or — sparse checkout of only the teleop code:
git clone --filter=blob:none --no-checkout https://github.com/dimensionalOS/dimos.git
cd dimos && git sparse-checkout set hosted && git checkout main
```

Then provision. The box is created *here* — Terraform runs `user_data` to
bootstrap it, so a hand-launched EC2 from the console won't work:

```bash
cd dimos/hosted/infra/terraform          # already inside dimos: cd hosted/infra/terraform
cp terraform.tfvars.example terraform.tfvars
```

Edit `terraform.tfvars` with your actual values:
```hcl
aws_region           = "us-east-2"
instance_type        = "t3.small"
key_name             = "daneel-local"
ssh_ingress_cidrs    = ["203.0.113.7/32"]  # REQUIRED — your admin IP(s); never 0.0.0.0/0
cf_teleop_app_id     = "<from CF dashboard>"
cf_teleop_app_secret = "<from CF dashboard>"
```

Deploy:
```bash
terraform init
terraform apply
```

Outputs:
- `public_ip` — Elastic IP (static, survives reboots)
- `ssh_command` — Ready-to-use SSH command
- `api_url` — HTTP endpoint for health check

## Step 2: DNS — Route53

Create an A record pointing `teleop.dimensionalos.com` to the Elastic IP.

**Option A: Manual (Route53 console)**
1. Go to Route53 → `dimensionalos.com` hosted zone
2. Create record:
   - Name: `teleop`
   - Type: `A`
   - Value: `<elastic_ip from terraform output>`
   - TTL: `300`

**Option B: Terraform (automated)**
1. Find your Route53 hosted zone ID for `dimensionalos.com`
2. Uncomment the block in `infra/terraform/route53.tf`
3. Set `route53_zone_id` in your tfvars
4. `terraform apply`

## Step 3: Deploy App Code

`user_data.sh.tpl` already creates `/opt/dimos-teleop`, the venv with deps,
the `.env` (from your CF terraform vars + the Cognito pool IDs), the systemd unit, and Caddy.
The only missing piece on a fresh instance is the app source — the repo is
private, so we don't `git clone` on the box. Instead, rsync the local clone
in via the deploy script:

```bash
# From the dimos repo root, on your laptop (deploy.sh self-locates):
SSH_KEY=/path/to/daneel-local.pem hosted/infra/scripts/deploy.sh <elastic_ip>
```

`infra/scripts/deploy.sh`:
1. Rsyncs `broker/` to `/opt/dimos-teleop/app/` (excludes `.venv`, `.env`, `*.db`).
2. Rsyncs `web/` (the SPA Caddy serves) and the repo `Caddyfile`
   (reloads Caddy only if it changed).
3. Reinstalls `requirements.txt` (cheap if nothing changed).
4. Restarts the `dimos-teleop` systemd unit.
5. Health-checks `http://127.0.0.1:8450/health` from inside the box.

Until the first run of `infra/scripts/deploy.sh`, the systemd unit fails-and-
retries because `/opt/dimos-teleop/app/main.py` doesn't exist yet. That's
expected and harmless.

The same script works for subsequent code updates — just push to `main` (or your
working branch), pull locally, and re-run.

## Step 4: Configure HTTPS (Caddy)

The repo `infra/Caddyfile` is the single source of truth — it splits `/api`,
`/health` and docs to uvicorn and serves the static SPA from `/opt/dimos-teleop/web`
(a bare `reverse_proxy` would break the frontend). `infra/scripts/deploy.sh` ships
it and reloads Caddy automatically; to do it by hand:

```bash
scp -i daneel-local.pem infra/Caddyfile ubuntu@<elastic_ip>:/tmp/
ssh -i daneel-local.pem ubuntu@<elastic_ip> \
  'sudo cp /tmp/Caddyfile /etc/caddy/Caddyfile && sudo systemctl reload caddy'
```

Once DNS propagates (`dig teleop.dimensionalos.com` returns the EIP), Caddy
auto-provisions Let's Encrypt TLS. HTTPS is live within seconds.

## Step 5: Verify

```bash
# Health check
curl https://teleop.dimensionalos.com/health
# → {"status":"ok","service":"dimos-teleop"}

# API docs (Swagger UI)
open https://teleop.dimensionalos.com/docs

# Operator accounts: sign up through the web UI (open self-signup, emailed
# verification code), or create one non-interactively:
#   aws cognito-idp admin-create-user --user-pool-id <pool> --username you@example.com ...
# The broker itself has no register/login endpoints — it only verifies Cognito tokens.
```

## Architecture

```
This microservice handles ONLY:
  - Auth (Cognito token verification, robot API keys)
  - Session lifecycle (create, join, leave, list, heartbeat)
  - SDP exchange with Cloudflare Realtime SFU API

Real-time data (video, pose commands) flows DIRECTLY:
  Operator ←→ Cloudflare Edge (WebRTC) ←→ Robot
  This EC2 is NOT in the real-time path.
```

## Updating

Push to `main`, pull locally, then re-run the deploy from the dimos repo root:
```bash
hosted/infra/scripts/deploy.sh <elastic_ip>
```

The box has no clone of the repo, so always deploy from your laptop — there is
nothing to `git pull` on the instance.
