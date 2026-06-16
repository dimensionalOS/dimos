# dimos-teleop

Private session microservice for hosted teleoperation. Handles auth, session lifecycle, and Cloudflare Realtime SFU orchestration.

## Architecture

```
Operator/Robot ──HTTPS──→ dimos-teleop (EC2) ──REST──→ Cloudflare Realtime SFU API
                          (auth + session mgmt)        (creates WebRTC sessions)

After session setup, real-time data flows direct:
Operator ←──WebRTC──→ Cloudflare Edge ←──WebRTC──→ Robot
```

The microservice is only in the path for session setup (create, join, leave); operator login happens directly against Cognito and the broker just verifies the resulting tokens. All video and command data flows directly through Cloudflare's WebRTC SFU.

## Quick Start (Local Dev)

```bash
cd app
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env  # Edit with your CF credentials
uvicorn main:app --reload --port 8450
```

API docs at `http://localhost:8450/docs`

## Deploy to EC2

### Prerequisites
- AWS CLI configured with EC2/VPC/EIP/Route53 permissions
- Terraform installed
- `daneel-local.pem` key pair in AWS (us-east-2)

### Deploy

```bash
cd terraform
cp terraform.tfvars.example terraform.tfvars  # Edit variables
terraform init
terraform apply
```

This creates:
- t3.small EC2 instance (Ubuntu 22.04)
- Elastic IP (static)
- Security group (HTTP 8450, HTTPS 443, SSH 22)
- Deploys the app via user_data

### DNS Setup (Route53)

After `terraform apply` outputs the Elastic IP:

1. Go to Route53 → dimensionalos.com hosted zone
2. Create A record: `teleop.dimensionalos.com` → `<elastic_ip>`

Or use the Terraform Route53 resource (see `terraform/route53.tf`).

## API

See `docs/api.md` for the full API spec.

## Environment Variables

| Variable | Description |
|----------|-------------|
| `CF_TELEOP_APP_ID` | Cloudflare Realtime SFU App ID |
| `CF_TELEOP_APP_SECRET` | Cloudflare Realtime SFU App Secret |
| `COGNITO_REGION` | Cognito region (default `us-east-2`) |
| `COGNITO_USER_POOL_ID` | Cognito user pool (terraform output `cognito_user_pool_id`) |
| `COGNITO_CLIENT_ID` | Cognito SPA app client (terraform output `cognito_client_id`) |
| `DATABASE_URL` | SQLite or Postgres connection string |
| `LIVEKIT_URL` | LiveKit server URL (`wss://…`) — alternative backend, optional |
| `LIVEKIT_API_KEY` | LiveKit API key (mints room JWTs server-side) |
| `LIVEKIT_API_SECRET` | LiveKit API secret |
