terraform {
  required_providers {
    aws = {
      source  = "hashicorp/aws"
      version = "~> 5.0"
    }
  }
}

provider "aws" {
  region = var.aws_region
}

# ─── Data sources ────────────────────────────────────────────────────

data "aws_vpc" "selected" {
  id = var.vpc_id != "" ? var.vpc_id : null
  default = var.vpc_id == "" ? true : null
}

data "aws_subnets" "available" {
  filter {
    name   = "vpc-id"
    values = [data.aws_vpc.selected.id]
  }
  filter {
    name   = "map-public-ip-on-launch"
    values = ["true"]
  }
}

data "aws_ami" "ubuntu" {
  most_recent = true
  owners      = ["099720109477"] # Canonical

  filter {
    name   = "name"
    values = ["ubuntu/images/hvm-ssd/ubuntu-jammy-22.04-amd64-server-*"]
  }

  filter {
    name   = "virtualization-type"
    values = ["hvm"]
  }
}

# ─── Security Group ──────────────────────────────────────────────────

resource "aws_security_group" "teleop" {
  name_prefix = "dimos-teleop-"
  description = "dimos-teleop microservice"
  vpc_id      = data.aws_vpc.selected.id

  # SSH
  ingress {
    from_port   = 22
    to_port     = 22
    protocol    = "tcp"
    cidr_blocks = ["0.0.0.0/0"]
  }

  # App (HTTP)
  ingress {
    from_port   = var.app_port
    to_port     = var.app_port
    protocol    = "tcp"
    cidr_blocks = ["0.0.0.0/0"]
  }

  # HTTPS (for future TLS termination)
  ingress {
    from_port   = 443
    to_port     = 443
    protocol    = "tcp"
    cidr_blocks = ["0.0.0.0/0"]
  }

  # HTTP (for health checks / redirects)
  ingress {
    from_port   = 80
    to_port     = 80
    protocol    = "tcp"
    cidr_blocks = ["0.0.0.0/0"]
  }

  egress {
    from_port   = 0
    to_port     = 0
    protocol    = "-1"
    cidr_blocks = ["0.0.0.0/0"]
  }

  tags = {
    Name = "dimos-teleop"
  }
}

# ─── EC2 Instance ────────────────────────────────────────────────────

resource "aws_instance" "teleop" {
  ami                    = data.aws_ami.ubuntu.id
  instance_type          = var.instance_type
  key_name               = var.key_name
  subnet_id              = var.subnet_id != "" ? var.subnet_id : data.aws_subnets.available.ids[0]
  vpc_security_group_ids = [aws_security_group.teleop.id]
  iam_instance_profile   = aws_iam_instance_profile.teleop.name

  root_block_device {
    volume_size = 20
    volume_type = "gp3"
  }

  user_data = templatefile("${path.module}/user_data.sh.tpl", {
    app_port             = var.app_port
    cf_teleop_app_id     = var.cf_teleop_app_id
    cf_teleop_app_secret = var.cf_teleop_app_secret
    cf_turn_key_id       = var.cf_turn_key_id
    cf_turn_api_token    = var.cf_turn_api_token
    cognito_region       = var.aws_region
    cognito_user_pool_id = aws_cognito_user_pool.teleop.id
    cognito_client_id    = aws_cognito_user_pool_client.spa.id
  })

  tags = {
    Name    = "dimos-teleop"
    Service = "teleop"
  }

  lifecycle {
    ignore_changes = [ami, user_data]
  }
}

# ─── Elastic IP ──────────────────────────────────────────────────────

resource "aws_eip" "teleop" {
  instance = aws_instance.teleop.id
  domain   = "vpc"

  tags = {
    Name = "dimos-teleop"
  }
}

# ─── Outputs ─────────────────────────────────────────────────────────

output "public_ip" {
  description = "Static Elastic IP — point your DNS A record here"
  value       = aws_eip.teleop.public_ip
}

output "ssh_command" {
  value = "ssh -i daneel-local.pem ubuntu@${aws_eip.teleop.public_ip}"
}

output "api_url" {
  value = "http://${aws_eip.teleop.public_ip}:${var.app_port}"
}

output "dns_instructions" {
  value = <<-EOT
    To connect to dimensionalos.com:
    
    1. Route53 → dimensionalos.com hosted zone
    2. Create A record:
       Name:  teleop
       Type:  A
       Value: ${aws_eip.teleop.public_ip}
       TTL:   300
    
    Then access at: https://teleop.dimensionalos.com
    (Add HTTPS via Caddy/Certbot on the instance)
  EOT
}
