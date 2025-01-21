provider "aws" {
  region = var.region
}

# VPC and Subnet data sources (assuming default VPC)
data "aws_vpc" "default" {
  default = true
}

data "aws_subnets" "default" {
  filter {
    name   = "vpc-id"
    values = [data.aws_vpc.default.id]
  }
}

# IAM role for EC2 CloudWatch access
resource "aws_iam_role" "ec2_cloudwatch" {
  name = "${var.name_prefix}-cloudwatch"

  assume_role_policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Action = "sts:AssumeRole"
        Effect = "Allow"
        Principal = {
          Service = "ec2.amazonaws.com"
        }
      }
    ]
  })
}

# CloudWatch policy
resource "aws_iam_role_policy" "cloudwatch" {
  name = "${var.name_prefix}-cloudwatch-policy"
  role = aws_iam_role.ec2_cloudwatch.id

  policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Effect = "Allow"
        Action = [
          "logs:CreateLogGroup",
          "logs:CreateLogStream",
          "logs:PutLogEvents",
          "logs:DescribeLogStreams",
          "logs:DescribeLogGroups",
          "logs:PutRetentionPolicy"
        ]
        Resource = ["arn:aws:logs:*:*:*"]
      },
      {
        Effect = "Allow"
        Action = [
          "ec2:DescribeTags"
        ]
        Resource = ["*"]
      }
    ]
  })
}

# ECR policy
resource "aws_iam_role_policy" "ecr" {
  name = "${var.name_prefix}-ecr-policy"
  role = aws_iam_role.ec2_cloudwatch.id

  policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Effect = "Allow"
        Action = [
          "ecr:GetAuthorizationToken",
          "ecr:BatchCheckLayerAvailability",
          "ecr:GetDownloadUrlForLayer",
          "ecr:BatchGetImage"
        ]
        Resource = ["*"]
      },
      {
        Effect = "Allow"
        Action = [
          "ecr:GetAuthorizationToken"
        ]
        Resource = ["*"]
      },
      {
        Effect = "Allow"
        Action = [
          "ecr:BatchGetImage",
          "ecr:GetDownloadUrlForLayer"
        ]
        Resource = ["arn:aws:ecr:us-west-1:654654535911:repository/*"]
      }
    ]
  })
}

# Instance profile
resource "aws_iam_instance_profile" "ec2_profile" {
  name = "${var.name_prefix}-profile"
  role = aws_iam_role.ec2_cloudwatch.name
}

# Security Group
resource "aws_security_group" "dimos_simulator" {
  name        = "${var.name_prefix}-sg"
  description = "Security group for Dimos Simulator"
  vpc_id      = data.aws_vpc.default.id

  # SSH access
  ingress {
    from_port   = 22
    to_port     = 22
    protocol    = "tcp"
    cidr_blocks = ["0.0.0.0/0"]
  }

  # RTSP port
  ingress {
    from_port   = 8554
    to_port     = 8554
    protocol    = "tcp"
    cidr_blocks = ["0.0.0.0/0"]
  }

  # Allow all outbound traffic
  egress {
    from_port   = 0
    to_port     = 0
    protocol    = "-1"
    cidr_blocks = ["0.0.0.0/0"]
  }

  tags = {
    Name = "${var.name_prefix}-sg"
  }
}

# Create the log groups explicitly
resource "aws_cloudwatch_log_group" "system_logs" {
  name              = "/${var.name_prefix}/system"
  retention_in_days = 7
}

resource "aws_cloudwatch_log_group" "docker_logs" {
  name              = "/${var.name_prefix}/docker"
  retention_in_days = 7
}

# EC2 Instance
resource "aws_instance" "dimos_simulator" {
  ami                  = var.ami_id
  instance_type        = var.instance_type
  key_name             = var.ssh_key_name
  iam_instance_profile = aws_iam_instance_profile.ec2_profile.name

  subnet_id                   = data.aws_subnets.default.ids[0]
  vpc_security_group_ids      = [aws_security_group.dimos_simulator.id]
  associate_public_ip_address = true

  root_block_device {
    volume_size = var.volume_size
    volume_type = "gp3"
  }

  user_data = <<-EOF
              #!/bin/bash
              exec > >(tee /var/log/user-data.log|logger -t user-data -s 2>/dev/console) 2>&1
              echo "[$(date)] Starting user_data script..."

              # Install CloudWatch agent
              echo "[$(date)] Installing CloudWatch agent..."
              wget https://s3.amazonaws.com/amazoncloudwatch-agent/ubuntu/amd64/latest/amazon-cloudwatch-agent.deb
              dpkg -i amazon-cloudwatch-agent.deb

              # Configure CloudWatch agent
              echo "[$(date)] Configuring CloudWatch agent..."
              cat > /opt/aws/amazon-cloudwatch-agent/bin/config.json <<'EOL'
              {
                "agent": {
                  "run_as_user": "root"
                },
                "logs": {
                  "logs_collected": {
                    "files": {
                      "collect_list": [
                        {
                          "file_path": "/var/log/syslog",
                          "log_group_name": "/${var.name_prefix}/system",
                          "log_stream_name": "{instance_id}/syslog",
                          "timestamp_format": "%b %d %H:%M:%S"
                        },
                        {
                          "file_path": "/home/ubuntu/dimos/docker/simulation/docker-compose.log",
                          "log_group_name": "/${var.name_prefix}/docker",
                          "log_stream_name": "{instance_id}/docker-compose",
                          "timestamp_format": "%Y-%m-%d %H:%M:%S"
                        }
                      ]
                    }
                  }
                }
              }
              EOL

              # Start CloudWatch agent with debug logging
              echo "[$(date)] Starting CloudWatch agent..."
              /opt/aws/amazon-cloudwatch-agent/bin/amazon-cloudwatch-agent-ctl -a fetch-config -m ec2 -s -c file:/opt/aws/amazon-cloudwatch-agent/bin/config.json
              systemctl enable amazon-cloudwatch-agent
              systemctl start amazon-cloudwatch-agent

              # Verify CloudWatch agent status
              echo "[$(date)] Verifying CloudWatch agent status..."
              systemctl status amazon-cloudwatch-agent > /home/ubuntu/cloudwatch-status.log

              # Ensure directory ownership
              echo "[$(date)] Setting directory ownership..."
              chown -R ubuntu:ubuntu /home/ubuntu/dimos

              # Update repo and start services as ubuntu user
              echo "[$(date)] Starting services as ubuntu user..."
              sudo -u ubuntu bash <<'ENDSUDO'
              cd /home/ubuntu/dimos
              git config --global --add safe.directory /home/ubuntu/dimos
              # Clean up any root-owned files
              rm -f .git/FETCH_HEAD
              git pull origin main
              # Create empty .env file for docker compose
              touch .env
              cd docker/simulation
              # Login to ECR
              aws ecr get-login-password --region us-west-1 | docker login --username AWS --password-stdin 654654535911.dkr.ecr.us-west-1.amazonaws.com
              # Pull latest images explicitly
              docker compose pull
              docker compose up -d 2>&1 | tee docker-compose.log
              ENDSUDO

              # Add log rotation
              echo "[$(date)] Setting up log rotation..."
              cat > /etc/logrotate.d/dimos <<'EOL'
              /home/ubuntu/dimos/docker/simulation/docker-compose.log {
                  daily
                  rotate 7
                  compress
                  missingok
                  notifempty
                  create 666 ubuntu ubuntu
              }
              EOL

              echo "[$(date)] User data script completed."
              EOF

  tags = {
    Name = "${var.name_prefix}"
  }
}

# Output the public IP
output "public_ip" {
  value = aws_instance.dimos_simulator.public_ip
} 