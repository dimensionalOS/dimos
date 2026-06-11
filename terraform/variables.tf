variable "aws_region" {
  default = "us-east-2"
}

variable "instance_type" {
  default = "t3.small"
}

variable "key_name" {
  default = "daneel-local"
}

variable "vpc_id" {
  description = "VPC ID. Use default VPC or specify existing."
  default     = ""
}

variable "subnet_id" {
  description = "Subnet ID. Leave empty to use first available in VPC."
  default     = ""
}

variable "app_port" {
  default = 8450
}

variable "cf_teleop_app_id" {
  description = "Cloudflare Realtime SFU App ID"
  sensitive   = true
}

variable "cf_teleop_app_secret" {
  description = "Cloudflare Realtime SFU App Secret"
  sensitive   = true
}
