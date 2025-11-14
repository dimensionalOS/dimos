variable "region" {
  description = "AWS region"
  type        = string
  default     = "us-east-2"
}

variable "name_prefix" {
  description = "Prefix for resource names to avoid conflicts"
  type        = string
  default     = "dimos-sim-1"
}

variable "instance_type" {
  description = "EC2 instance type"
  type        = string
  default     = "g4dn.2xlarge"
}

variable "ami_id" {
  description = "AMI ID with Ubuntu 22.04 and NVIDIA driver 535"
  type        = string
  default     = "ami-00b5876d1753f91cb"
}

variable "volume_size" {
  description = "Root volume size in GB"
  type        = number
  default     = 50
}

variable "ecr_registry" {
  description = "ECR registry URL"
  type        = string
  default     = "654654535911.dkr.ecr.us-west-1.amazonaws.com"
}

variable "ssh_key_name" {
  description = "Name of the SSH key pair to use"
  type        = string
  default     = "ec2-daneel-ubuntu"
} 