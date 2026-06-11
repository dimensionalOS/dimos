# ─── SQLite backup: Litestream → S3 ──────────────────────────────────
#
# The app DB (robot API keys, session history) lives on the instance root
# volume; a `terraform taint` re-pave or dead instance would lose it.
# Litestream continuously replicates the SQLite WAL to this bucket, and
# user_data restores the latest generation on first boot.

resource "aws_s3_bucket" "db_backup" {
  bucket = "dimos-teleop-db-backup"

  tags = {
    Name    = "dimos-teleop-db-backup"
    Service = "teleop"
  }
}

resource "aws_s3_bucket_public_access_block" "db_backup" {
  bucket = aws_s3_bucket.db_backup.id

  block_public_acls       = true
  block_public_policy     = true
  ignore_public_acls      = true
  restrict_public_buckets = true
}

resource "aws_s3_bucket_versioning" "db_backup" {
  bucket = aws_s3_bucket.db_backup.id

  versioning_configuration {
    status = "Enabled"
  }
}

resource "aws_iam_role" "teleop_instance" {
  name = "dimos-teleop-instance"

  assume_role_policy = jsonencode({
    Version = "2012-10-17"
    Statement = [{
      Effect    = "Allow"
      Principal = { Service = "ec2.amazonaws.com" }
      Action    = "sts:AssumeRole"
    }]
  })

  tags = { Service = "teleop" }
}

resource "aws_iam_role_policy" "teleop_db_backup" {
  name = "litestream-db-backup"
  role = aws_iam_role.teleop_instance.id

  policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Effect   = "Allow"
        Action   = ["s3:ListBucket", "s3:GetBucketLocation"]
        Resource = aws_s3_bucket.db_backup.arn
      },
      {
        Effect   = "Allow"
        Action   = ["s3:GetObject", "s3:PutObject", "s3:DeleteObject"]
        Resource = "${aws_s3_bucket.db_backup.arn}/*"
      },
    ]
  })
}

resource "aws_iam_instance_profile" "teleop" {
  name = "dimos-teleop-instance"
  role = aws_iam_role.teleop_instance.name
}
