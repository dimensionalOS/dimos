# ─── Cognito user pool (operator accounts) ───────────────────────────
#
# Operators authenticate against this pool from the SPA (USER_PASSWORD_AUTH
# + REFRESH_TOKEN_AUTH, no client secret). The broker verifies the resulting
# RS256 ID tokens against the pool JWKS — it never talks to Cognito itself.
# Robots are unaffected (X-Robot-API-Key, dtk_* keys in the app DB).

resource "aws_cognito_user_pool" "teleop" {
  name = "dimos-teleop"

  # Sign in with email; Cognito generates the internal username (UUID).
  username_attributes      = ["email"]
  auto_verified_attributes = ["email"]

  # Open self-signup.
  admin_create_user_config {
    allow_admin_create_user_only = false
  }

  password_policy {
    minimum_length    = 8
    require_lowercase = false
    require_numbers   = false
    require_symbols   = false
    require_uppercase = false
  }

  account_recovery_setting {
    recovery_mechanism {
      name     = "verified_email"
      priority = 1
    }
  }

  # Cognito-default sender: fine pre-production (50 emails/day cap).
  # Move to SES (DEVELOPER) before real launch.
  email_configuration {
    email_sending_account = "COGNITO_DEFAULT"
  }

  schema {
    name                = "email"
    attribute_data_type = "String"
    required            = true
    mutable             = true

    string_attribute_constraints {
      min_length = 3
      max_length = 256
    }
  }

  deletion_protection = "ACTIVE"

  tags = {
    Name    = "dimos-teleop"
    Service = "teleop"
  }
}

resource "aws_cognito_user_pool_client" "spa" {
  name         = "teleop-web"
  user_pool_id = aws_cognito_user_pool.teleop.id

  # Public SPA client — no secret.
  generate_secret = false

  explicit_auth_flows = [
    "ALLOW_USER_PASSWORD_AUTH",
    "ALLOW_USER_SRP_AUTH",
    "ALLOW_REFRESH_TOKEN_AUTH",
  ]

  # Don't reveal whether an email is registered on failed login.
  prevent_user_existence_errors = "ENABLED"

  id_token_validity      = 24
  access_token_validity  = 24
  refresh_token_validity = 30

  token_validity_units {
    id_token      = "hours"
    access_token  = "hours"
    refresh_token = "days"
  }
}

# Membership in this group maps to role=admin in the broker.
resource "aws_cognito_user_group" "admin" {
  name         = "admin"
  user_pool_id = aws_cognito_user_pool.teleop.id
  description  = "Operators with admin role"
}

output "cognito_user_pool_id" {
  value = aws_cognito_user_pool.teleop.id
}

output "cognito_client_id" {
  value = aws_cognito_user_pool_client.spa.id
}

output "cognito_issuer" {
  value = "https://cognito-idp.${var.aws_region}.amazonaws.com/${aws_cognito_user_pool.teleop.id}"
}
