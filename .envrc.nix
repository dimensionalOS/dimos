if ! has nix_direnv_version || ! nix_direnv_version 3.0.6; then
  source_url "https://raw.githubusercontent.com/nix-community/nix-direnv/3.0.6/direnvrc" "sha256-RYcUJaRMf8oF5LznDrlCXbkOQrywm0HDv1VjYGaJGdM="
fi
use flake .
for venv in venv .venv env; do
  if [[ -f "$venv/bin/activate" ]]; then
    source "$venv/bin/activate"
    break
  fi
done
dotenv_if_exists
