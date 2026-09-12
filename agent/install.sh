#!/bin/sh
# Installs the harness only. DimOS installation belongs to the agent's skill.
set -eu

main() {
  package='@spomichter/dimcode'
  version=${DIMCODE_VERSION:-next}
  data=${DIMCODE_INSTALL_DIR:-${XDG_DATA_HOME:-$HOME/.local/share}/dimcode}
  bin=${DIMCODE_BIN_DIR:-$HOME/.local/bin}
  case "$data:$bin" in /*:/*) ;; *) echo 'Install directories must be absolute.' >&2; exit 1 ;; esac
  case "$(uname -s)-$(uname -m)" in
    Linux-x86_64) platform=linux-x64 ;;
    Linux-aarch64|Linux-arm64) platform=linux-arm64 ;;
    Darwin-arm64) platform=darwin-arm64 ;;
    *) echo 'This prerelease supports Linux x64/arm64 and Apple Silicon.' >&2; exit 1 ;;
  esac
  case "$version" in ''|*[!a-zA-Z0-9._-]*) echo 'Invalid DIMCODE_VERSION.' >&2; exit 1 ;; esac
  command -v curl >/dev/null || { echo 'curl is required.' >&2; exit 1; }
  mkdir -p "$data" "$bin"
  staging=$(mktemp -d "$data/.install.XXXXXX")
  trap 'rm -rf "$staging"' EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM
  printf '\ndimcode · Install your Dimensional agent\n\n'

  node="$data/node/bin/node"
  if ! "$node" -e 'process.exit(+process.versions.node.split(".")[0] === 24 ? 0 : 1)' 2>/dev/null; then
    dist=https://nodejs.org/dist/latest-v24.x
    printf 'Installing a private Node 24 runtime…\n'
    curl -fsSL "$dist/SHASUMS256.txt" -o "$staging/checksums"
    archive=$(awk -v suffix="-$platform.tar.gz" '$2 ~ /^node-v[0-9.]+-/ && substr($2,length($2)-length(suffix)+1)==suffix {print $2; exit}' "$staging/checksums")
    [ -n "$archive" ] || { echo 'No compatible Node archive found.' >&2; exit 1; }
    curl -fsSL "$dist/$archive" -o "$staging/$archive"
    awk -v file="$archive" '$2==file {print}' "$staging/checksums" > "$staging/selected.sha256"
    if command -v sha256sum >/dev/null 2>&1; then
      (cd "$staging" && sha256sum -c selected.sha256)
    elif command -v shasum >/dev/null 2>&1; then
      (cd "$staging" && shasum -a 256 -c selected.sha256)
    else
      echo 'sha256sum or shasum is required to verify Node.' >&2; exit 1
    fi
    tar -xzf "$staging/$archive" -C "$staging"
    if [ -e "$data/node" ] || [ -L "$data/node" ]; then
      echo "Existing runtime at $data/node is unusable; choose another DIMCODE_INSTALL_DIR." >&2
      exit 1
    fi
    mv "$staging/${archive%.tar.gz}" "$data/node"
  fi
  PATH="$data/node/bin:$PATH"
  export PATH
  npm="$data/node/bin/npm"
  curl -fsSL "https://registry.npmjs.org/$package/$version" -o "$staging/metadata.json"
  resolved=$("$node" -p 'JSON.parse(require("node:fs").readFileSync(process.argv[1])).version' "$staging/metadata.json")
  case "$resolved" in ''|*[!a-zA-Z0-9._-]*) echo 'Invalid package version from registry.' >&2; exit 1 ;; esac
  release="$data/releases/$resolved"
  entry="$release/node_modules/$package/dist/main.js"
  if [ ! -f "$entry" ]; then
    printf 'Installing %s@%s…\n' "$package" "$resolved"
    tarball=$("$node" -p 'JSON.parse(require("node:fs").readFileSync(process.argv[1])).dist.tarball' "$staging/metadata.json")
    case "$tarball" in https://registry.npmjs.org/*) ;; *) echo 'Unexpected package registry.' >&2; exit 1 ;; esac
    curl -fsSL "$tarball" -o "$staging/dimcode.tgz"
    "$node" - "$staging/metadata.json" "$staging/dimcode.tgz" "$package" <<'NODE'
const fs = require('node:fs');
const crypto = require('node:crypto');
const metadata = JSON.parse(fs.readFileSync(process.argv[2]));
const actual = 'sha512-' + crypto.createHash('sha512').update(fs.readFileSync(process.argv[3])).digest('base64');
if (metadata.name !== process.argv[4] || actual !== metadata.dist.integrity) throw new Error('Package integrity check failed');
NODE
    "$npm" install --prefix "$staging/package" --ignore-scripts --min-release-age=0 --omit=dev --no-audit --no-fund "$staging/dimcode.tgz"
    "$node" "$staging/package/node_modules/$package/dist/main.js" --help >/dev/null
    mkdir -p "$data/releases"
    [ ! -e "$release" ] || { echo "Incomplete release at $release; choose another DIMCODE_INSTALL_DIR." >&2; exit 1; }
    mv "$staging/package" "$release"
  fi
  "$node" - "$bin/dimcode" "$node" "$entry" <<'NODE'
const fs = require('node:fs');
const [dest, node, entry] = process.argv.slice(2);
const quote = (value) => "'" + value.replaceAll("'", "'\\''") + "'";
const temporary = `${dest}.install-${process.pid}`;
fs.writeFileSync(temporary, `#!/bin/sh\nexec ${quote(node)} ${quote(entry)} "$@"\n`, { mode: 0o755, flag: 'wx' });
fs.renameSync(temporary, dest);
NODE
  printf '\nInstalled %s@%s\n' "$package" "$resolved"
  case ":${PATH}:" in
    *":$bin:"*) printf 'Run: dimcode setup\nThen: dimcode\n' ;;
    *) printf 'Run: %s/dimcode setup\nAdd %s to your shell PATH to use the short command.\n' "$bin" "$bin" ;;
  esac
  printf 'No daemon or robot was started. Setup asks about gateway startup.\n'
  if [ "${DIMCODE_SETUP:-1}" = 1 ] && ( : </dev/tty ) 2>/dev/null; then
    printf '\nOpen interactive setup now? [Y/n] ' >/dev/tty
    IFS= read -r answer </dev/tty || answer=n
    case "$answer" in n|N|no|NO) ;; *) "$bin/dimcode" setup </dev/tty ;; esac
  fi
}

main "$@"
