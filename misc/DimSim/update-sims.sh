#!/usr/bin/env bash
# Regenerate public/sims/manifest.json from the .json files in public/sims/.
# Run after adding or removing scene files.
set -e
DIR="$(cd "$(dirname "$0")" && pwd)/public/sims"
cd "$DIR"
echo -n "[" > manifest.json
first=true
for f in *.json; do
  [ "$f" = "manifest.json" ] && continue
  name="${f%.json}"
  if [ "$first" = true ]; then first=false; else echo -n "," >> manifest.json; fi
  echo -n "\"$name\"" >> manifest.json
done
echo "]" >> manifest.json
echo "Updated manifest: $(cat manifest.json)"
