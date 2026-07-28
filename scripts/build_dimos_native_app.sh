#!/bin/zsh

set -euo pipefail

SCRIPT_DIR="${0:A:h}"
PROJECT_DIR="${SCRIPT_DIR:h}"
SOURCE_DIR="${PROJECT_DIR}/apps/DimOS Native"
APP_DIR="${PROJECT_DIR}/apps/DimOS Native.app"
CONTENTS_DIR="${APP_DIR}/Contents"
MACOS_DIR="${CONTENTS_DIR}/MacOS"

rm -rf "${APP_DIR}"
mkdir -p "${MACOS_DIR}"

swiftc \
  -O \
  -framework AppKit \
  "${SOURCE_DIR}/main.swift" \
  -o "${MACOS_DIR}/DimOS Native"

cp "${SOURCE_DIR}/Info.plist" "${CONTENTS_DIR}/Info.plist"
chmod +x "${MACOS_DIR}/DimOS Native"

codesign --force --deep --sign - "${APP_DIR}"

echo "${APP_DIR}"
