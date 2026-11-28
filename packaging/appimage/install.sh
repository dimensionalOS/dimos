#!/bin/sh
# Installs a DimOS AppImage by extracting it. This needs no FUSE. Run it as root:
#
#   sudo sh install.sh dimos-<version>-<arch>.AppImage
#
# The tree goes to /opt/dimos/<version>. /opt/dimos/current points at it and
# /usr/local/bin/dimos runs it. An update is the same command with a newer image.
# To go back, point the link at the old version: ln -sfn <version> /opt/dimos/current

set -eu

image="$(readlink -f "$1")"
root=/opt/dimos

mkdir -p "$root"
cd "$root"
[ -x "$image" ] || chmod +x "$image"
# The version comes first, from one small file.
rm -rf squashfs-root
"$image" --appimage-extract dimos.desktop > /dev/null
version="$(sed -n 's/^X-AppImage-Version=//p' squashfs-root/dimos.desktop)"
rm -rf squashfs-root
if [ -z "$version" ] || [ -e "$version" ]; then
  echo "install.sh: $root/${version:-<no version in the image>} is already there" >&2
  exit 1
fi
"$image" --appimage-extract > /dev/null
mv squashfs-root "$version"
ln -sfn "$version" current
ln -sf "$root/current/AppRun" /usr/local/bin/dimos
echo "DimOS $version is installed in $root/$version"
