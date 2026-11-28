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
exec 9<"$root"
flock 9
stage="$(mktemp -d "$root/.install.XXXXXXXX")"
trap 'rm -rf -- "$stage"' EXIT
trap 'exit 1' HUP INT TERM
cd "$stage"
[ -x "$image" ] || chmod +x "$image"
# The version comes first, from one small file.
"$image" --appimage-extract dimos.desktop > /dev/null
version="$(sed -n 's/^X-AppImage-Version=//p' squashfs-root/dimos.desktop)"
case "$version" in
  ''|*/*|.|..|current) echo "install.sh: invalid image version '$version'" >&2; exit 1 ;;
esac
if [ -e "$root/$version" ] || [ -L "$root/$version" ]; then
  echo "install.sh: $root/$version is already there" >&2
  exit 1
fi
"$image" --appimage-extract > /dev/null
mv -T squashfs-root "$root/$version"
ln -sfnT "$version" "$root/current"
ln -sfT "$root/current/AppRun" /usr/local/bin/dimos
echo "DimOS $version is installed in $root/$version"
