#!/usr/bin/env bash
# Runs as root: keep what lands on /state readable and deletable by the host user.
umask 0000
exec "$@"
