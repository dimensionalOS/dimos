#!/usr/bin/env bash
# Everything in the container runs as root, so leave what the eval writes on the
# /state bind mount readable and deletable by the host user from the moment it
# is written, including the partial results of an eval stopped midway.
umask 0000
exec "$@"
