#!/usr/bin/env bash
# Run the eval, then open up what it wrote on the /state bind mount. Everything
# in the container runs as root, and the eval runner creates its run directory
# with owner-only permissions, so without this the host user cannot read (or
# delete) results, recordings and Rerun files.
#
# The eval runs as a child so a `docker stop` (SIGTERM) or Ctrl-C reaches it
# while this script lives on to do the chmod; a wait cut short by the signal is
# followed by one for the eval to actually exit.
"$@" &
pid=$!
trap 'kill -TERM "$pid" 2>/dev/null' TERM INT
wait "$pid"
rc=$?
if kill -0 "$pid" 2>/dev/null; then
    wait "$pid"
    rc=$?
fi
chmod -R a+rwX /state/dimos/evals /state/dimos/recordings 2>/dev/null || true
exit $rc
