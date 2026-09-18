#!/usr/bin/env bash
# Run the eval, then open up what it wrote on the /state bind mount. Everything
# in the container runs as root, and the eval runner creates its run directory
# with owner-only permissions, so without this the host user cannot read (or
# delete) results, recordings and Rerun files.
"$@"
rc=$?
chmod -R a+rwX /state/dimos/evals /state/dimos/recordings 2>/dev/null || true
exit $rc
