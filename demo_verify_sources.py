"""Compare final and recorded source hashes without mutating any source."""
from pathlib import Path
import subprocess
for path in (
    '/home/cc/work/dimos-bimanual-annotation-artifacts/session_v1/recording.mcap',
    '/home/cc/work/dimos-bimanual-annotation-artifacts/session_v1/schema.json',
    '/home/cc/work/dimos-bimanual-annotation-artifacts/session_v1/manifest.json',
    '/home/cc/hermes/dimos-annotation-data/reference/session_19ep_v1/recording.mcap',
    '/home/cc/hermes/dimos-annotation-data/reference/session_19ep_v1/schema.json',
    '/home/cc/hermes/dimos-annotation-data/reference/session_fix_v1/recording.mcap',
    '/home/cc/hermes/dimos-annotation-data/reference/session_fix_v1/schema.json',
):
    print(path, subprocess.check_output(['sha256sum', path], text=True).split()[0])
