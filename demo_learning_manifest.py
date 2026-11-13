import json
from pathlib import Path

p = Path("/home/cc/work/dimos-bimanual-annotation-artifacts/session_v1/manifest.json")
v = json.loads(p.read_text())
print(v.keys())
for k in v:
    if k in ("segments", "statusEvents"):
        print(k, json.dumps(v[k])[:6000])
