from pathlib import Path
import json
from dimos.imitation.learning.media import capture
info = capture(Path('/home/cc/work/dimos-bimanual-annotation-artifacts/session_v1'))
print(json.dumps({k:v for k,v in info.items() if k != 'schema'}, indent=2))
