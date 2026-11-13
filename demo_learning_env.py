from pathlib import Path
import importlib.util
for name in ('torch','lerobot','cv2','dimos_lcm'):
    spec=importlib.util.find_spec(name)
    print(name, spec.origin if spec else None)
for root in (Path('/home/cc/.cache/uv'), Path('/home/cc/work/r1pro-supermimic-aligned-v1')):
    for p in root.glob('**/lerobot/policies/act/modeling_act.py'):
        print(p)
