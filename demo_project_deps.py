from pathlib import Path
import re
source=Path('pyproject.toml').read_text()
match=re.search(r'(?m)^dependencies = \[\n(.*?)^\]',source,re.S)
for line in match.group(1).splitlines():
    if line.strip().startswith(('#','"')):
        print(line.strip().rstrip(','))
