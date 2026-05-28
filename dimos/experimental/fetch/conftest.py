from __future__ import annotations

import sys
from pathlib import Path


FETCH_DIR = Path(__file__).resolve().parent
if str(FETCH_DIR) not in sys.path:
    sys.path.insert(0, str(FETCH_DIR))
