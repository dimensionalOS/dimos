from typing import TypedDict

import numpy as np


class Frame(TypedDict):
    frame: np.ndarray  # The actual image data from cv2
    timestamp: float  # Unix timestamp when frame was captured
    frame_number: int  # Sequential frame number
