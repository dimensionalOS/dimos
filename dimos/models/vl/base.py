from abc import ABC, abstractmethod
from typing import Union

import numpy as np

from dimos.msgs.sensor_msgs import Image


class VlModel(ABC):
    @abstractmethod
    def query(self, image: Union[Image, np.ndarray], query: str) -> str: ...
