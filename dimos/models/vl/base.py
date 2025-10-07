from abc import ABC, abstractmethod

import numpy as np

from dimos.msgs.sensor_msgs import Image


class VlModel(ABC):
    @abstractmethod
    def query(self, image: Image | np.ndarray, query: str) -> str: ...
