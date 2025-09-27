from abc import ABC, abstractmethod

import numpy as np


class VlModel(ABC):
    @abstractmethod
    def query(self, image: np.ndarray, query: str) -> str: ...
