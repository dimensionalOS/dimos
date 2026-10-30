# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Small ONNX inference shared by robot policies, independent of a physics engine."""

import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort  # type: ignore[import-untyped]

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

POLICY_CONTROL_DT = 0.02
POLICY_ACTION_SCALE = 0.5


class OnnxPolicy:
    def __init__(self, path: str) -> None:
        options = ort.SessionOptions()  # type: ignore[attr-defined]
        options.intra_op_num_threads = 1
        self._session = ort.InferenceSession(
            path, sess_options=options, providers=["CPUExecutionProvider"]
        )
        self._input = self._session.get_inputs()[0].name
        self._output = self._session.get_outputs()[0].name
        logger.info("Loaded locomotion policy", path=path, providers=self._session.get_providers())

    def predict(self, observation: NDArray[np.float32]) -> NDArray[np.float32]:
        outputs = self._session.run([self._output], {self._input: observation.reshape(1, -1)})
        return np.asarray(outputs[0][0], dtype=np.float32)
