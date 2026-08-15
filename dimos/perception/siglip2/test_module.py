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

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.siglip2.module import SigLIP2Module
from dimos.utils.data import get_data


@pytest.mark.self_hosted
@pytest.mark.skipif_in_ci
def test_siglip2_module_process() -> None:
    """The module turns an Image into a CPU numpy patch-embedding grid."""
    image = Image.from_file(get_data("cafe.jpg")).to_rgb()
    image.frame_id = "camera_optical"

    module = SigLIP2Module()
    module.model.start()
    try:
        patches = module._process(image)

        assert isinstance(patches.vector, np.ndarray)
        assert patches.grid_shape == (16, 16)
        assert patches.dim == 768
        assert patches.frame_id == "camera_optical"
        assert patches.ts == image.ts
        assert patches.flat().shape == (256, 768)
    finally:
        module.model.stop()
        module._close_module()
