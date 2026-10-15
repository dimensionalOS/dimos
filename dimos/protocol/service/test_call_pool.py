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

from contextlib import ExitStack
import pickle

import pytest

from dimos.protocol.service.call_pool import CallPool


@pytest.mark.parametrize("stopped", [False, True])
def test_pickle_rebuilds_pool_without_live_workers_or_shutdown_state(stopped):
    with ExitStack() as cleanup:
        pool = CallPool(max_workers=2)
        cleanup.callback(pool.shutdown)
        assert pool.get().submit(lambda: 42).result(timeout=2.0) == 42
        if stopped:
            pool.shutdown()

        clone = pickle.loads(pickle.dumps(pool))
        cleanup.callback(clone.shutdown)

        assert clone.get().submit(lambda: 43).result(timeout=2.0) == 43
        clone.shutdown()
        with pytest.raises(RuntimeError, match="cannot accept calls"):
            clone.get()
