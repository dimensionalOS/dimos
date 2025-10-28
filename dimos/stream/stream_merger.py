# Copyright 2025 Dimensional Inc.
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

from typing import TypeVar

from reactivex import Observable, operators as ops

T = TypeVar("T")
Q = TypeVar("Q")


def create_stream_merger(
    data_input_stream: Observable[T], text_query_stream: Observable[Q]
) -> Observable[tuple[Q, list[T]]]:
    """
    Creates a merged stream that combines the latest value from data_input_stream
    with each value from text_query_stream.

    Args:
        data_input_stream: Observable stream of data values
        text_query_stream: Observable stream of query values

    Returns:
        Observable that emits tuples of (query, latest_data)
    """
    # Encompass any data items as a list for safe evaluation
    safe_data_stream = data_input_stream.pipe(
        # We don't modify the data, just pass it through in a list
        # This avoids any boolean evaluation of arrays
        ops.map(lambda x: [x])
    )

    # Use safe_data_stream instead of raw data_input_stream
    return text_query_stream.pipe(ops.with_latest_from(safe_data_stream))
