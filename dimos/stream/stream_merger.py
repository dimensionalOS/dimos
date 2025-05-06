from typing import List, TypeVar, Tuple, Any
from reactivex import Observable
from reactivex import operators as ops

T = TypeVar('T')
Q = TypeVar('Q')

def create_stream_merger(
    data_input_stream: Observable[T],
    text_query_stream: Observable[Q]
) -> Observable[Tuple[Q, List[T]]]:
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
    return text_query_stream.pipe(
        ops.with_latest_from(safe_data_stream)
    )
