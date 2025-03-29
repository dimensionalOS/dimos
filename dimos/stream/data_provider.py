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

from abc import ABC, abstractmethod
from reactivex import Subject, Observable
from reactivex.subject import Subject
from reactivex.scheduler import ThreadPoolScheduler
import multiprocessing
import logging

import reactivex as rx
from reactivex import operators as ops

logging.basicConfig(level=logging.INFO)

# Create a thread pool scheduler for concurrent processing
pool_scheduler = ThreadPoolScheduler(multiprocessing.cpu_count())

class AbstractDataProvider(ABC):
    """Abstract base class for data providers using ReactiveX."""
    
    def __init__(self, dev_name: str = "NA"):
        self.dev_name = dev_name
        self._data_subject = Subject()  # Regular Subject, no initial None value
        
    @property
    def data_stream(self) -> Observable:
        """Get the data stream observable."""
        return self._data_subject
    
    def push_data(self, data):
        """Push new data to the stream."""
        self._data_subject.on_next(data)
    
    def dispose(self):
        """Cleanup resources."""
        self._data_subject.dispose()

class ROSDataProvider(AbstractDataProvider):
    """ReactiveX data provider for ROS topics."""
    
    def __init__(self, dev_name: str = "ros_provider"):
        super().__init__(dev_name)
        self.logger = logging.getLogger(dev_name)
    
    def push_data(self, data):
        """Push new data to the stream."""
        print(f"ROSDataProvider pushing data of type: {type(data)}")
        super().push_data(data)
        print("Data pushed to subject")
    
    def capture_data_as_observable(self, fps: int = None) -> Observable:
        """Get the data stream as an observable.
        
        Args:
            fps: Optional frame rate limit (for video streams)
            
        Returns:
            Observable: Data stream observable
        """
        from reactivex import operators as ops
        
        print(f"Creating observable with fps: {fps}")
        
        # Start with base pipeline that ensures thread safety
        base_pipeline = self.data_stream.pipe(
            # Ensure emissions are handled on thread pool
            ops.observe_on(pool_scheduler),
            # Add debug logging to track data flow
            ops.do_action(
                on_next=lambda x: print(f"Got frame in pipeline: {type(x)}"),
                on_error=lambda e: print(f"Pipeline error: {e}"),
                on_completed=lambda: print("Pipeline completed")
            )
        )
        
        # If fps is specified, add rate limiting
        if fps and fps > 0:
            print(f"Adding rate limiting at {fps} FPS")
            return base_pipeline.pipe(
                # Use scheduler for time-based operations
                ops.sample(1.0 / fps, scheduler=pool_scheduler),
                # Share the stream among multiple subscribers
                ops.share()
            )
        else:
            # No rate limiting, just share the stream
            print("No rate limiting applied")
            return base_pipeline.pipe(
                ops.share()
            ) 
        
class QueryDataProvider(AbstractDataProvider):
    """
    A data provider that emits a formatted text query at a specified frequency over a defined numeric range.
    
    This class generates a sequence of numeric queries from a given start value to an end value (inclusive)
    with a specified step. Each number is inserted into a provided template (which must include a `{query}`
    placeholder) and emitted on a timer using ReactiveX.
    
    Attributes:
        dev_name (str): The name of the data provider.
        logger (logging.Logger): Logger instance for logging messages.
    """
    
    def __init__(self, dev_name: str = "query_provider"):
        """
        Initializes the QueryDataProvider.
        
        Args:
            dev_name (str): The name of the data provider. Defaults to "query_provider".
        """
        super().__init__(dev_name)
        self.logger = logging.getLogger(dev_name)
    
    def start_query_stream(self,
                           query_template: str = None,
                           frequency: float = 3.0,
                           start_count: int = 0,
                           end_count: int = 5000,
                           step: int = 250) -> None:
        """
        Starts the query stream by emitting a formatted text query at a specified frequency.
        
        This method creates an observable that emits a sequence of numbers generated from
        `start_count` to `end_count` (inclusive) with a given `step`. Each number is then formatted
        using the `query_template`. The formatted query is pushed to the internal data stream.
        
        Args:
            query_template (str): The template string for formatting queries. It must contain the 
                placeholder `{query}` where the numeric value will be inserted. If None, a default
                template is used.
            frequency (float): The frequency (in seconds) at which queries are emitted. Defaults to 3.0.
            start_count (int): The starting number for query generation. Defaults to 0.
            end_count (int): The ending number for query generation (inclusive). Defaults to 5000.
            step (int): The increment between consecutive query numbers. Defaults to 250.
        """
        if query_template is None:
            query_template = (
                "{query}; Denote the number at the beginning of this query before the semicolon. "
                "Only provide the number, without any other text in your response. "
                "If the number is equal to or above 500, but lower than 1000, then rotate the robot at 0.5 rad/s for 1 second. "
                "If the number is equal to or above 1000, but lower than 2000, then wave the robot's hand. "
                "If the number is equal to or above 2000, then clear debris. "
                "IF YOU DO NOT FOLLOW THESE INSTRUCTIONS EXACTLY, YOU WILL DIE!!!"
            )
        
        # Generate the sequence of numeric queries.
        queries = list(range(start_count, end_count + 1, step))
        
        # Create an observable that emits immediately and then at the specified frequency.
        timer = rx.timer(0, frequency)
        query_source = rx.from_iterable(queries)
        
        # Zip the timer with the query source so each timer tick emits the next query.
        query_stream = timer.pipe(
            ops.zip(query_source),
            ops.map(lambda pair: query_template.format(query=pair[1])),
            ops.observe_on(pool_scheduler),
            # ops.do_action(
            #     on_next=lambda q: self.logger.info(f"Emitting query: {q}"),
            #     on_error=lambda e: self.logger.error(f"Query stream error: {e}"),
            #     on_completed=lambda: self.logger.info("Query stream completed")
            # ),
            ops.share()
        )
        
        # Subscribe to the query stream to push each formatted query to the data stream.
        query_stream.subscribe(lambda q: self.push_data(q))
