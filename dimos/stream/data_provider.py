from abc import ABC, abstractmethod
from reactivex import Subject, Observable
from reactivex.subject import Subject
from reactivex.scheduler import ThreadPoolScheduler
import multiprocessing
import logging

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