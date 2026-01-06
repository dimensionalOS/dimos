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

import threading
import time

from dimos.core import In, Module, Out, rpc


class Counter(Module):
    current_count: int = 0

    count_stream: Out[int]

    def __init__(self) -> None:
        super().__init__()
        self.current_count = 0

    @rpc
    def increment(self):
        """Increment the counter and publish the new value."""
        self.current_count += 1
        self.count_stream.publish(self.current_count)
        return self.current_count


class CounterValidator(Module):
    """Calls counter.increment() as fast as possible and validates no numbers are skipped."""

    count_in: In[int]

    def __init__(self, increment_func) -> None:
        super().__init__()
        self.increment_func = increment_func
        self.last_seen = 0
        self.missing_numbers = []
        self.running = False
        self.call_thread = None
        self.call_count = 0
        self.total_latency = 0.0
        self.call_start_time = None
        self.waiting_for_response = False

    @rpc
    def start(self) -> None:
        """Start the validator."""
        self.count_in.subscribe(self._on_count_received)
        self.running = True
        self.call_thread = threading.Thread(target=self._call_loop)
        self.call_thread.start()

    @rpc
    def stop(self) -> None:
        """Stop the validator."""
        self.running = False
        if self.call_thread:
            self.call_thread.join()

    def _on_count_received(self, count: int) -> None:
        """Check if we received all numbers in sequence and trigger next call."""
        # Calculate round trip time
        if self.call_start_time:
            latency = time.time() - self.call_start_time
            self.total_latency += latency

        if count != self.last_seen + 1:
            for missing in range(self.last_seen + 1, count):
                self.missing_numbers.append(missing)
                print(f"[VALIDATOR] Missing number detected: {missing}")
        self.last_seen = count

        # Signal that we can make the next call
        self.waiting_for_response = False

    def _call_loop(self) -> None:
        """Call increment only after receiving response from previous call."""
        while self.running:
            if not self.waiting_for_response:
                try:
                    self.waiting_for_response = True
                    self.call_start_time = time.time()
                    result = self.increment_func()
                    call_time = time.time() - self.call_start_time
                    self.call_count += 1
                    if self.call_count % 100 == 0:
                        avg_latency = (
                            self.total_latency / self.call_count if self.call_count > 0 else 0
                        )
                        print(
                            f"[VALIDATOR] Made {self.call_count} calls, last result: {result}, RPC call time: {call_time * 1000:.2f}ms, avg RTT: {avg_latency * 1000:.2f}ms"
                        )
                except Exception as e:
                    print(f"[VALIDATOR] Error calling increment: {e}")
                    self.waiting_for_response = False
                    time.sleep(0.001)  # Small delay on error
            else:
                # Don't sleep - busy wait for maximum speed
                pass

    @rpc
    def get_stats(self):
        """Get validation statistics."""
        avg_latency = self.total_latency / self.call_count if self.call_count > 0 else 0
        return {
            "call_count": self.call_count,
            "last_seen": self.last_seen,
            "missing_count": len(self.missing_numbers),
            "missing_numbers": self.missing_numbers[:10] if self.missing_numbers else [],
            "avg_rtt_ms": avg_latency * 1000,
            "calls_per_sec": self.call_count / 10.0 if self.call_count > 0 else 0,
        }


if __name__ == "__main__":
    import dimos.core as core
    from dimos.core import pLCMTransport

    # Start dimos with 2 workers
    client = core.start(2)

    # Deploy counter module
    counter = client.deploy(Counter)
    counter.count_stream.transport = pLCMTransport("/counter_stream")

    # Deploy validator module with increment function
    validator = client.deploy(CounterValidator, counter.increment)
    validator.count_in.transport = pLCMTransport("/counter_stream")

    # Connect validator to counter's output
    validator.count_in.connect(counter.count_stream)

    # Start modules
    validator.start()

    print("[MAIN] Counter and validator started. Running for 10 seconds...")

    # Test direct RPC speed for comparison
    print("\n[MAIN] Testing direct RPC call speed for 1 second...")
    start = time.time()
    direct_count = 0
    while time.time() - start < 1.0:
        counter.increment()
        direct_count += 1
    print(f"[MAIN] Direct RPC calls per second: {direct_count}")

    # Run for 10 seconds
    time.sleep(10)

    # Get stats before stopping
    stats = validator.get_stats()
    print("\n[MAIN] Final statistics:")
    print(f"  - Total calls made: {stats['call_count']}")
    print(f"  - Last number seen: {stats['last_seen']}")
    print(f"  - Missing numbers: {stats['missing_count']}")
    print(f"  - Average RTT: {stats['avg_rtt_ms']:.2f}ms")
    print(f"  - Calls per second: {stats['calls_per_sec']:.1f}")
    if stats["missing_numbers"]:
        print(f"  - First missing numbers: {stats['missing_numbers']}")

    # Stop modules
    validator.stop()

    # Shutdown dimos
    client.shutdown()

    print("[MAIN] Test complete.")
