#!/usr/bin/env python3
"""
use lcm_foxglove_bridge as a module from dimos_utils
"""

import asyncio
import threading
import dimos_utils.lcm_foxglove_bridge as bridge

def run_bridge_example():
    """Example of running the bridge in a separate thread"""
    
    def bridge_thread():
        """Thread function to run the bridge"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            bridge_runner = bridge.LcmFoxgloveBridgeRunner(
                host="0.0.0.0",
                port=8765,
                debug=True,
                num_threads=4
            )

            loop.run_until_complete(bridge_runner.run())
        except Exception as e:
            print(f"Bridge error: {e}")
        finally:
            loop.close()

    thread = threading.Thread(target=bridge_thread, daemon=True)
    thread.start()
    
    print("Bridge started in background thread")
    print("Open Foxglove Studio and connect to ws://localhost:8765")
    print("Press Ctrl+C to exit")
    
    try:
        while True:
            threading.Event().wait(1)
    except KeyboardInterrupt:
        print("Shutting down...")


if __name__ == "__main__":
    run_bridge_example() 