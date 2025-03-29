import time


def keepalive():
    try:
        # Keep the program running
        print("Press Ctrl+C to exit")
        print("-" * 60)
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping pipeline")
