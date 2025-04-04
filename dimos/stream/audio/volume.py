#!/usr/bin/env python3
import numpy as np


def calculate_rms_volume(audio_data: np.ndarray) -> float:
    """
    Calculate RMS (Root Mean Square) volume of audio data.

    Args:
        audio_data: Audio data as numpy array

    Returns:
        RMS volume as a float between 0.0 and 1.0
    """
    # For multi-channel audio, calculate RMS across all channels
    if len(audio_data.shape) > 1 and audio_data.shape[1] > 1:
        # Flatten all channels
        audio_data = audio_data.flatten()

    # Calculate RMS
    rms = np.sqrt(np.mean(np.square(audio_data)))

    # For int16 data, normalize to [0, 1]
    if audio_data.dtype == np.int16:
        rms = rms / 32768.0

    return rms


def calculate_peak_volume(audio_data: np.ndarray) -> float:
    """
    Calculate peak volume of audio data.

    Args:
        audio_data: Audio data as numpy array

    Returns:
        Peak volume as a float between 0.0 and 1.0
    """
    # For multi-channel audio, find max across all channels
    if len(audio_data.shape) > 1 and audio_data.shape[1] > 1:
        # Flatten all channels
        audio_data = audio_data.flatten()

    # Find absolute peak value
    peak = np.max(np.abs(audio_data))

    # For int16 data, normalize to [0, 1]
    if audio_data.dtype == np.int16:
        peak = peak / 32768.0

    return peak


if __name__ == "__main__":
    # Example usage
    import time
    from .node_simulated import SimulatedAudioSource

    # Create a simulated audio source
    audio_source = SimulatedAudioSource()

    # Create observable and subscribe to get a single frame
    audio_observable = audio_source.capture_audio_as_observable()

    def process_frame(frame):
        # Calculate and print both RMS and peak volumes
        rms_vol = calculate_rms_volume(frame.data)
        peak_vol = calculate_peak_volume(frame.data)

        print(f"RMS Volume: {rms_vol:.4f}")
        print(f"Peak Volume: {peak_vol:.4f}")
        print(f"Ratio (Peak/RMS): {peak_vol/rms_vol:.2f}")

    # Set a flag to track when processing is complete
    processed = {"done": False}

    def process_frame_wrapper(frame):
        # Process the frame
        process_frame(frame)
        # Mark as processed
        processed["done"] = True

    # Subscribe to get a single frame and process it
    subscription = audio_observable.subscribe(
        on_next=process_frame_wrapper, on_completed=lambda: print("Completed")
    )

    # Wait for frame processing to complete
    while not processed["done"]:
        time.sleep(0.01)

    # Now dispose the subscription from the main thread, not from within the callback
    subscription.dispose()
