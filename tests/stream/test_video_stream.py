#
#
#

"""Tests for the video streaming components in the dimos package."""

import pytest
import numpy as np
import cv2
from unittest import mock
import time

import tests.test_header

from dimos.stream.videostream import VideoStream
from dimos.stream.video_provider import VideoProvider
from dimos.stream.frame_processor import FrameProcessor


def test_video_stream_creation():
    """Test that a VideoStream can be created with a mock source."""
    with mock.patch('cv2.VideoCapture') as mock_capture:
        mock_capture.return_value.isOpened.return_value = True
        stream = VideoStream(source=0)
        assert stream is not None
        mock_capture.assert_called_once_with(0)


def test_video_provider_creation():
    """Test that a VideoProvider can be created."""
    with mock.patch('dimos.stream.video_provider.VideoProvider._initialize_capture') as mock_init:
        provider = VideoProvider(dev_name="test_device")
        assert provider is not None
        assert provider.dev_name == "test_device"


def test_video_stream_iteration():
    """Test that a VideoStream can be iterated."""
    test_frames = [np.zeros((480, 640, 3), dtype=np.uint8) for _ in range(3)]
    
    with mock.patch('cv2.VideoCapture') as mock_capture:
        mock_capture.return_value.isOpened.return_value = True
        mock_capture.return_value.read.side_effect = [(True, frame) for frame in test_frames] + [(False, None)]
        
        stream = VideoStream(source=0)
        
        frames = list(stream)
        
        assert len(frames) == len(test_frames), f"Expected {len(test_frames)} frames, got {len(frames)}"
        
        for i, frame in enumerate(frames):
            assert np.array_equal(frame, test_frames[i]), f"Frame {i} does not match expected frame"


def test_video_stream_release():
    """Test that a VideoStream can be properly released."""
    with mock.patch('cv2.VideoCapture') as mock_capture:
        mock_capture.return_value.isOpened.return_value = True
        
        stream = VideoStream(source=0)
        stream.release()
        
        mock_capture.return_value.release.assert_called_once()


def test_frame_processor():
    """Test that a FrameProcessor can process frames with transformations."""
    test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(test_frame, (100, 100), (540, 380), (255, 255, 255), 2)
    
    processor = FrameProcessor()
    
    gray_result = processor.to_grayscale(test_frame)
    assert gray_result is not None
    assert len(gray_result.shape) == 2
    assert gray_result.dtype == np.uint8
    
    edge_result = processor.edge_detection(test_frame)
    assert edge_result is not None
    assert edge_result.dtype == np.uint8
    
    resize_result = processor.resize(test_frame, scale=0.5)
    assert resize_result is not None
    assert resize_result.shape[0] == test_frame.shape[0] // 2
    assert resize_result.shape[1] == test_frame.shape[1] // 2
