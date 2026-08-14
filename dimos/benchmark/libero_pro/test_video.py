"""Hermetic tests for LIBERO trial video artifacts."""

from pathlib import Path

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.benchmark.libero_pro import video
from dimos.benchmark.libero_pro.video import SideBySideVideo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def test_video_pairs_matching_timestamps_and_finalizes_atomically(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    writer = mocker.Mock()
    writer.isOpened.return_value = True

    def open_writer(path: Path, fps: float) -> object:
        assert fps == 20.0
        path.touch()
        return writer

    mocker.patch.object(video, "_open_writer", side_effect=open_writer)
    output = tmp_path / "trial.mp4"
    recording = SideBySideVideo(output)
    red = np.zeros((128, 128, 3), dtype=np.uint8)
    red[:, :, 0] = 255
    green = np.zeros((128, 128, 3), dtype=np.uint8)
    green[:, :, 1] = 255

    recording.add(
        "robot0_eye_in_hand",
        Image(data=green, format=ImageFormat.RGB, ts=42.0),
    )
    writer.write.assert_not_called()
    recording.add(
        "agentview",
        Image(data=red, format=ImageFormat.RGB, ts=42.0),
    )
    recording.add(
        "agentview",
        Image(data=red, format=ImageFormat.RGB, ts=41.0),
    )
    recording.finish()

    frame = writer.write.call_args.args[0]
    assert recording.frames_written == 1
    assert frame.shape == (128, 256, 3)
    assert frame[0, 0].tolist() == [0, 0, 255]
    assert frame[0, 255].tolist() == [0, 255, 0]
    assert output.is_file()
    assert not (tmp_path / "trial.partial.mp4").exists()
    writer.write.assert_called_once()
    writer.release.assert_called_once_with()


def test_video_without_a_complete_pair_does_not_publish_artifact(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    writer = mocker.Mock()
    writer.isOpened.return_value = True

    def open_writer(path: Path, _fps: float) -> object:
        path.touch()
        return writer

    mocker.patch.object(video, "_open_writer", side_effect=open_writer)
    output = tmp_path / "trial.mp4"
    recording = SideBySideVideo(output)
    recording.add(
        "agentview",
        Image(data=np.zeros((128, 128, 3), dtype=np.uint8), ts=1.0),
    )

    with pytest.raises(RuntimeError, match="no synchronized video frames"):
        recording.finish()

    assert not output.exists()
    assert not (tmp_path / "trial.partial.mp4").exists()
