# Copyright 2026 Dimensional Inc.
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

import json

import numpy as np

from dimos.simulation.behavior.radio_diagnostic_camera import RadioDiagnosticCamera


def test_evaluator_camera_records_labelled_stage_and_releases_writer(tmp_path, mocker):
    camera = mocker.Mock()
    camera.get_obs.return_value = ({"rgb": np.zeros((20, 30, 4), dtype=np.uint8)}, {})
    cv2 = mocker.Mock()
    mocker.patch.dict("sys.modules", {"cv2": cv2})
    (tmp_path / "active-stage.json").write_text(json.dumps("close_right"))
    recorder = RadioDiagnosticCamera(camera, str(tmp_path))
    try:
        recorder.capture(123.0)
        record = json.loads((tmp_path / "evaluator-sidecam-frames.jsonl").read_text())
        assert record["evaluator_only"] is True
        assert record["stage"] == "close_right"
        assert record["timestamp"] == 123.0
        assert "EVALUATOR ONLY" in cv2.putText.call_args.args[1]
        cv2.VideoWriter.return_value.write.assert_called_once()
    finally:
        recorder.close()
    cv2.VideoWriter.return_value.release.assert_called_once()


def test_evaluator_camera_persists_sensor_failure_without_affecting_motion(tmp_path, mocker):
    camera = mocker.Mock()
    camera.get_obs.side_effect = RuntimeError("viewer unavailable")
    mocker.patch.dict("sys.modules", {"cv2": mocker.Mock()})
    recorder = RadioDiagnosticCamera(camera, str(tmp_path))
    try:
        recorder.capture(123.0)
        record = json.loads((tmp_path / "evaluator-sidecam-frames.jsonl").read_text())
        assert "viewer unavailable" in record["error"]
        assert recorder.frames == 0
    finally:
        recorder.close()
