# Camera calibration

Operator workflow for chessboard targets and `dimos cameracalibrate` (ROS-style CameraInfo YAML). The square size you pass to the CLI must match the board you actually print and measure.

## Print and measure the chessboard

`dimos cameracalibrate` uses `--cols` and `--rows` as the number of **inner** corner points along each axis, the same convention as `cv2.findChessboardCorners` `patternSize=(cols, rows)`.

A practical default is **8 by 6 inner corners** on **A4**: enough intersections for a stable solve without making each square too small for a typical desk webcam.

1. **Generate a checkerboard.** OpenCV documents pattern creation in [Create calibration pattern](https://docs.opencv.org/4.12.0/da/d0d/tutorial_camera_calibration_pattern.html). To draw your own board, download and run [OpenCV's gen_pattern.py](https://github.com/opencv/opencv/blob/4.12.0/doc/pattern_tools/gen_pattern.py) from that OpenCV version (see the tutorial for dependencies). In that script, `--columns` and `--rows` count **checker squares** along each axis. For **8 by 6 inner corners**, use **9 columns and 7 rows** of squares (inner corners are one less than square count in each direction):

```bash
python gen_pattern.py -o chessboard_a4.svg -T checkerboard --columns 9 --rows 7 --square_size 25 -u mm -a A4
```

Tune `--square_size` so the pattern fits with margins; convert SVG to PDF in your viewer if needed.

2. **Print at nominal scale.** Turn off "fit to page" or other scaling that would change the printed square size relative to the file.

3. **Measure one printed square with calipers.** Use the edge length of a single black or white square on the **printed** sheet, not the value from the generator unless you verified the print. Convert to meters for `--square-size-m` (for example 24.85 mm becomes `0.02485`).

## Capture practice

- Aim for 15-25 frames with the board fully in view and inner corners detected in each; keep a few spares so you can drop outliers.
- Cover the full image over the set: include poses where the board reaches toward the frame edges and corners, not only the center.
- Vary tilt and camera-to-board distance between frames so the solver sees diverse rigid poses.
- Lock exposure and use fixed white balance when the camera or capture app allows it, so brightness does not drift across the sequence.
- Avoid motion blur: mount or brace the camera, use enough light, and only save frames when the preview is sharp (same for stills saved to a folder).

Example after you have calibration images in `./capture`:

```bash
uv run dimos cameracalibrate --source folder --images ./capture --cols 8 --rows 6 --square-size-m 0.02485 --out ./camera_info.yaml ./camera_info.preview.png
```

Wrong `--square-size-m` skews metric geometry even when reprojection error looks good.

## Run `dimos cameracalibrate`

Run from the repo or any directory where `uv run dimos` resolves (same pattern as other `dimos` CLIs). Required flags are always `--source`, `--cols`, `--rows`, and `--square-size-m`. Folder mode also requires `--images`.

**Webcam (interactive).** Open a live preview on device `--device-index`. When inner corners are detected, the board is drawn on the preview; press **Space** to accept the current frame. The CLI collects `--target-count` accepted frames (default 20) then solves. Press **q** to quit early (that aborts unless enough frames were already accepted). The detector first tries `--cols` and `--rows` as inner-corner counts, then also accepts the common square-count form (for example a 12 by 8 square board is detected as 11 by 7 inner corners).

```bash
uv run dimos cameracalibrate --source webcam --device-index 0 --cols 8 --rows 6 --square-size-m 0.02485 --out ./camera_info.yaml ./camera_info.preview.png
```

**Folder (stills).** Load every `*.png`, `*.jpg`, and `*.jpeg` in the directory, sorted by filename. Each image should show the full board with detectable inner corners.

```bash
uv run dimos cameracalibrate --source folder --images ./capture/ --cols 8 --rows 6 --square-size-m 0.02485 --out ./camera_info.yaml ./camera_info.preview.png
```

Output files are explicit. Pass `--out ./camera_info.yaml` to write the ROS CameraInfo YAML. Pass a preview PNG path immediately after it to write a corner-overlay preview, for example `--out ./camera_info.yaml ./camera_info.preview.png`. If you omit both output paths, the command still runs calibration and prints RMS, but does not write YAML or PNG files. A preview PNG path without `--out` is rejected.

Optional flags (same for both sources): `--target-count` (webcam only; default 20), `--camera-name` (default `webcam`), `--no-display` (no OpenCV window; for headless or automation), `--debug` (write detailed capture logs to the system temp directory).

On success the process prints the calibration RMS, the detected pattern, and any output paths you requested. Example:

```text
RMS: 0.342187 px (20 frame(s) used)
Detected pattern: (8, 6) (requested inner corners)
Wrote camera info YAML to camera_info.yaml
Wrote preview overlay PNG to camera_info.preview.png
```

Your RMS and frame count depend on the capture. Paths echo only the files you explicitly requested.

## Verify the YAML

You can sanity-check an existing CameraInfo YAML without rerunning calibration.

1. Open the file and confirm `image_width` and `image_height` match the resolution your camera actually delivers in the stack (same width and height as your calibration images or as your `Webcam` width and height settings). Wrong dimensions mean intrinsics are being applied to the wrong raster shape.

2. Confirm `distortion_model` is `plumb_bob`. dimos `load_camera_info` reads this key (not `camera_model`); other values are only valid if every consumer in your pipeline agrees on the same model.

3. `frame_id`: the YAML written by `dimos cameracalibrate` does not embed `frame_id`. When you build a `CameraInfo` from the file (for example `load_camera_info(path, frame_id=...)`), the `frame_id` you pass must match the `Image.header.frame_id` used for that camera in your graph. The stock `Webcam` publishes color images with frame id `camera_optical` (or `{frame_id_prefix}/camera_optical` if `frame_id_prefix` is set). The no-robot desk blueprint (ticket T5 in the feature backlog) publishes TF so that optical frame is consistent with that naming; keep your loaded `CameraInfo` header aligned with whatever your desk stack actually publishes for the optical camera.

For the shape of the ROS-style fields only (matrix blocks, row and column counts, key names), see the checked-in example [`dimos/hardware/sensors/camera/zed/single_webcam.yaml`](/dimos/hardware/sensors/camera/zed/single_webcam.yaml). Treat it as a schema reference, not as calibration numbers for your camera.
