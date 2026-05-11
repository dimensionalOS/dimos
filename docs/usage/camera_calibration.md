# Camera calibration

Operator workflow for chessboard targets and `dimos cameracalibrate` (ROS-style CameraInfo YAML). The square size you pass to the CLI must match the board you actually print and measure.

## Print and measure the chessboard

`dimos cameracalibrate` uses `--cols` and `--rows` as the number of **inner** corner points along each axis, the same convention as `cv2.findChessboardCorners` `patternSize=(cols, rows)`.

A practical default is **8 by 6 inner corners** on **A4**: enough intersections for a stable solve without making each square too small for a typical desk webcam.

1. **Generate a checkerboard.** OpenCV documents pattern creation in [Create calibration pattern](https://docs.opencv.org/4.12.0/da/d0d/tutorial_camera_calibration_pattern.html). To draw your own board, run upstream [`gen_pattern.py`](https://github.com/opencv/opencv/blob/4.12.0/doc/pattern_tools/gen_pattern.py) from that OpenCV version (see the tutorial for dependencies). In `gen_pattern.py`, `--columns` and `--rows` count **checker squares** along each axis. For **8 by 6 inner corners**, use **9 columns and 7 rows** of squares (inner corners are one less than square count in each direction):

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
uv run dimos cameracalibrate --source folder --images ./capture --cols 8 --rows 6 --square-size-m 0.02485 --out ./camera_info.yaml
```

Wrong `--square-size-m` skews metric geometry even when reprojection error looks good.

## Run `dimos cameracalibrate`

Run from the repo or any directory where `uv run dimos` resolves (same pattern as other `dimos` CLIs). Required flags are always `--source`, `--cols`, `--rows`, `--square-size-m`, and `--out`. Folder mode also requires `--images`.

**Webcam (interactive).** Open a live preview on device `--device-index`. When inner corners are detected, the board is drawn on the preview; press **Space** to accept the current frame. The CLI collects `--target-count` accepted frames (default 20) then solves. Press **q** to quit early (that aborts unless enough frames were already accepted).

```bash
uv run dimos cameracalibrate --source webcam --device-index 0 --cols 8 --rows 6 --square-size-m 0.02485 --out ./camera_info.yaml
```

**Folder (stills).** Load every `*.png`, `*.jpg`, and `*.jpeg` in the directory, sorted by filename. Each image should show the full board with detectable inner corners.

```bash
uv run dimos cameracalibrate --source folder --images ./capture/ --cols 8 --rows 6 --square-size-m 0.02485 --out ./camera_info.yaml
```

Optional flags (same for both sources): `--target-count` (webcam only; default 20), `--camera-name` (default `webcam`), `--frame-id` (default `camera_optical`; accepted by the CLI but not written into the YAML, which matches the ROS CameraInfo file schema), `--no-display` (no OpenCV window; for headless or automation).

On success the process prints the calibration RMS, the YAML path, and a preview PNG path (same basename as `--out`, suffix `.preview.png`). Example:

```text
RMS: 0.342187 px (20 frame(s) used)
Wrote camera info YAML to camera_info.yaml
Wrote preview overlay PNG to camera_info.preview.png
```

Your RMS and frame count depend on the capture; paths echo the `--out` you passed (for example `./camera_info.yaml` if you used that form).
