---
name: position_of_thing
description: Use when asked for the world position (x, y) / coordinates / location of an object the robot saw in the recording — "where is X", "what are the coordinates of X", "verify where X is". Primary method is BEARING TRIANGULATION from two or more well-separated frames; falls back to hypothesise-and-verify if only one usable view exists.
---

# Locating an object the robot saw

The recording gives you each camera frame's world pose, but the objects
in the frames have no labelled coordinates. The reliable way to find
an object's `(x, y)` is to use **two or more frames as bearing rays
that intersect at the object** — depth from a single camera is too
noisy to guess. Triangulation reduces the problem to "what direction
is the object in?" — which the LLM can read from the image — rather
than "how far away is it?" — which the LLM cannot.

## When to use

- "Where is the white robot?"
- "What's the world position of the desk / man / vending machine / mural?"
- "Verify the coordinates of X"
- "Triangulate where Y is"

## Tools allowed

- `search_semantic` (find candidate frames containing the object)
- `show_image`      (look at each frame, judge the object's image column)
- `calc`            (compute bearings + run least-squares triangulation)
- `frames_facing`   (verification — projects the result back into frames)
- (When you're done, end with a plain text reply — no tool call.)

The agent's only *visual* job in this procedure is to estimate the
**horizontal pixel column** (or fraction across the image) where the
object's base appears in each frame — much easier and more reliable
than estimating depth.

## Go2 camera constants (hard-coded; do not re-derive)

```
fx = 819.553492   # focal length in pixels
cx = 625.284099   # image center x
image_width  = 1280
image_height = 720
```

## Primary procedure — bearing triangulation

1. **Find candidate frames.**
   `search_semantic("color_image_embedded", "<short description>", k=10)`.
   Note the ts + camera pose of frames where sim ≥ 0.25. If multiple
   distinct objects matched, pick one and triangulate it; mention
   others separately.

2. **Pick 2–4 frames for triangulation.** Aim for:
   - clearly different camera positions (≥ ~1 m apart in xy),
   - the object should appear at meaningfully different horizontal
     positions across the frames (left in one, right in another → good;
     same column in both → near-parallel rays, bad).
   You only know "different positions" from the camera pose in each
   `search_semantic` hit — use that to pre-filter.

3. **For each picked frame**, call
   `show_image(stream="color_image", ts=<ts>)` and look at the
   image. Estimate, as a number, the horizontal pixel column where
   the **base of the object** appears (where it meets the floor or
   its lowest visible point):
   - `px = 640` is dead-centre,
   - `px = 0` is the far left,
   - `px = 1280` is the far right.
   You can express it as a fraction of width first (e.g. "30% from
   the left") and convert: `px = fraction * 1280`. Be deliberate; this
   number drives the answer.

4. **Compute world-frame bearings in `calc`** for every chosen frame.
   Use this exact pattern:

   ```python
   calc('''
   fx, cx = 819.553492, 625.284099

   # For each frame: (cam_x, cam_y, cam_yaw_rad, pixel_col_of_object_base)
   frames = [
       (-0.26,  7.10, <yaw_rad>, <px_0>),
       ( 1.40, 12.20, <yaw_rad>, <px_1>),
       # ... more frames as needed
   ]

   def deg2rad(d):
       return d * 3.141592653589793 / 180.0

   rays = []
   for (cx_, cy_, yaw, px) in frames:
       bearing_offset = (px - cx) / fx      # tan(offset) approximation
       # exact form (safe small-angle either way):
       #   bearing_offset = atan2(px - cx, fx)
       # but Monty has no math.atan2; use the tan approximation which is
       # within 1% out to ~30deg from centre. For points near the edge,
       # use the exact-via-iteration form below.
       theta = yaw - bearing_offset   # camera +X is image-right => world (yaw - offset)
       dx = (1.0) / ((1.0 + bearing_offset * bearing_offset) ** 0.5)
       dy = bearing_offset * dx
       # Rotate body-frame ray (dx, dy) into world frame by yaw:
       sx = dx * (1.0) ; sy = dy * (1.0)  # placeholder; see expanded form below
       rays.append((cx_, cy_, theta))
   rays
   ''')
   ```

   The above shows the bearing computation. For the actual
   triangulation, use these formulas — they live in one `calc` block
   that takes the `rays` list and returns `(x, y, residual_rms)`:

   ```python
   calc('''
   # Each ray: world-frame line through (cam_x, cam_y) along bearing theta.
   # Find (x, y) minimising sum of squared perpendicular distances.
   # Perp distance from (px, py) to ray through (cx, cy) with direction
   # (cos t, sin t) is |(px - cx) sin t - (py - cy) cos t|.
   # Setting d/dx and d/dy of the sum-of-squares to zero gives a 2x2
   # linear system:
   #   [sum(sin^2)        -sum(sin*cos)] [x]   [sum(sin*(cx*sin - cy*cos))]
   #   [-sum(sin*cos)      sum(cos^2)  ] [y] = [-sum(cos*(cx*sin - cy*cos))]

   def cos_(t):
       # cos via Taylor (Monty has no math.cos)
       # reduce to [-pi, pi]
       PI = 3.141592653589793
       while t > PI:  t -= 2*PI
       while t < -PI: t += 2*PI
       t2 = t*t
       return 1 - t2/2 + t2*t2/24 - t2*t2*t2/720 + t2*t2*t2*t2/40320

   def sin_(t):
       PI = 3.141592653589793
       return cos_(t - PI/2)

   def solve_triangulation(rays):
       A00 = 0.0; A01 = 0.0; A11 = 0.0
       b0 = 0.0;  b1 = 0.0
       for (cx_, cy_, th) in rays:
           s = sin_(th); c = cos_(th)
           A00 += s*s
           A01 += -s*c
           A11 += c*c
           rhs = cx_*s - cy_*c
           b0  += s * rhs
           b1  += -c * rhs
       det = A00*A11 - A01*A01
       x = (A11*b0 - A01*b1) / det
       y = (A00*b1 - A01*b0) / det
       # Residuals: perp distance of (x,y) to each ray
       resid_sq = 0.0
       for (cx_, cy_, th) in rays:
           s = sin_(th); c = cos_(th)
           d = (x - cx_) * s - (y - cy_) * c
           resid_sq += d * d
       rms = (resid_sq / len(rays)) ** 0.5
       return (x, y, rms)

   rays = [
       # (cam_x, cam_y, world_bearing_rad)
       ...
   ]
   solve_triangulation(rays)
   ''')
   ```

   Two practical notes on running this:
   - Build `rays` by computing the bearing for each frame: the
     formula `theta = camera_yaw_rad - atan_tan_approx((px - cx)/fx)`.
     For `|px - cx| <= ~250 pixels`, the simple form
     `bearing_offset = (px - cx) / fx` (small-angle approx) is within
     ~2% — fine. For `|px - cx| > 250` you can iterate:
     `t = (px - cx) / fx; offset = t - (t**3)/3 + (t**5)/5` (atan series).
   - `camera_yaw_rad` comes from each frame's pose quaternion.
     The captions of `show_image` give `pose=(x, y, z)`, which omits
     yaw. For triangulation you DO need yaw — pull it via the
     `search_semantic` output, which includes the full pose tuple
     `pose=(x, y, z, qx, qy, qz, qw)` from which yaw is
     `atan2(2(qw*qz + qx*qy), 1 - 2(qy^2 + qz^2))`. Compute that
     conversion in the same `calc` block.

5. **Check the residual.** `solve_triangulation` returns
   `(x, y, rms)`. The `rms` is the average perpendicular distance
   (metres) from your computed point to each bearing ray.
   - `rms < 0.3 m` → tight fit; trust the answer.
   - `0.3 m ≤ rms < 1.0 m` → moderate fit; report the answer with the
     residual as an uncertainty estimate.
   - `rms ≥ 1.0 m` → triangulation failed. Causes: rays too parallel,
     wrong objects matched across frames, or pixel-column estimates
     off. Reconsider your frame choice / column estimates, or fall
     back to the alternative procedure below.

6. **Verify visually with `frames_facing`.**
   `frames_facing(x=<x>, y=<y>, k=4)`. The red cross should land near
   the base of the object in each candidate frame. If it lands clearly
   off in one frame but on in others, that frame may show a different
   object — drop it and re-triangulate with the remaining rays.

7. **Reply** with the final plain text answer:
   - the `(x, y)` from triangulation,
   - the RMS residual (uncertainty),
   - the ts of frames used,
   - a one-line caveat: "triangulated from N bearings; the residual
     is M m so the estimate is good to about that much."

## Fallback procedure — hypothesise + verify

Use this only when you have **fewer than two usable frames** (because
the object is in only one CLIP hit, or because all candidates have
near-parallel bearings).

1. Open the one frame with `show_image`. From the camera pose, the
   camera yaw, and the object's image column, compute a bearing.
2. Guess a depth (1–8 m). Compute `x_hat = cam_x + depth * cos(theta);
   y_hat = cam_y + depth * sin(theta)` in `calc`.
3. Call `frames_facing(x_hat, y_hat, k=4)`. Look at where the red
   cross lands.
4. Adjust depth and re-run. Iterate at most 3 rounds. Stop when the
   cross lands at the object's base.
5. Report the answer with an explicit ± uncertainty (typically ±0.5–1
   m) and say the estimate comes from a single-view depth guess.

## Discipline notes

- **Estimate columns, not depths.** Columns are read straight off the
  image; depth is a guess. Triangulation converts column estimates
  into depth for you.
- **Use the object's base**, not its top or centre — the floor-line
  projection is what `frames_facing`'s cross uses to verify.
- The viewing-cone map from `frames_facing` is a useful sanity check:
  if cones from your chosen cameras don't both contain the
  triangulated point, the geometry didn't work.
- Don't submit a position whose RMS residual you didn't compute or
  whose cross-on-object check you didn't run.
- If multiple distinct objects matched the query (e.g. "white robot"
  returned both a service robot and a cone robot), triangulate each
  separately and report both.
