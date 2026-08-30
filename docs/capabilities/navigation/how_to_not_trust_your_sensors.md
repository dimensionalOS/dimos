# How to Not Trust Your Sensors

Sensors aren't equally reliable, and they don't all measure the same things. The goal is to tell the system **which parts of each sensor to trust.**

For each type of movement:

- **0** → ignore the sensor
- **Small number** → trust it more
- **Large number** → trust it less
- **Negative** → use the sensor's own estimate of its reliability

You can set this independently for movement (x, y, z) and rotation (roll, pitch, yaw).

---

# 🏠 Start here: Tell the system your robot is on the ground

If your robot drives indoors, it normally:

- doesn't move up or down
- doesn't roll
- doesn't pitch

Tell the system this. It prevents sensor errors from making the robot slowly sink through the floor or appear to tilt.

```python
per_dimension_error_variance=Covariance(
    z=1e-6,
    roll=1e-6,
    pitch=1e-6,
)
```

Think of this as:

> "The robot is on a flat surface. Treat these movements as impossible."

If the robot drives on ramps, make these constraints less strict.

---

# 📷 Camera / Visual Odometry

Cameras can be good at tracking some movements and bad at others. Tell the system which parts of the camera's estimate to trust.

```python
visual_odom_pose_variances=Covariance(
    x=0.01,
    y=0.01,
    z=0.0,
    roll=0.05,
    pitch=0.05,
    yaw=0.05,
)
```

For example:

- Good at forward/sideways movement → trust x and y
- Poor depth → ignore z
- Lots of motion blur → trust the whole camera less
- Trouble during fast rotation → trust yaw less

The camera must be trusted for at least one type of movement.

---

# 🛞 Wheel Encoders

Wheel encoders are good at telling you:

- how fast the robot is moving forward
- how fast it's turning

Use them for **movement**, not long-term position. Small errors accumulate over time.

```python
twist_variances=Covariance(
    x=0.01,
    yaw=0.02,
)
```

If the wheels slip, trust them less.

For skid-steer robots, wheel slip during turns is common, so you may need to trust the wheel-based turning measurement much less.

---

# 🧭 IMU

IMUs work a little differently. Instead of directly saying how much you trust them, you provide their **noise characteristics**.

These can come from the datasheet.

```python
imus=[ImuConfig(
    gyro_noise_density=...,
    gyro_random_walk=...,
    accel_noise_density=...,
    accel_random_walk=...,
)]
```

**More noise = less trust.**

If the robot vibrates, increase the accelerometer noise rather than letting the vibration look like real movement.

---

# The 4 rules

1. **Don't use a sensor for things it doesn't measure.** Ignore those dimensions.
2. **Tell the system about physically impossible movement.** For a ground robot, constrain z, roll, and pitch.
3. **Be careful with accumulated error.** Wheel odometry tells you how far you moved, but becomes less reliable as errors accumulate.
4. **Tune relative trust.** When sensors disagree, the more trusted sensor wins. Focus on the ratio between their trust rather than finding one perfect number.
