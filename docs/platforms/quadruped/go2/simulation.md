# Unitree Go2 Simulation

Run the full Go2 navigation stack without any hardware: replay recorded sessions or simulate in MuJoCo. Same code, no robot.

See [Setup](/docs/platforms/quadruped/go2/setup.md) for installing dimOS first.

## Try It: No Hardware Needed

```bash
# Replay a recorded Go2 navigation session
# The first run downloads the default Go2 recording (about 84 MB) from Git LFS.
dimos --replay run unitree-go2
```

Opens Rerun 3D visualization. Watch the Go2 map and navigate an office in real time.

## MuJoCo Simulation

```bash
# The installer's default extras include sim.
dimos --simulation run unitree-go2
```

Full navigation stack in MuJoCo: same code, simulated robot.

With the stack viewer disabled, or on Linux with neither an X11 nor a Wayland
display, the simulator runs without its native window. Odometry, lidar and camera
images continue publishing. On Linux the headless simulator uses EGL unless
you set `MUJOCO_GL` to another backend.

```bash
dimos --simulation --viewer none run unitree-go2-basic
```

To select the native window independently, pass
`--go2connection.mujoco.headless=true` or `false` after the blueprint name.
A window requires a working display.
