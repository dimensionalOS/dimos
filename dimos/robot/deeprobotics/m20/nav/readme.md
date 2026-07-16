# M20 Simple Navigation

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [1. Switch to the DimOS Workspace](#1-switch-to-the-dimos-workspace)
  - [2. Register All Blueprints](#2-register-all-blueprints)
  - [3. First Launch](#3-first-launch)
- [Running the System](#running-the-system)
- [Using the Navigation System](#using-the-navigation-system)
- [Troubleshooting](#troubleshooting)
  - [1. Failed to Build the Ray Tracing Module](#1-failed-to-build-the-ray-tracing-module)
  - [2. Cargo.lock Version Error](#2-cargolock-version-error)
  - [3. Cargo Registry (rsproxy) Error](#3-cargo-registry-rsproxy-error)
- [Notes](#notes)

---

# Prerequisites

Before getting started, make sure that:

- The DimOS environment has been installed successfully.
- You are working from the DimOS workspace.
- The robot is powered on and connected to the same network as your workstation.

---

# Installation

## 1. Switch to the DimOS Workspace

Before compiling or launching the navigation system, switch to the latest `main` branch:

```bash
cd ~/workplace/dimos
git fetch origin
git switch main
```

---

## 2. Register All Blueprints

Run the following command once after setting up the environment:

```bash
cd ~/workplace/dimos
source .venv/bin/activate

uv run pytest dimos/robot/test_all_blueprints_generation.py
```

---

## 3. First Launch

Launch the navigation system:

```bash
cd ~/workplace/dimos
source .venv/bin/activate

dimos --transport=zenoh \
  --zenoh-connect tcp/<aos_ip>:7447 \
  run m20-simple-nav
```

Replace `<aos_ip>` with the IP address of the robot's **AOS module**.

For the M20 robot, the AOS module IP address is 10.21.33.103. You can use this address directly.
The examples below use the default M20 AOS module IP address (`10.21.31.103`). If your deployment uses a different IP address (for example, due to custom network routing or forwarding), simply replace it with the appropriate address for your setup.

Such as:

```bash
dimos --transport=zenoh \
  --zenoh-connect tcp/10.21.31.103:7447 \
  run m20-simple-nav
```

Internal documentation:

https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/14lgGw3P8vv3mxz3Udj5N6y585daZ90D?dontjump=true


On the first launch, the ray tracing voxel mapping module is compiled automatically. This process may take several minutes.


---

# Running the System

Activate the Python environment:

```bash
cd ~/workplace/dimos
source .venv/bin/activate
```

Launch the navigation system:

```bash
dimos --transport=zenoh \
  --zenoh-connect tcp/<10.21.31.103>:7447 \
  run m20-simple-nav
```

To load the checked-in real-robot trajectory generation and smoothing profile:

```bash
dimos --transport=zenoh \
  --zenoh-connect tcp/<aos_ip>:7447 \
  run m20-simple-nav \
  --config dimos/robot/deeprobotics/m20/config/m20_simple_nav.yaml
```

The initial profile covers the A* objective and final-path diagnostics/smoothing.
Raw-path publication and constrained smoothing are disabled by default. Edit the
profile and restart DimOS to enable them during controlled physical validation.
The profile does not change mapping, CostMapper, local-controller, or MuJoCo settings.



---

# Using the Navigation System

After the system starts successfully:

- Click the **keyboard icon** in the lower-right corner to enable keyboard teleoperation.
- Click any reachable **blue region** on the map to send a navigation goal.
- The robot will automatically generate a path and navigate to the selected destination.

---

# Troubleshooting

## 1. Failed to Build the Ray Tracing Module

If the navigation system fails during startup, manually compile the Rust ray tracing executable:

```bash
cd dimos/mapping/ray_tracing/rust

cargo build --release --bin voxel_ray_tracing
```

---

## 2. Cargo.lock Version Error

If the following error appears:

```text
error: failed to parse lock file

lock file version 4 requires -Znext-lockfile-bump
```

Update the Rust toolchain:

```bash
rustup toolchain install stable
rustup update stable

cd ~/workplace/dimos
source .venv/bin/activate

rustup override set stable

source "$HOME/.cargo/env"
```

Then launch the navigation system again:

```bash
dimos --transport=zenoh \
  --zenoh-connect tcp/<10.21.31.103>:7447 \
  run m20-simple-nav
```

---

## 3. Cargo Registry (rsproxy) Error

If Cargo reports an error similar to:

```text
failed to fetch https://rsproxy.cn/crates.io-index
```

Remove the old Cargo mirror configuration:

```bash
mv ~/.cargo/config ~/.cargo/config.disabled
mv ~/.cargo/config.toml ~/.cargo/config.toml.disabled
```

Then rerun:

```bash
dimos --transport=zenoh \
  --zenoh-connect tcp/<aos_ip>:7447 \
  run m20-simple-nav
```

The project should now compile successfully.

---

# Notes

- The first launch may take **2–10 minutes**, depending on your machine, because the Rust ray tracing module is compiled automatically.
- The default IP address of the robot's **AOS module** is `10.21.31.103`. If your deployment uses a different IP address, replace it accordingly before launching the navigation system.
- Ensure the robot is powered on and connected to the same network as the host machine.
- If the build fails, refer to the **Troubleshooting** section before rerunning the navigation system.
- Manual compilation of the Rust ray tracing module is normally required only once unless its source code changes.
