# M20 Simple Navigation


# Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [1. Switch to the DimOS workspace](#1-switch-to-the-dimos-workspace)
  - [2. Register all blueprints](#2-register-all-blueprints)
  - [3. First Launch](#3-first-launch)
- [Running the System](#running-the-system)
  - [Terminal 1: Start the Zenoh Bridge](#terminal-1-start-the-zenoh-bridge)
  - [Terminal 2: Launch Navigation](#terminal-2-launch-navigation)
- [Using the Navigation System](#using-the-navigation-system)
- [Troubleshooting](#troubleshooting)
  - [1. Failed to Build the Ray Tracing Module](#1-failed-to-build-the-ray-tracing-module)
  - [2. Cargo.lock Version Error](#2-cargolock-version-error)
  - [3. Cargo Registry (rsproxy) Error](#3-cargo-registry-rsproxy-error)
- [Notes](#notes)
---

# Prerequisites

Before getting started, make sure that:

* The DimOS environment has been installed successfully.
* You are working from the DimOS workspace.

---

# Installation

## 1. Switch to the DimOS workspace

Before compiling or launching the navigation system, switch to latest `main` branch:

```bash
cd ~/workplace/dimos
git fetch origin
git switch main
```

---

## 2. Register all blueprints

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
dimos run m20-simple-nav
```

On the first launch, the ray tracing voxel mapping module is compiled automatically. This process may take several minutes.

If the compilation completes successfully without any errors, press **Ctrl+C** to terminate the process and continue with the steps in **Running the System**.

---

# Running the System

## Terminal 1: Start the Zenoh Bridge

Activate the Python environment:

```bash
cd ~/workplace/dimos
source .venv/bin/activate
```

Start the Zenoh bridge:

```bash
DIMOS_ZENOH_CONNECT=tcp/<m20_ip> \
python -m dimos.robot.deeprobotics.m20.zenoh_lcm_bridge
```

Replace `<m20_ip>` with the robot IP address.

Example:

```bash
DIMOS_ZENOH_CONNECT=tcp/10.21.31.103:7447 \
python -m dimos.robot.deeprobotics.m20.zenoh_lcm_bridge
```

---

## Terminal 2: Launch Navigation

Open another terminal.

Activate the Python environment:

```bash
cd ~/workplace/dimos
source .venv/bin/activate
```

Launch the navigation system:

```bash
dimos run m20-simple-nav
```

---

# Using the Navigation System

After the system starts successfully:

* Click the **keyboard icon** in the lower-right corner to enable keyboard teleoperation.
* Click any reachable **blue region** on the map to send a navigation goal.
* The robot will automatically generate a path and navigate to the selected destination.

---

# Troubleshooting

## 1. Failed to Build the Ray Tracing Module

If `dimos run m20-simple-nav` fails during startup, manually compile the Rust ray tracing executable:

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
dimos run m20-simple-nav
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
dimos run m20-simple-nav
```

The project should now compile successfully.

---

# Notes

* The first launch may take **2–10 minutes**, depending on your machine, because the Rust ray tracing module is compiled automatically.
* Always make sure the robot IP address is reachable before starting the Zenoh bridge.
* If the build fails, refer to the **Troubleshooting** section before rerunning the navigation system.
* Manual compilation of the Rust module is normally required only once unless the module source code changes.
