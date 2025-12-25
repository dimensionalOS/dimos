# DimOS

## Instalation

Clone the repo:

```bash
git clone --single-branch git@github.com:dimensionalOS/dimos.git
cd dimos
```

### System dependencies

Tested on Ubuntu 22.04/24.04.

```bash
sudo apt update
sudo apt install git-lfs python3-venv python3-pyaudio portaudio19-dev libturbojpeg0-dev
```

### Python dependencies

Install `uv` by [following their instructions](https://docs.astral.sh/uv/getting-started/installation/) or just run:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Install Python dependencies:

```bash
uv sync
```

Depending on what you want to test you might want to install more optional dependencies as well (recommended):

```bash
uv sync --extra dev --extra cpu --extra sim --extra drone
```

### Install Foxglove Studio (robot visualization and control)

(This will be obsolete once we finish our migration to opensource [Rerun](https://rerun.io/).)

Download and install [Foxglove Studio](https://foxglove.dev/download):

```bash
wget https://get.foxglove.dev/desktop/latest/foxglove-studio-latest-linux-amd64.deb
sudo apt install ./foxglove-studio-*.deb
```

[Register an account](https://app.foxglove.dev/signup) to use it.

Open Foxglove Studio:

```bash
foxglove-studio
```

Click on "Open connection". In the popup window leave the WebSocket URL as `ws://localhost:8765` and click on "Open".

You need to load our dashboard. In the top right, click on the "Default" dropdown, and then "Import from file...". Go to the `dimos` repo and select `assets/foxglove_dashboards/unitree.json`.

### Test the install

Run the Python tests:

```bash
uv run pytest dimos
```

They should all pass in about 3 minutes.

### Test a robot replay

Run the system by playing back recorded data from a robot:

```bash
uv run dimos --replay run unitree-go2-basic
```

You can visualize the robot data in Foxglove Studio.

### Run a simulation

```bash
uv run dimos --simulation run unitree-go2-basic
```

This will open a MuJoCo simulation window. You can also visualize data in Foxglove.

If you want to also teleoperate the simulated robot run:

```bash
uv run dimos --simulation run unitree-go2-basic --extra-module keyboard_teleop
```

This will also open a Keyboard Teleop window. Focus on the window and use WASD to control the robot.

### Command center

You can also control the robot from the `command-center` extension to Foxglove.

First, pull the LFS file:

```bash
git lfs pull --include="assets/dimensional.command-center-extension-0.0.1.foxe"
```

To install it, drag that file over the Foxglove Studio window. The extension will be installed automatically. Then, click on the "Add panel" icon on the top right and add "command-center".

You can now click on the map to give it a travel goal, or click on "Start Keyboard Control" to teleoperate it.
