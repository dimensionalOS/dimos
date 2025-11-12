# Package Usage

## With `uv`

Init your repo if not already done:

```bash
uv init
```

Install:

```bash
uv add dimos[dev,cpu,sim]
```

Test the Unitree Go2 robot in the simulator:

```bash
uv run dimos-robot --simulation run unitree-g1
```

Run your actual robot:

```bash
uv run dimos-robot --robot-ip=192.168.X.XXX run unitree-g1
```

### Without installing

With `uv` you can run tools without having to explicitly install:

```bash
uvx --from dimos dimos-robot --robot-ip=192.168.X.XXX run unitree-g1
```

## With `pip`

Create an environment if not already done:

```bash
python -m venv .venv
. .venv/bin/activate
```

Install:

```bash
pip install dimos[dev,cpu,sim]
```

Test the Unitree Go2 robot in the simulator:

```bash
dimos-robot --simulation run unitree-g1
```

Run your actual robot:

```bash
dimos-robot --robot-ip=192.168.X.XXX run unitree-g1
```
