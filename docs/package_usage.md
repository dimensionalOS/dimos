# Package Usage

## With `uv`

Init your repo if not already done:

```bash
uv init
```

Install:

```bash
uv add dimos[base,dev,unitree]
```

Test the Unitree Go2 robot in the simulator:

```bash
uv run dimos --simulation run unitree-go2
```

Run your actual robot:

```bash
uv run dimos --robot-ip=192.168.X.XXX run unitree-go2
```

### Without installing

With `uv` you can run tools without having to explicitly install:

```bash
uvx --from dimos[base,unitree] dimos --robot-ip=192.168.X.XXX run unitree-go2
```

## With `pip`

Create an environment if not already done:

```bash
python -m venv .venv
. .venv/bin/activate
```

Install:

```bash
pip install dimos[base,dev,unitree]
```

Test the Unitree Go2 robot in the simulator:

```bash
dimos --simulation run unitree-go2
```

Run your actual robot:

```bash
dimos --robot-ip=192.168.X.XXX run unitree-go2
```
