# Development Environment Guide

## Approach

We optimise for flexibility—if your favourite editor is **notepad.exe**, you’re good to go. Everything below is tooling for convenience.

---

## Dev Containers

Dev containers give us a reproducible, container-based workspace identical to CI.

### Why use them?

* Consistent toolchain across all OSs.
* Unified formatting, linting and type-checking.
* Zero host-level dependencies (apart from Docker).

### IDE quick start

Install the *Dev Containers* plug-in for VS Code, Cursor, or your IDE of choice (you’ll likely be prompted automatically when you open our repo).

### Shell only quick start

Terminal within your IDE should use devcontainer transparently given you installed the plugin, but in case you want to run our shell without an IDE, you can use `./bin/dev`
(it depends on npm/node being installed)

```sh
./bin/dev
devcontainer CLI (https://github.com/devcontainers/cli) not found. Install into repo root? (y/n): y

added 1 package, and audited 2 packages in 8s
found 0 vulnerabilities

[1 ms] @devcontainers/cli 0.76.0. Node.js v20.19.0. linux 6.12.27-amd64 x64.
[4838 ms] Start: Run: docker start f0355b6574d9bd277d6eb613e1dc32e3bc18e7493e5b170e335d0e403578bcdb
[5299 ms] f0355b6574d9bd277d6eb613e1dc32e3bc18e7493e5b170e335d0e403578bcdb
{"outcome":"success","containerId":"f0355b6574d9bd277d6eb613e1dc32e3bc18e7493e5b170e335d0e403578bcdb","remoteUser":"root","remoteWorkspaceFolder":"/workspaces/dimos"}

  ██████╗ ██╗███╗   ███╗███████╗███╗   ██╗███████╗██╗ ██████╗ ███╗   ██╗ █████╗ ██╗
  ██╔══██╗██║████╗ ████║██╔════╝████╗  ██║██╔════╝██║██╔═══██╗████╗  ██║██╔══██╗██║
  ██║  ██║██║██╔████╔██║█████╗  ██╔██╗ ██║███████╗██║██║   ██║██╔██╗ ██║███████║██║
  ██║  ██║██║██║╚██╔╝██║██╔══╝  ██║╚██╗██║╚════██║██║██║   ██║██║╚██╗██║██╔══██║██║
  ██████╔╝██║██║ ╚═╝ ██║███████╗██║ ╚████║███████║██║╚██████╔╝██║ ╚████║██║  ██║███████╗
  ╚═════╝ ╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝  ╚═╝╚══════╝

  v_unknown:unknown | Wed May 28 09:23:33 PM UTC 2025

root@dimos:/workspaces/dimos #
```

The script will:

* Offer to npm install `@devcontainers/cli` locally (if not available globally) on first run.
* Pull `ghcr.io/dimensionalos/dev:dev` if not present (external contributors: we plan to mirror to Docker Hub).

You’ll land in the workspace as **root** with all project tooling available.

## Pre-Commit Hooks

We use [pre-commit](https://pre-commit.com) (config in `.pre-commit-config.yaml`) to enforce formatting, licence headers, EOLs, LFS checks, etc. Hooks run in **milliseconds**.
Hooks also run in CI; any auto-fixes are committed back to your PR, so local installation is optional — but gives faster feedback.

```sh
CRLF end-lines checker...................................................Passed
CRLF end-lines remover...................................................Passed
Insert license in comments...............................................Passed
ruff format..............................................................Passed
check for case conflicts.................................................Passed
check json...............................................................Passed
check toml...............................................................Passed
check yaml...............................................................Passed
format json..............................................................Passed
LFS data.................................................................Passed

```
Given your editor uses ruff via devcontainers (which it should) actual auto-commit hook won't ever reformat your code - IDE will have already done this.

### Running hooks manually

Given your editor uses git via devcontainers (which it should) auto-commit hooks will run automatically, this is in case you want to run them manually.

Inside the dev container (Your IDE will likely run this transparently for each commit if using devcontainer plugin):

```sh
pre-commit run --all-files
```

### Installing pre-commit on your host

```sh
apt install pre-commit      # or brew install pre-commit
pre-commit install          # install git hook
pre-commit run --all-files
```


---

## Testing

All tests run with **pytest** inside the dev container, ensuring local results match CI.

### Basic usage

```sh
./bin/dev          # start container
pytest             # run all tests beneath the current directory
```

Depending on which dir you are in, only tests from that dir will run, which is convinient when developing - you can frequently validate your feature tree.

Your vibe coding agent will know to use these tests via the devcontainer so it can validate it's work.


#### Useful options

| Purpose                    | Command                 |
| -------------------------- | ----------------------- |
| Show `print()` output      | `pytest -s`             |
| Filter by name substring   | `pytest -k "<pattern>"` |
| Run tests with a given tag | `pytest -m <tag>`       |


We use tags for special tests, like `vis` or `tool` for things that aren't meant to be ran in CI and when casually developing, something that requires hardware or visual inspection (pointcloud merging vis etc)

You can enable a tag by selecting -m <tag_name> - these are configured in `./pyproject.toml`

```sh
root@dimos:/workspaces/dimos/dimos # pytest -sm vis -k my_visualization
...
```

Classic development run within a subtree:

```sh
./bin/dev

... container init ...

root@dimos:/workspaces/dimos # cd dimos/robot/unitree_webrtc/
root@dimos:/workspaces/dimos/dimos/robot/unitree_webrtc # pytest
collected 27 items / 22 deselected / 5 selected

type/test_map.py::test_robot_mapping PASSED
type/test_timeseries.py::test_repr PASSED
type/test_timeseries.py::test_equals PASSED
type/test_timeseries.py::test_range PASSED
type/test_timeseries.py::test_duration PASSED

```

Showing prints:

```sh
root@dimos:/workspaces/dimos/dimos/robot/unitree_webrtc/type # pytest -s test_odometry.py
test_odometry.py::test_odometry_conversion_and_count Odom ts(2025-05-30 13:52:03) pos(→ Vector Vector([0.432199 0.108042 0.316589])), rot(↑ Vector Vector([ 7.7200000e-04 -9.1280000e-03  3.006
8621e+00])) yaw(172.3°)
Odom ts(2025-05-30 13:52:03) pos(→ Vector Vector([0.433629 0.105965 0.316143])), rot(↑ Vector Vector([ 0.003814   -0.006436    2.99591235])) yaw(171.7°)
Odom ts(2025-05-30 13:52:04) pos(→ Vector Vector([0.434459 0.104739 0.314794])), rot(↗ Vector Vector([ 0.005558   -0.004183    3.00068456])) yaw(171.9°)
Odom ts(2025-05-30 13:52:04) pos(→ Vector Vector([0.435621 0.101699 0.315852])), rot(↑ Vector Vector([ 0.005391   -0.006002    3.00246893])) yaw(172.0°)
Odom ts(2025-05-30 13:52:04) pos(→ Vector Vector([0.436457 0.09857  0.315254])), rot(↑ Vector Vector([ 0.003358   -0.006916    3.00347172])) yaw(172.1°)
Odom ts(2025-05-30 13:52:04) pos(→ Vector Vector([0.435535 0.097022 0.314399])), rot(↑ Vector Vector([ 1.88300000e-03 -8.17800000e-03  3.00573432e+00])) yaw(172.2°)
Odom ts(2025-05-30 13:52:04) pos(→ Vector Vector([0.433739 0.097553 0.313479])), rot(↑ Vector Vector([ 8.10000000e-05 -8.71700000e-03  3.00729616e+00])) yaw(172.3°)
Odom ts(2025-05-30 13:52:04) pos(→ Vector Vector([0.430924 0.09859  0.31322 ])), rot(↑ Vector Vector([ 1.84000000e-04 -9.68700000e-03  3.00945623e+00])) yaw(172.4°)
... etc
```
---

## Cheatsheet

| Action                      | Command                      |
| --------------------------- | ---------------------------- |
| Enter dev container         | `./bin/dev`                  |
| Run all pre-commit hooks    | `pre-commit run --all-files` |
| Install hooks in local repo | `pre-commit install`         |
| Run tests in current path   | `pytest`                     |
| Filter tests by name        | `pytest -k "<pattern>"`      |
| Enable stdout in tests      | `pytest -s`                  |
| Run tagged tests            | `pytest -m <tag>`            |
