# Continuous Integration Guide

> *If you are ******not****** editing CI-related files, you can safely ignore this document.*

Our GitHub Actions pipeline lives in **`.github/workflows/`** and is split into three top-level workflows:

| Workflow    | File          | Purpose                                                              |
| ----------- | ------------- | -------------------------------------------------------------------- |
| **cleanup** | `cleanup.yml` | Auto-formats code with *pre-commit* and pushes fixes to your branch. |
| **docker**  | `docker.yml`  | Builds (and caches) our Docker image hierarchy.                      |
| **tests**   | `tests.yml`   | Pulls the *dev* image and runs the test suite.                       |

---

## `cleanup.yml`

* Checks out the branch.
* Executes **pre-commit** hooks.
* If hooks modify files, commits and pushes the changes back to the same branch.

> This guarantees consistent formatting even if the developer has not installed pre-commit locally.

---

## `tests.yml`

* Pulls the pre-built **dev** container image.
* Executes:

```bash
pytest
```

That’s it—making the job trivial to reproduce locally via:

```bash
./bin/dev   # enter container
pytest      # run tests
```

---

## `docker.yml`

### Objectives

1. **Layered images**: each image builds on its parent, enabling parallel builds once dependencies are ready.
2. **Speed**: build children as soon as parents finish; leverage aggressive caching.
3. **Minimal work**: skip images whose context hasn’t changed.

### Current hierarchy

    
```
    ┌───┐┌────────┐
    │ros││python  │
    └┬──┘└───────┬┘
    ┌▽─────────┐┌▽──┐
    │ros-python││dev│
    └┬─────────┘└───┘
    ┌▽──────┐
    │ros-dev│
    └───────┘
```   

> **Note**: The diagram shows only currently active images; the system is extensible—new combinations are possible, builds can be run per branch and as parallel as possible

   
```
    ┌──────┐
    │ubuntu│
    └┬────┬┘
    ┌▽──┐┌▽────────────────────────┐
    │ros││python                   │
    └┬──┘└───────────────────┬────┬┘
    ┌▽─────────────────────┐┌▽──┐┌▽──────┐
    │ros-python            ││dev││unitree│
    └┬────────┬───────────┬┘└───┘└───────┘
    ┌▽──────┐┌▽─────────┐┌▽──────────┐
    │ros-dev││ros-jetson││ros-unitree│
    └───────┘└──────────┘└───────────┘
```

### Branch-aware tagging

When a branch triggers a build:

* Only images whose context changed are rebuilt.
* New images receive the tag `<image>:<branch_name>`.
* Unchanged parents are pulled from the registry, e.g.

given we made python requirements.txt changes, but no ros changes, image dep graph would look like this:

```
ghcr.io/dimensionalos/ros:dev → ghcr.io/dimensionalos/ros-python:my_branch → ghcr.io/dimensionalos/dev:my_branch
```

### Job matrix & the **check-changes** step

To decide what to build we run a `check-changes` job that compares the diff against path filters:

```yaml
filters: |
  ros:
    - .github/workflows/_docker-build-template.yml
    - .github/workflows/docker.yml
    - docker/base-ros/**

  python:
    - docker/base-python/**
    - requirements*.txt

  dev:
    - docker/dev/**
```

This populates a build matrix (ros, python, dev) with `true/false` flags.

### The dependency execution issue

Ideally a child job (e.g. **ros-python**) should depend on both:

* **check-changes** (to know if it *should* run)
* Its **parent image job** (to wait for the artifact)

GitHub Actions can’t express “run only if *both* conditions are true *and* the parent job wasn’t skipped”.

We are using `needs: [check-changes, ros]` to ensure the job runs after the ros build, but if ros build has been skipped we need `if: always()` to ensure that the build runs anyway.
Adding `always` for some reason completely breaks the conditional check, we cannot have OR, AND operators, it just makes the job _always_ run, which means we build python even if we don't need to. 

This is unfortunate as the build takes ~30 min first time (a few minutes afterwards thanks to caching) and I've spent a lot of time on this, lots of viable seeming options didn't pan out and probably we need to completely rewrite and own the actions runner and not depend on github structure at all. Single job called `CI` or something, within our custom docker image.

---

## `run-tests` (job inside `docker.yml`)

After all requested images are built, this job triggers **tests.yml**, passing the freshly created *dev* image tag so the suite runs against the branch-specific environment.
