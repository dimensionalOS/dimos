# macOS Install (12.6 or newer)

## 1. Install Homebrew

```sh
# Official installer; follow prompts to add brew to PATH if the script says so.
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

## 2. Install system dependencies

```sh
brew install gnu-sed gcc portaudio git-lfs libjpeg-turbo python pre-commit
```

## 3. Install uv (fast Python package manager)

```sh
curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"
```

---

# Using DimOS as a Library

## 1. Create project

```sh
mkdir myproject && cd myproject
uv venv --python 3.12
source .venv/bin/activate
```

---

## 2. Install DimOS (choose ONE setup)

### 🔹 Minimal (core only)

```sh
uv pip install dimos
```

---

### 🔹 Simulation only (recommended starting point)

```sh
uv pip install 'dimos[sim,visualization]'
```

---

### 🔹 AI Agents (LLM + audio)

```sh
uv pip install 'dimos[agents,cpu]'
```

---

### 🔹 Vision / Perception

```sh
uv pip install 'dimos[perception,cpu]'
```

---

### 🔹 Web backend (FastAPI)

```sh
uv pip install 'dimos[web]'
```

---

### 🔹 Balanced full-stack (recommended)

```sh
uv pip install 'dimos[sim,agents,web,perception,visualization,cpu]'
```

---

### 🔹 Development (testing, linting)

```sh
uv pip install 'dimos[dev]'
```

---

### 🔹 Unitree robots (heavy)

```sh
uv pip install 'dimos[unitree]'
```

⚠️ This installs many dependencies automatically.

---

## ⚠️ Important (zsh users)

Always quote extras:

```sh
uv pip install 'dimos[sim,agents]'
```

Without quotes, zsh treats `[]` as a glob pattern and the command will fail.

---

# Developing on DimOS

<!-- Clone workflow: matches AGENTS.md / typical dev setup; DDS omitted unless you install that extra. -->

```sh
# Skip LFS file checkout until needed (smaller clone); fetch LFS objects when you need them.
export GIT_LFS_SKIP_SMUDGE=1

git clone -b dev https://github.com/dimensionalOS/dimos.git
cd dimos

# Project lockfile + all optional groups.
uv sync --all-extras
```

## Type checking

```sh
uv run mypy dimos
```

## Run tests

```sh
uv run pytest dimos
```
