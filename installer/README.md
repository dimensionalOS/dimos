# dimup machine setup

Install uv and dimup, then prepare your machine from this PR branch:

```bash
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/feat/dimup-setup/installer/bootstrap.sh | bash
```

The bootstrap installs the standalone dimup tool and runs `dimup setup`. Follow
its printed PATH instruction to use dimup in the current terminal. Rerun setup
when prerequisites are missing:

```bash
dimup setup
```

Automatic setup covers Ubuntu 22.04/24.04 x86_64 and Apple Silicon macOS 14+.
Other platforms, including Arch Linux, receive manual prerequisite instructions.
Setup installs apt/Homebrew dependencies and prepares Cargo, Nix, and Deno.
It is safe to rerun. Tool output streams to the terminal and is saved in the
setup log; each stage reports its elapsed time. `NO_COLOR=1` disables color.

The repository bootstrap uses a pinned source archive. Release packaging creates
a standalone wheel, a checksum file, and a bootstrap that verifies the wheel.
These are GitHub release assets; SDK PyPI uploads remain separate.
