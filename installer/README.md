# dimup

Install uv and dimup, then prepare your machine from the current PR branch:

```bash
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/feat/dimup-release/installer/bootstrap.sh | bash
```

This package is independent of the DimOS runtime. Automatic setup covers Ubuntu
22.04/24.04 x86_64 and Apple Silicon macOS 14+. Other platforms, including Arch
Linux, receive manual prerequisite instructions; `dimup init` has no host-platform
gate. Automatic setup installs host packages with apt/Homebrew,
prepares Cargo and Nix, and writes a detailed log. It is safe to rerun.

The repository bootstrap installs dimup from a pinned source archive, so it needs
neither Git nor a published release. Follow the printed PATH instruction to use
dimup in your current terminal. The release bootstrap uses a checksum-verified
wheel. Both run `dimup setup` and install only the setup CLI globally.

Until the SDK changes merge, create an application with the tested PR revision:

```bash
dimup init my-robot --ref f5ac2458232fc59cbaedfff4c0cbe26fac0bb79b
cd my-robot
source .dimos/activate.sh
dimos run my-robot.demo
```

During creation, dimup groups SDK metadata work into one stage and streams uv's
installation output to the terminal. Each stage finishes with its elapsed time.
The final summary prints activation and run commands, optional direnv activation,
and the full log path. Failures keep the project directory and log for inspection.
Redirected output uses plain lines; terminal output adds color and a spinner.
Set `NO_COLOR=1` to disable color.
