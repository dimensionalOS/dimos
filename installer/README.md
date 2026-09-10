# dimup

For the complete test procedure and expected results, see the
[installation test guide](../docs/installation/installer.md).

Install uv and dimup, then prepare your machine from the current PR branch:

```bash
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/feat/dimup-release/installer/bootstrap.sh \
  | DIMUP_REF=feat/dimup-release bash
```

This package is independent of the DimOS runtime. Automatic setup covers Ubuntu
22.04/24.04 x86_64 and Apple Silicon macOS 14+. Other platforms, including Arch
Linux, receive manual prerequisite instructions; `dimup init` has no host-platform
gate. Automatic setup installs host packages with apt/Homebrew,
prepares Cargo and Nix, and writes a detailed log. It is safe to rerun.

The bootstrap downloads the dimOS repository and installs its `installer/`
directory with uv. It defaults to `main`; set `DIMUP_REF` to a branch, tag, or
commit when testing another version. No Git installation or separate package
release is required. Follow the printed PATH instruction to use dimup in the
current terminal.

`DIMUP_REF` selects the installer source. `dimup init --ref` separately selects
the SDK revision pinned in the generated application.

Until the SDK changes merge, create an application from the PR branch:

```bash
dimup init my-robot --ref feat/dimup-release
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
