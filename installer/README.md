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
dimup init my-robot --ref f4053beb18ec9b0c87e895f2b0d6ac2c5f969f14
cd my-robot
source .dimos/activate.sh
dimos run my-robot.demo
```
