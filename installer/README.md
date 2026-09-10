# dimup

Install uv and dimup, then prepare your machine from the current PR branch:

```bash
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/feat/dimup-release/installer/bootstrap.sh | bash
```

This package is independent of the DimOS runtime. It supports Ubuntu 22.04/24.04
x86_64 and Apple Silicon macOS 14+. Setup installs host packages with apt/Homebrew,
prepares Cargo and Nix, and writes a detailed log. It is safe to rerun.

The repository bootstrap installs dimup from a pinned source archive, so it needs
neither Git nor a published release. Follow the printed PATH instruction to use
dimup in your current terminal. The release bootstrap uses a checksum-verified
wheel. Both run `dimup setup` and install only the setup CLI globally.

Until the SDK changes merge, create an application with the tested PR revision:

```bash
dimup init my-robot --ref 9e0bf017e084f499efb6ec1e641cf2c6e6a40291
cd my-robot
source .dimos/activate.sh
dimos run my-robot.demo
```
