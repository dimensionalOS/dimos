# ORB-SLAM3 Native Module

Visual SLAM module wrapping [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) as a native subprocess.

The C++ binary lives in a separate GPL-3.0 repo ([dimos-orb-slam3](https://github.com/dimensionalOS/dimos-orb-slam3)) and is pulled in at build time via Nix.

## Known Issues

- **Transform / trajectory reconstruction mismatch**: The reconstructed trajectory does not match ground-truth poses. There is a suspected coordinate-frame or transform-composition issue causing output to diverge from base truth. Needs investigation.
