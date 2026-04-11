# Nix Build Workaround for ARM64 Kernel 5.10 (Rockchip RK3588)

## The Problem

Nix builds fail on M20's NOS board during `unpackPhase` and `fixupPhase`:

```
unpacking source archive /nix/store/hash-source
cp: setting permissions for 'hash-source': No such file or directory
do not know how to unpack source archive /nix/store/hash-source
```

## Root Cause

**Confirmed via Codex/GPT-5.4 diagnosis + manual investigation:**

NOS runs a Rockchip vendor kernel **5.10.198** on Ubuntu 20.04 (glibc 2.31). Nix's bundled coreutils is compiled against **glibc 2.42**, which uses the `fchmodat2` syscall. This syscall was only added in kernel **6.6**. When nix's `cp` or `chmod` tries to set permissions, the syscall fails silently with `ENOENT`.

Two separate issues compound:

1. **`fchmodat2` syscall missing** — nix's coreutils (glibc 2.42) calls `fchmodat2` which kernel 5.10 doesn't have. This affects `unpackPhase` (source copy) and `fixupPhase` (binary permissions).

2. **overlayfs root filesystem** — The Rockchip vendor kernel uses overlayfs for the root partition. Nix's directory-source unpack path (`cp -r` from store) has additional permission/copy-up issues on overlayfs. Moving `/nix` to ext4 via bind mount resolves this part.

The sandbox setting (`--sandbox true/false`) doesn't help — the issue is in the coreutils binary itself, not the sandbox mechanism.

## Environment Details

| Component | Version |
|-----------|---------|
| Board | Deep Robotics M20 NOS (Rockchip RK3588) |
| Kernel | 5.10.198 (vendor backport, not mainline) |
| OS | Ubuntu 20.04.6 LTS (Focal Fossa) |
| Arch | aarch64 (ARM64) |
| Host glibc | 2.31 |
| Nix | 2.20.9 (single-user install) |
| Nix glibc | 2.42 (from nixpkgs unstable) |
| Root FS | overlayfs |
| `/nix` FS | ext4 (bind-mounted from `/var/opt/robot/data/nix`) |

## The Fix

### Step 1: Move `/nix` off overlayfs onto ext4

The NOS root filesystem is overlayfs. `/nix/store` must be on a real filesystem (ext4, xfs, btrfs):

```bash
sudo mkdir -p /var/opt/robot/data/nix
sudo cp -a /nix/* /var/opt/robot/data/nix/
sudo rm -rf /nix && sudo mkdir /nix
sudo mount --bind /var/opt/robot/data/nix /nix
```

Note: nix rejects symlinks (`/nix -> /var/opt/robot/data/nix`), so a bind mount is required.

To persist across reboots, add to `/etc/fstab`:
```
/var/opt/robot/data/nix /nix none bind 0 0
```

### Step 2: Override `unpackPhase` and `fixupPhase`

Replace nix's coreutils with host binaries (`/usr/bin/cp`, `/usr/bin/chmod`) which link against the host's glibc 2.31 and don't use `fchmodat2`:

```nix
pkgs.stdenv.mkDerivation {
  pname = "my-module";
  version = "0.1.0";
  src = ./.;

  nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
  buildInputs = [ pkgs.lcm pkgs.glib ];

  # Workaround: nix's coreutils (glibc 2.42) uses fchmodat2 which
  # kernel 5.10 (Rockchip RK3588) doesn't have. Use host binaries.
  unpackPhase = ''
    /usr/bin/cp -r $src source
    /usr/bin/chmod -R u+w source
    cd source
  '';
  fixupPhase = ''
    /usr/bin/find $out -type f -executable -exec /usr/bin/chmod 0755 {} \;
    /usr/bin/find $out -type d -exec /usr/bin/chmod 0755 {} \;
  '';
};
```

### Step 3: Build with sandbox disabled

The sandbox would block access to `/usr/bin`, so disable it:

```bash
nix build \
  --extra-experimental-features 'nix-command flakes' \
  --option sandbox false
```

### Step 4: For upstream modules you can't modify

Use a wrapper flake that overrides the derivation:

```nix
# wrapper-flake/flake.nix
{
  inputs = {
    mod.url = "github:dimensionalOS/dimos-module-arise-slam/v0.1.0";
  };

  outputs = { self, mod, ... }: let
    system = "aarch64-linux";
    pkg = mod.packages.${system}.default;
  in {
    packages.${system}.default = pkg.overrideAttrs (old: {
      unpackPhase = ''
        /usr/bin/cp -r $src source
        /usr/bin/chmod -R u+w source
        cd source
      '';
      fixupPhase = ''
        /usr/bin/find $out -type f -executable -exec /usr/bin/chmod 0755 {} \;
        /usr/bin/find $out -type d -exec /usr/bin/chmod 0755 {} \;
      '';
    });
  };
}
```

Then build:
```bash
cd wrapper-flake && nix build \
  --extra-experimental-features 'nix-command flakes' \
  --option sandbox false
```

## What We Built Successfully

Using this workaround, all five NativeModules compile and run on NOS:

| Module | Source | Binary |
|--------|--------|--------|
| DrddsLidarBridge | Local (our code) | `drdds_lidar_bridge` |
| AriseSLAM | `dimos-module-arise-slam/v0.1.0` | `arise_slam` |
| TerrainAnalysis | `dimos-module-terrain-analysis/v0.1.1` | `terrain_analysis` |
| LocalPlanner | `dimos-module-local-planner/v0.1.1` | `local_planner` |
| PathFollower | `dimos-module-path-follower/v0.1.1` | `path_follower` |

All dependencies (LCM, glib, cmake, gcc, PCL, Ceres, etc.) are fetched from the nixpkgs binary cache — no compilation needed for deps. Only the module source code compiles locally.

## Possible Upstream Fixes

1. **Add the overrides to each `dimos-module-*` flake** behind a flag or auto-detect kernel version
2. **Create a shared nix overlay** that patches stdenv's coreutils for kernel < 6.6 targets
3. **Pin nix coreutils** to an older version that doesn't use `fchmodat2` (e.g. coreutils 9.3 or earlier)
4. **Use a static coreutils** built against musl instead of glibc (avoids the syscall issue entirely)
