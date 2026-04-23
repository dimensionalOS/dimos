# M20 NOS Nix Module Wrappers

## Why this exists

The M20 NOS board runs Rockchip RK3588 + Ubuntu 20.04 with kernel 5.10
and glibc 2.31. Nix's bundled coreutils (glibc 2.42) uses the
`fchmodat2` syscall which kernel 5.10 doesn't have, so `unpackPhase`
and `fixupPhase` fail silently during `nix build`:

```
unpacking source archive /nix/store/hash-source
cp: setting permissions for 'hash-source': No such file or directory
do not know how to unpack source archive ...
```

Upstream Dimensional module flakes
(`dimensionalOS/dimos-module-{local-planner,arise-slam,...}`) don't
have the workaround baked in. These wrappers apply it via
`overrideAttrs`, so we can build the upstream modules on NOS without
modifying the upstream repos.

See `plans/m20-rosnav-migration/06-simplify-remove-container/nix-arm64-kernel510-workaround.md`
for the full root-cause explanation.

## What's here

Each subdirectory is a minimal flake that imports one upstream module
and overrides `unpackPhase`/`fixupPhase` to use host coreutils
(`/usr/bin/cp`, `/usr/bin/chmod`) which link against the host's glibc
2.31 and bypass `fchmodat2`.

| Module | Upstream | Purpose |
|---|---|---|
| `local_planner/` | `dimensionalOS/dimos-module-local-planner/v0.1.1` | Smart-nav local path planner |
| `arise_slam/` | `dimensionalOS/dimos-module-arise-slam/v0.1.0` | Legacy SLAM backend (parallel to FAST-LIO2) |
| `terrain_analysis/` | `dimensionalOS/dimos-module-terrain-analysis/v0.1.1` | Ground/obstacle classifier for the costmap |
| `path_follower/` | `dimensionalOS/dimos-module-path-follower/v0.1.1` | Pure-pursuit controller |

## Usage

On NOS (kernel 5.10):

```bash
# Build one module (output → result/ symlink):
cd dimos/robot/deeprobotics/m20/nix_wrappers/local_planner
nix --extra-experimental-features 'nix-command flakes' \
  build . --option sandbox false --no-write-lock-file

# Or use the helper script to build all four + relink + pin GC roots:
./build_all.sh
```

Note: `--option sandbox false` is required because the overridden
`unpackPhase` calls `/usr/bin/cp`, which isn't visible inside nix's
sandbox.

## Pinning to prevent garbage collection

`nix-collect-garbage` deletes any store path not reachable from a GC
root. After building, pin each result to survive future GC:

```bash
mkdir -p /nix/var/nix/gcroots/custom
for mod in local_planner arise_slam terrain_analysis path_follower; do
  path=$(readlink -f dimos/robot/deeprobotics/m20/nix_wrappers/$mod/result)
  ln -sfn "$path" "/nix/var/nix/gcroots/custom/$mod-current"
done
```

`build_all.sh` handles this automatically.
