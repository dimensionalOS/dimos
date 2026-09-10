# Apple Silicon macOS

For SDK applications and contributor checkouts, follow the
[dimup installation guide](/docs/installation/installer.md).

After its machine bootstrap, prepare a contributor checkout with:

```sh skip
dimup dev dimos --ref feat/dimup-release
cd dimos
source .dimos/activate.sh
dimos doctor
git switch -c feat/my-change
```

Keep the branch override while testing this PR. After merge, omit `--ref` to
start from `main`. The command installs the editable SDK, runtime dependencies,
test/lint tools, and commit hooks. Native modules build on demand.

## Transport note for macOS

LCM over UDP can be unreliable on macOS for large or high-rate replay workloads. dimOS defaults the global stream transport to **Zenoh** everywhere, so you never need `--transport=zenoh`. Use `--transport=lcm` if you need to force the legacy multicast path.

See the [Zenoh quickstart](/docs/usage/transports/index.md#zenoh-quickstart) for what the localhost-pinned default reaches and how to point it at a robot or the LAN.

```sh skip
dimos --dtop --replay --replay-db=go2_bigoffice run unitree-go2
```
