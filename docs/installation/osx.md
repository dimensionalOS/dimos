# macOS installation

Use the official installer on macOS 12.6 or newer:

```sh skip
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
```

The installer sets up Homebrew dependencies, uv, Python 3.12, and dimOS. Choose **library** for the published package or **dev** for a source checkout. Follow the printed activation command when it finishes.

Apple Silicon is the target macOS configuration. macOS CI is paused because runner capacity is exhausted; the current installer needs local validation. Package and hardware support can differ from Linux.

See [installer options and local testing](/docs/installation/index.md).

## Transport note for macOS

LCM over UDP can be unreliable on macOS for large or high-rate replay workloads. dimOS defaults the global stream transport to **Zenoh** everywhere, so you never need `--transport=zenoh`. Use `--transport=lcm` if you need to force the legacy multicast path.

See the [Zenoh quickstart](/docs/usage/transports/index.md#zenoh-quickstart) for what the localhost-pinned default reaches and how to point it at a robot or the LAN.
