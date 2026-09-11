# Private browser access

Target URL: https://omarchy.tailca0707.ts.net:8443

The gateway is running with a trusted Tailscale HTTPS certificate and was
verified from the Mac over the tailnet. It binds only the configured tailnet IPv4 address. The
public internet and sim.tule.world are not enabled by this configuration.

## First certificate

From a terminal with access to sudo on Omarchy:

```bash
cd ~/projects/microduck-world
sudo tailscale cert --cert-file=config/tls/omarchy.crt --key-file=config/tls/omarchy.key omarchy.tailca0707.ts.net
sudo chown tule:tule config/tls/omarchy.crt config/tls/omarchy.key
chmod 600 config/tls/omarchy.key
./gateway-service start
```

Tailscale HTTPS must be enabled for the tailnet. If the cert command reports that
it is disabled, enable HTTPS in the tailnet's DNS settings and retry. Certificates
expire; repeat issuance before expiry, then stop/start the gateway to load the new
files. Automatic renewal is not configured; service startup at boot is enabled.

## Operations

```bash
./gateway-service status
./gateway-service logs
./gateway-service stop
./gateway-service start
```

The gateway and world are supervised user services, enabled for boot.
Linger is enabled for tule, so they do not require an interactive login.
Service source, configuration, keys, and logs live here; systemd maintains account
registration symlinks. Use ./service to manage both, or ./gateway-service for
gateway-only compatibility. Process crash recovery has been tested; the whole PC
has not been rebooted for validation.

GET /healthz returns ready only while a robot is registered with the relay.
The project frontend remains available during relay restarts and retries its connection.

## Connection design

Browser HTTPS -> project gateway TCP 8443 -> project frontend/assets or loopback HTTP relay.
The private config/tailnet.json sets frontend_dir to this project's web/dist and
world_assets_dir to state/viewer. The gateway serves / and /client/* from the built
frontend and /world-assets/scene-<hash>.json from generated visual assets. Existing
Host/Origin/cross-site checks apply to all these routes. Model responses are immutable
and gzip-compressed; configuration, TLS keys and robot policy files are not served.
Browser QUIC -> project gateway UDP 8443 -> existing loopback QUIC relay.

/api/info retains the relay's ephemeral certificate hash and rewrites only its
advertised endpoint. The browser receives that pin over trusted HTTPS and verifies
the original relay certificate end-to-end. No certificate checks are disabled,
and the DimOS relay and SDK source remain unchanged. A relay restart changes the
upstream port; the next bootstrap refresh updates forwarding and retires old
connections. UDP peers are capped at 64 and idle mappings expire after 60 seconds.

Tailnet membership/ACLs are the current access boundary. Wrong Host, foreign Origin
and cross-site browser requests are rejected; wildcard CORS is stripped. This is
not the future public visitor/session authorization layer. The UDP endpoint can
carry the relay's protocol, so only trusted tailnet members should have access.

## Validation so far

- 12 focused tests cover discovery/pin preservation, wrong origins and hosts,
  private interface validation, read-only HTTP, separate UDP clients, connection
  limits and upstream restart cleanup.
- Strict mypy passed on the two gateway source files; Ruff passed.
- The actual running relay delivered a 10,882-byte JPEG through the UDP forwarder.
- Trusted HTTPS verified from the Mac without certificate exceptions.
- Chromium on the Mac received live video at roughly 18 fps, reconnected after
  reload, and moved the simulated duck with keyboard control. Key release
  returned commanded velocity to zero.
- Certificate expires 2026-12-04; renewal remains a manual operation.

## Reference

User lingering starts the user service manager at boot and keeps it after logout:
[systemd loginctl documentation](https://www.freedesktop.org/software/systemd/man/252/loginctl.html).
