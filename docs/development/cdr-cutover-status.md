# CDR cutover and legacy cleanup status

**WIP.** The shared factory supports generated CDR types and legacy LCM types.
The repository migration is incomplete. Keep legacy schemas, decoders and value
wrappers until their callers are migrated and verified.

## Inspect remaining boundaries

Run the static audit from this checkout. It does not import runtime modules or
contact hardware:

```sh
python scripts/audit_cdr_boundaries.py --output /tmp/cdr-boundaries.json
python scripts/audit_cdr_boundaries.py --check
```

`--check` returns nonzero while candidates remain. This snapshot has **312 legacy
In/Out declarations** and **998 legacy imports**. These are syntax matches, not
validated runtime failures. Internal driver values can be valid conversion
boundaries. Inspect complete blueprint port pairs before changing them.

## Current validation limits

The default-layer check has 37 passing tests and four failures:

- The Python `TransportRecorder` skips generated values on LCM and Zenoh. The new
  Rust recording path is separate; this does not prove Python capture cutover.
- The XArm and Piper collection graphs do not expose the expected generated
  aggregate `coordinator_joint_state` input on their recorder.

Agent, drone, Habitat and other legacy graphs still need migration. Original
work and prior heads remain in the consolidation backups. This restack does not
complete or discard those migrations.

Before deleting an API, migrate its callers and verify the selected graph,
capture/replay, source timestamps and schema identity on an installed package.
Keep the converter's explicit historical decode boundary for old recordings.
Nix hardware packaging and physical robot behavior are not verified here.
