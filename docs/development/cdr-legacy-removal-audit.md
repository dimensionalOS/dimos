# Retire legacy message wrappers

Each capability refactor replaces its production code, internal state, configuration, tests, examples, and native entry points with generated messages and shared utilities. Converting its transport ports alone does not complete the refactor. Code must not reconstruct, alias, or reexport the retired wrappers.

The stack places schema definitions and generator fixes before their consumers. Shared helpers appear before the first transport or recording consumer. Viewer rendering belongs to the viewer layer. Mapping, control, robot, and conversion changes remain in their corresponding capability PRs.

The final cleanup PR deletes `dimos/msgs` after those caller migrations. It contains the import guard and removal documentation, rather than another batch of capability conversions.

## Check a capability

Run the static check without importing robot drivers:

```bash
python scripts/audit_cdr_boundaries.py --check --scope dimos/control --scope dimos/manipulation --scope dimos/teleop
```

Run the repository check from the completed stack:

```bash
python scripts/audit_cdr_boundaries.py --check
```

The default check covers capabilities, tests, examples, native Python runtimes, miscellaneous tools, and scripts. It rejects imports of retired wrappers, including `dimos.msgs.helpers`. It also rejects raw LCM message imports outside the listed historical readers, protocol fixtures, and external Habitat ABI.

A clean static check does not prove numerical parity, native compilation, or user interface behavior. Each capability also requires its behavior tests and the applicable transport or recording acceptance checks.

## Historical data and external protocols

Historical LCM recordings decode into generated values. Historical pickle readers map the retired class metadata to neutral state and then construct generated values. Neither reader imports `dimos.msgs`.

The Habitat Python 3.9 server retains its external raw LCM protocol. Its DimOS connection translates that protocol to generated public messages. Historical browser LCM fixtures remain explicit protocol fixtures. These exceptions do not permit old DimOS wrapper types on live typed ports.

Raw CDR recordings retain their declared schema. Native MCAP recording accepts independently generated external message packages, without adding their types to a built-in Rust dispatch table.

## Validation evidence

The broad capability run passed 3,023 tests. A subsequent 106-test recording and WebXR run passed, including the browser command encoder with the existing pinned Deno and npm cache. The default broad run also reported 11 optional-dependency skips. These are separate overlapping suites, not an additive test total.

Deleting the old directory passed the import guard and the actual Rerun example. The guard includes a negative example that reproduces the previously missed helper import in runnable examples and native Python runtimes.

An explicit UI-less `build_py` artifact contains generated messages and schemas and no retired package. A normal build still requires the cockpit and SDK web distributions. The UI-less artifact does not establish a complete web wheel build.

## Validation limits

The native Livox and PointLIO modules pass their locked offline library tests and all-target checks. dimSLAM passes its locked offline tests and Clippy with the upstream cuVSLAM stub. These checks do not establish hardware or GPU operation.

Nix is unavailable in the validation environment, so changed flakes and local source input locks still require Nix evaluation and lock refresh. Full Go2 DDS compilation stops in the CycloneDDS bindings before the migrated producer compiles. The M20 C++ bridge needs the vendor DrDDS, DrIDL, and Zenoh dependencies, together with the generated C++ and FastCDR prefix. Their complete builds remain outstanding.

Robot hardware, live ROS, and GPU-dependent features were not exercised. Missing optional SDKs remain explicit test gaps. No performance improvement is claimed by this completion work.
