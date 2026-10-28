# Legacy message removal audit

Audited stack tip: robots PR #4738, commit `73e71cf8d619f02e63923e11f3fd29b639a1f0f6`. Counts below are files containing the literal `dimos.msgs` outside that directory, including tests and documentation. They describe that pinned remote snapshot, before the control follow-up in this branch; they are not counts of production imports.

The final migration removes `dimos/msgs`. Converting transport ports while retaining legacy numerical messages is intermediate work, not completion.

| Area | Files with references |
| --- | ---: |
| robot | 95 |
| manipulation | 82 |
| control | 56 |
| mapping | 35 |
| navigation | 34 |
| memory | 29 |
| experimental | 27 |
| docs | 25 |
| evals | 24 |
| protocol | 24 |
| web | 23 |
| agents | 19 |
| teleop | 14 |
| simulation | 13 |
| hardware | 12 |
| core | 11 |
| e2e_tests | 11 |
| imitation | 9 |
| models | 8 |
| cli | 7 |
| examples | 7 |
| utils | 5 |
| visualization | 5 |
| native | 4 |
| misc | 3 |
| cloud | 2 |
| perception | 2 |
| porcelain | 2 |
| types | 2 |
| .github | 1 |
| AGENTS.md | 1 |
| README.md | 1 |
| hosted | 1 |
| spec | 1 |
| packages | 1 |
| scripts | 1 |

## Required work before the removal PR

- Migrate remaining runtime consumers in robot drivers, manipulation, navigation, mapping, memory, protocol, agents, hardware, and web. Use generated messages plus utilities; remove adapters that reconstruct old message objects.
- Migrate tests, fixtures, benchmarks, examples, CI snippets, and user-facing documentation. Audit dynamic import strings and serialized type identifiers as well as Python imports.
- Resolve shared configuration values such as `RobotModelConfig.base_pose`, which still constructs a legacy pose outside the control directory. Control's mathematical use of that configuration remains a cross-layer dependency until the planning layer migrates.
- Verify historical recording readers against the actual Go2 dataset and custom MCAP recordings after removal. A text scan cannot establish persisted-data compatibility.
- Regenerate package artifacts from committed message definitions and inspect wheel contents. Remove obsolete packaging, stub, and native/example references together with the directory.
- Run an import guard with legacy imports disabled, then control/IK, navigation/mapping replay, browser command transport, third-party message, and recording-reader acceptance. Preserve final outputs, timestamp precision, frame conventions, and ownership behavior.

## Control follow-up

This branch removes direct legacy references and wire-format aliases from `dimos/control`, uses generated messages throughout its contracts and task state, and moves trajectory sampling to message utilities. WebXR retains generated poses and controller messages internally. The newly generated `dimos_msgs/ControlValues` preserves the existing contract fields; finite-value rejection is enforced at validation boundaries rather than in a legacy constructor.

The other areas listed above are audited follow-up work. This branch does not delete the legacy directory or claim the whole repository is ready for deletion.
