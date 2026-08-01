## ADDED Requirements

### Requirement: Audited dependency installation with general network access
The case environment SHALL permit arbitrary Python execution, `uv` dependency installation, and general outbound network access. The benchmark SHALL NOT enforce a package-index-only destination policy.

#### Scenario: Install a dependency
- **WHEN** code in a case requests a Python package through `uv`
- **THEN** the installation may use the available network and the dependency declaration, resolution, and observable command/network activity are recorded for review

### Requirement: Heuristic network-activity audit
The host SHALL preserve the Pi transcript, tool trace, and sandbox command audit for post-run review. It SHALL flag observable network-oriented commands or configuration when detectable and SHALL state in the run evidence that this audit is heuristic and cannot prove that online information or services were not used.

#### Scenario: Review observable network activity
- **WHEN** a case invokes an observable network-oriented command or configures a network client
- **THEN** the host retains the audit record and flags the observation without treating the flag, or its absence, as proof of online-use compliance

#### Scenario: Run local analysis
- **WHEN** the agent executes a Python program using staged files and workspace files
- **THEN** the program can run normally, with its transcript, tool trace, and sandbox command audit retained for post-run review
