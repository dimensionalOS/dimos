## 1. Generalize the Locked Image and Registry

- [x] 1.1 Move the outer image recipe and launcher into the shared evaluation framework.
- [x] 1.2 Remove external Evaluation entry-point discovery, tests, and documentation.

## 2. Make Public Evaluation Container-Only

- [x] 2.1 Route `dimos eval run` through the generic launcher and keep a hidden inside-container command for the existing runner.
- [x] 2.2 Add read-only input, private output staging, cache, IPC, GPU/CDI, rootless socket, environment forwarding, and atomic publication behavior.
- [x] 2.3 Require built-in task manifests to be relative to their run specification.

## 3. Reuse the Launcher for Autoresearch

- [x] 3.1 Bootstrap the existing LIBERO autoresearch module through the generic container without changing its benchmark command.
- [x] 3.2 Remove the LIBERO-specific outer runner and update evaluation documentation.

## 4. Verify the Replacement

- [x] 4.1 Add focused registry, launcher, CLI, publication, secret-redaction, and autoresearch bootstrap tests.
- [x] 4.2 Run formatting, lint, typing, focused unit suites, and the generic outer-container preflight.
- [ ] 4.3 Run one checked-in case for each built-in and the four-case development panel after credential rotation.
