# Ubuntu

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
