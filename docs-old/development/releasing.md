# Releasing

How to cut a dimos release.

Throughout this document, replace `X.Y.Z` with the version you are releasing (e.g. `0.0.13`).

## 1. Preparing for a release

1. Check for an existing `release/*` branch on the remote (`git ls-remote --heads origin 'release/*'`, or the Branches page). If one is still around from a previous release, complete section 3 for that branch before continuing.
2. Bump the version on `main`. `uv version --bump patch` (or `minor` / `major`). Open a PR, squash-merge.
3. Create the temporary release branch from the version-bump commit (need CI to complete on main before push will succeed):

   ```bash
   git fetch origin
   git checkout -b release/X.Y.Z origin/main
   git push -u origin release/X.Y.Z
   ```

4. Create a backport label for this release. Repo → Issues → Labels (in left sidebar) → New label, named `backport release/X.Y.Z`. (Or `gh label create "backport release/X.Y.Z" --repo dimensionalOS/dimos`.) The backport bot only runs when this label exists.
5. To backport a fix from `main`: add the `backport release/X.Y.Z` label to a PR targeting `main` (before or after merging). The backport bot will open a cherry-pick PR onto the release branch; review it and squash-merge.

## 2. Creating the release

1. Run the full test suite locally on the release branch.

   ```bash
   uv run pytest -m '' --error-for-skips
   ```

2. [Run](https://docs.github.com/en/actions/how-tos/manage-workflow-runs/manually-run-a-workflow#running-a-workflow) the `release` workflow on the `release/X.Y.Z` branch.
3. Monitor the CI run. When it reaches the publish-pypi step, you'll need other team members to approve the release.
4. After completion, the bot will have pushed a signed merge-back commit directly to `main`. Confirm with `git log --first-parent main -1` — the tip should be `Merge release/X.Y.Z back to main`. Then verify `vX.Y.Z` shows on https://github.com/dimensionalOS/dimos/releases and on https://pypi.org/project/dimos/.

## 3. Cleanup

At this point, you can leave the branch around for potential patch releases, or cleanup immediately (recreating the branch from the tag if needed).

When ready to cleanup the branch, follow these steps:

1. Rename the release branch out of the protected `release/*` namespace, then delete it. The `release/*` ruleset blocks direct deletion; renaming takes the branch out of scope.

   Web UI: Repo → Branches → find `release/X.Y.Z` → ⋯ → **Rename branch** → e.g. `archived/X.Y.Z`. Then delete it from the same Branches page.

   Or via `gh`:

   ```bash
   gh api -X POST "repos/dimensionalOS/dimos/branches/release/X.Y.Z/rename" -f new_name=archived/X.Y.Z
   git push origin --delete archived/X.Y.Z
   ```

2. Delete the `backport release/X.Y.Z` label from the Labels page (or `gh label delete "backport release/X.Y.Z"`).

## Patch fix on a released version

When you need to ship a patch fix:

1. If the release branch has already been deleted, re-cut from the tag and recreate the label (step 1.4):

   ```bash
   git fetch origin --tags
   git checkout -b release/X.Y.Z vX.Y.Z
   git push -u origin release/X.Y.Z
   ```

2. Apply a patch version bump: `uv version --bump patch`
3. Apply any fixes and follow sections 2 and 3 as before.

### Verifying a release tag

```bash
gpg --import docs-old/release-signing-key.asc
git fetch --tags
git tag -v vX.Y.Z
```

`git tag -v` exits 0 and reports a good signature from `dimos-release-bot` when the tag is valid. GitHub's web UI shows "Unverified" because the GPG key isn't bound to a GitHub user account — this is expected.

### Rotating the App private key

Rotate immediately on suspected compromise.

1. App settings → **Private keys → Generate a private key** → download the new `.pem`.
2. Update the `RELEASE_BOT_APP_PRIVATE_KEY` secret in the `release-tag` environment with the new key contents.
3. App settings → **Private keys** → delete the old key.

### Rotating the GPG signing key

Rotate immediately on suspected compromise.
Sam/Stash have revocation certificates which can be published to revoke the old key.

Regenerate the key in memory on Linux (private material never touches persistent storage — `/dev/shm` is always `tmpfs`):

```bash
export GNUPGHOME=/dev/shm/gnupg-release-bot
mkdir -p "$GNUPGHOME" && chmod 700 "$GNUPGHOME"

gpg --batch --pinentry-mode loopback --passphrase '' \
    --quick-generate-key 'dimos-release-bot <build@dimensionalOS.com>' ed25519 sign 2y

KEY_ID=$(gpg --list-secret-keys --keyid-format=long --with-colons dimos-release-bot \
         | awk -F: '/^sec/{print $5; exit}')
echo "RELEASE_BOT_GPG_KEY_ID: $KEY_ID"

# Wayland uses `wl-copy`; on X11 replace with `xclip -i -selection clipboard`
# (and `wl-copy --clear` with `echo -n | xclip -i -selection clipboard`).
gpg --armor --export-secret-keys "$KEY_ID" | wl-copy   # paste into RELEASE_BOT_GPG_PRIVATE_KEY secret
wl-copy --clear
gpg --armor --export "$KEY_ID" | wl-copy               # paste into docs-old/release-signing-key.asc
wl-copy --clear

rm -rf "$GNUPGHOME"
unset GNUPGHOME
```

Then update the `RELEASE_BOT_GPG_PRIVATE_KEY` secret in the `release-tag` environment, repo variable `RELEASE_BOT_GPG_KEY_ID`, and commit the replacement `docs-old/release-signing-key.asc`.
