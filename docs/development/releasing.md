# Releasing

How to cut a dimos release.

Throughout this document, replace `X.Y.Z` with the version you are releasing (e.g. `0.0.13`).

## 1. Preparing for a release

1. Check for an existing `release/*` branch on the remote (`git ls-remote --heads origin 'release/*'`, or the Branches page). If one is still around from a previous release, complete section 3 for that branch before continuing.
2. Bump the version on `main`. `uv version --bump patch` (or `minor` / `major`). Open a PR, squash-merge.
3. Create the temporary release branch from the version-bump commit:

   ```bash
   git fetch origin
   git checkout -b release/X.Y.Z origin/main
   git push -u origin release/X.Y.Z
   ```

4. Create a backport label for this release. Repo → Issues → Labels → New label, named `backport release/X.Y.Z`. (Or `gh label create "backport release/X.Y.Z" --repo dimensionalOS/dimos`.) The backport bot only runs when this label exists.
5. To backport a fix from `main`: add the `backport release/X.Y.Z` label to a PR targeting `main` (before or after merging). The backport bot will open a cherry-pick PR onto the release branch; review it and squash-merge.

## 2. Creating the release

1. Run the full test suite locally on the release branch.

   ```bash
   uv run pytest -m 'not tool' --error-for-skips
   ```

2. [Run](https://docs.github.com/en/actions/how-tos/manage-workflow-runs/manually-run-a-workflow#running-a-workflow) the `release` workflow on the `release/X.Y.Z` branch.
3. Monitor the CI run. When it reaches the publish-pypi step, you'll need other team members to approve the release.
4. After completion, a merge-back PR will have been created. Find the PR titled `Merge release/X.Y.Z back to main` and merge-commit.

    > **Warning** — pick **"Create a merge commit"** from the merge-button dropdown. NOT "Squash and merge", NOT "Rebase and merge". Squashing collapses the two-parent topology and the tag stops being reachable from main.

5. Confirm `vX.Y.Z` shows on https://github.com/dimensionalOS/dimos/releases and on https://pypi.org/project/dimos/.

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

2. Apply any fixes and follow sections 2 and 3 as before.
