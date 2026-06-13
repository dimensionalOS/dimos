#!/usr/bin/env bash
# Scan a contributor's branch for code-quality issues and open a PR of fixes back into that branch.
#
# Usage: misc/auto-fixes/auto-fix.sh <branch>
#
set -euo pipefail

# Print the script's own messages in green so they stand out from git/uv/claude output.
GREEN=$'\033[0;32m'
RESET=$'\033[0m'
log() { echo "${GREEN}$*${RESET}"; }      # stdout (progress)
err() { echo "${GREEN}$*${RESET}" >&2; }  # stderr (errors)

usage() {
  err "usage: $0 <branch>"
  exit 2
}

[[ $# -eq 1 ]] || usage
BRANCH="$1"
# shellcheck disable=SC2016
PLACEHOLDER='$$BRANCH$$'
# shellcheck disable=SC2016
RULES_PLACEHOLDER='$$RULES$$'

REPO_ROOT="$(git rev-parse --show-toplevel)"
SCAN_TEMPLATE="$REPO_ROOT/misc/auto-fixes/scan_template.md"
FIX_TEMPLATE="$REPO_ROOT/misc/auto-fixes/fix_template.md"
# The code-quality rules live in docs so they can be reused by other prompts; the scan prompt is
# assembled by injecting them at the template's $$RULES$$ placeholder.
RULES_FILE="$REPO_ROOT/docs/coding-agents/code-quality-rules.md"
[[ -f "$SCAN_TEMPLATE" ]] || { err "missing $SCAN_TEMPLATE"; exit 1; }
[[ -f "$FIX_TEMPLATE" ]]  || { err "missing $FIX_TEMPLATE";  exit 1; }
[[ -f "$RULES_FILE" ]]   || { err "missing $RULES_FILE";   exit 1; }

EXPECTED_NAME="$(git -C "$REPO_ROOT" config user.name)"
EXPECTED_EMAIL="$(git -C "$REPO_ROOT" config user.email)"

# This repo has several contributor remotes, so gh can't infer a default repo; pass it to gh via -R.
ORIGIN_URL="$(git -C "$REPO_ROOT" remote get-url origin)"
REPO_SLUG="${ORIGIN_URL#*github.com}"  # strip scheme/host -> ":OWNER/REPO.git" or "/OWNER/REPO.git"
REPO_SLUG="${REPO_SLUG#[:/]}"          # drop the leading : or /
REPO_SLUG="${REPO_SLUG%.git}"          # drop trailing .git -> OWNER/REPO

SUFFIX="$(openssl rand -hex 4)"
WORKTREE="$(cd "$REPO_ROOT/.." && pwd)/dimos-worktree-${SUFFIX}"
AUTOFIX_BRANCH=""  # set once we know there are issues; cleanup uses it

# Leave no state in the user's repo: remove the worktree, then (it shares the common git dir) the
# local autofix branch and any filter-branch backup ref. On success the branch lives on origin; on
# failure this is a throwaway attempt, so deleting the local ref is intended.
cleanup() {
  local code=$?
  if [[ -d "$WORKTREE" ]]; then
    git -C "$REPO_ROOT" worktree remove --force "$WORKTREE" 2>/dev/null || rm -rf "$WORKTREE"
  fi
  git -C "$REPO_ROOT" worktree prune 2>/dev/null || true
  if [[ -n "$AUTOFIX_BRANCH" ]]; then
    git -C "$REPO_ROOT" branch -D "$AUTOFIX_BRANCH" 2>/dev/null || true
    git -C "$REPO_ROOT" update-ref -d "refs/original/refs/heads/$AUTOFIX_BRANCH" 2>/dev/null || true
  fi
  exit "$code"
}
trap cleanup EXIT INT TERM

log ">> fetching origin main $BRANCH"
git -C "$REPO_ROOT" fetch origin main "$BRANCH"

log ">> creating worktree $WORKTREE (detached on origin/$BRANCH)"
git -C "$REPO_ROOT" worktree add --detach "$WORKTREE" "origin/$BRANCH"

log ">> installing dependencies"
( cd "$WORKTREE" && CYCLONEDDS_HOME=/opt/cyclonedds uv sync --all-extras --all-groups )

log ">> running scan agent"
# In the detached worktree the bare local <branch> ref may not exist, so the template's
# `git diff main...$$BRANCH$$` is pointed at origin/<branch>.
scan_prompt="$(cat "$SCAN_TEMPLATE")"
scan_prompt="${scan_prompt//"$RULES_PLACEHOLDER"/$(cat "$RULES_FILE")}"
scan_prompt="${scan_prompt//"$PLACEHOLDER"/origin/$BRANCH}"
if ! ( cd "$WORKTREE" && claude -p "$scan_prompt" --dangerously-skip-permissions ); then
  err "scan agent failed"
  exit 1
fi

ISSUES="$WORKTREE/issues.ignore.md"
if [[ ! -s "$ISSUES" ]] || ! grep -q '[^[:space:]]' "$ISSUES"; then
  log ">> no issues found for $BRANCH; nothing to do."
  exit 0
fi

# Pick a fresh autofix branch name. Every run starts clean, so instead of aborting on a collision we
# bump the suffix (-autofixes, -autofixes2, ...) to the first name that is free both locally and on
# origin (GitHub). A branch left behind by a failed prior run is simply skipped, never a blocker.
branch_exists() {
  git -C "$REPO_ROOT" show-ref --verify --quiet "refs/heads/$1" \
    || git -C "$REPO_ROOT" ls-remote --exit-code --heads origin "$1" >/dev/null 2>&1
}

AUTOFIX_BRANCH="${BRANCH}-autofixes"
suffix_n=2
while branch_exists "$AUTOFIX_BRANCH"; do
  AUTOFIX_BRANCH="${BRANCH}-autofixes${suffix_n}"
  suffix_n=$((suffix_n + 1))
done
log ">> autofix branch: $AUTOFIX_BRANCH"

BASE_SHA="$(git -C "$WORKTREE" rev-parse HEAD)"
git -C "$WORKTREE" checkout -b "$AUTOFIX_BRANCH"

log ">> running fix agent"
fix_prompt="$(cat "$FIX_TEMPLATE")"
fix_prompt="${fix_prompt//"$PLACEHOLDER"/$BRANCH}"
if ! ( cd "$WORKTREE" && claude -p "$fix_prompt" \
         --dangerously-skip-permissions \
         --settings '{"includeCoAuthoredBy": false}' ); then
  err "fix agent failed"
  exit 1
fi

n_commits="$(git -C "$WORKTREE" rev-list --count "$BASE_SHA"..HEAD)"
if [[ "$n_commits" -eq 0 ]]; then
  log ">> fix agent made no commits; nothing to PR."
  exit 0
fi
log ">> fix agent made $n_commits commit(s)"

# Safety net: --settings includeCoAuthoredBy=false is the primary guard, but --print silently ignores
# an invalid settings string, so mechanically strip any attribution lines from the new commits and
# verify nothing slipped through.
log ">> stripping any agent attribution from commit messages"
FILTER_BRANCH_SQUELCH_WARNING=1 git -C "$WORKTREE" filter-branch -f --msg-filter \
  'grep -viE "^(Co-authored-by:|Generated with \[Claude Code\])|🤖"' \
  -- "$BASE_SHA"..HEAD

if git -C "$WORKTREE" log --format='%B' "$BASE_SHA"..HEAD \
   | grep -qiE 'co-authored-by:|generated with \[claude code\]|🤖'; then
  err "agent attribution survived strip; aborting."
  exit 1
fi

bad=0
while IFS='|' read -r sha aname aemail; do
  if [[ "$aname" != "$EXPECTED_NAME" || "$aemail" != "$EXPECTED_EMAIL" ]]; then
    err "non-default author on $sha: $aname <$aemail> (expected $EXPECTED_NAME <$EXPECTED_EMAIL>)"
    bad=1
  fi
done < <(git -C "$WORKTREE" log --format='%H|%an|%ae' "$BASE_SHA"..HEAD)
[[ "$bad" -eq 0 ]] || { err "refusing to push: non-default author detected."; exit 1; }

log ">> pushing $AUTOFIX_BRANCH"
git -C "$WORKTREE" push -u origin "$AUTOFIX_BRANCH"

log ">> opening PR into $BRANCH"
# Backticks are literal markdown for the PR body, not command substitution.
# shellcheck disable=SC2016
pr_body='These are automated fixes. Each fix is a separate commit. Use `git rebase -i` to drop any you disagree with.'
if ! ( cd "$WORKTREE" && gh pr create \
    -R "$REPO_SLUG" \
    --base "$BRANCH" \
    --head "$AUTOFIX_BRANCH" \
    --title "Auto-fixes for $BRANCH" \
    --body "$pr_body" ); then
  err "PR creation failed, but $AUTOFIX_BRANCH is already pushed to origin. Open it manually with:"
  err "  gh pr create -R $REPO_SLUG --base $BRANCH --head $AUTOFIX_BRANCH --title \"Auto-fixes for $BRANCH\""
  exit 1
fi

log ">> done: opened PR $AUTOFIX_BRANCH -> $BRANCH"
