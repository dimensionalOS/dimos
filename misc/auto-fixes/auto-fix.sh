#!/usr/bin/env bash
# Scan a contributor's branch for code-quality issues and open a PR of fixes back into that branch.
#
# Usage: misc/auto-fixes/auto-fix.sh [--agent claude|opencode] <branch>
#
set -euo pipefail

# Print the script's own messages in green so they stand out from git/uv/agent output.
GREEN=$'\033[0;32m'
RESET=$'\033[0m'
log() { echo "${GREEN}$*${RESET}"; }      # stdout (progress)
err() { echo "${GREEN}$*${RESET}" >&2; }  # stderr (errors)

usage() {
  err "usage: $0 [--agent claude|opencode] <branch>"
  err "       AUTO_FIX_AGENT=opencode $0 <branch>"
  exit 2
}

AGENT="${AUTO_FIX_AGENT:-claude}"
BRANCH=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --agent|-a)
      [[ $# -ge 2 ]] || usage
      AGENT="$2"
      shift 2
      ;;
    --agent=*)
      AGENT="${1#*=}"
      shift
      ;;
    -h|--help)
      usage
      ;;
    --)
      shift
      break
      ;;
    -*)
      err "unknown option: $1"
      usage
      ;;
    *)
      [[ -z "$BRANCH" ]] || usage
      BRANCH="$1"
      shift
      ;;
  esac
done

if [[ $# -gt 0 ]]; then
  [[ -z "$BRANCH" && $# -eq 1 ]] || usage
  BRANCH="$1"
fi

[[ -n "$BRANCH" ]] || usage

case "$AGENT" in
  claude|opencode)
    ;;
  *)
    err "unsupported agent: $AGENT (expected claude or opencode)"
    exit 2
    ;;
esac

command -v "$AGENT" >/dev/null 2>&1 || { err "missing $AGENT on PATH"; exit 1; }

# shellcheck disable=SC2016
PLACEHOLDER='$$BRANCH$$'
# shellcheck disable=SC2016
AUTOFIX_PLACEHOLDER='$$AUTOFIX_BRANCH$$'
# shellcheck disable=SC2016
RULES_PLACEHOLDER='$$RULES$$'

REPO_ROOT="$(git rev-parse --show-toplevel)"
SCAN_TEMPLATE="$REPO_ROOT/misc/auto-fixes/scan_template.md"
FIX_TEMPLATE="$REPO_ROOT/misc/auto-fixes/fix_template.md"
# The code-quality rules live in docs so they can be reused by other prompts; the scan prompt is
# assembled by injecting them at the template's $$RULES$$ placeholder.
RULES_FILE="$REPO_ROOT/docs-old/coding-agents/code-quality-rules.md"
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

run_scan_agent() {
  local prompt="$1"
  case "$AGENT" in
    claude)
      ( cd "$WORKTREE" && claude -p "$prompt" --dangerously-skip-permissions )
      ;;
    opencode)
      opencode run --auto --dir "$WORKTREE" "$prompt"
      ;;
  esac
}

run_fix_agent() {
  local prompt="$1"
  case "$AGENT" in
    claude)
      ( cd "$WORKTREE" && claude -p "$prompt" \
        --dangerously-skip-permissions \
        --settings '{"includeCoAuthoredBy": false}' )
      ;;
    opencode)
      opencode run --auto --dir "$WORKTREE" "$prompt"
      ;;
  esac
}

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
( cd "$WORKTREE" && uv sync --extra all --all-groups )

log ">> running scan agent ($AGENT)"
# In the detached worktree the bare local <branch> ref may not exist, so the template's
# `git diff origin/main...$$BRANCH$$` is pointed at origin/<branch>.
scan_prompt="$(cat "$SCAN_TEMPLATE")"
scan_prompt="${scan_prompt//"$RULES_PLACEHOLDER"/$(cat "$RULES_FILE")}"
scan_prompt="${scan_prompt//"$PLACEHOLDER"/origin/$BRANCH}"
if ! run_scan_agent "$scan_prompt"; then
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

log ">> running fix agent ($AGENT)"
fix_prompt="$(cat "$FIX_TEMPLATE")"
fix_prompt="${fix_prompt//"$PLACEHOLDER"/$BRANCH}"
fix_prompt="${fix_prompt//"$AUTOFIX_PLACEHOLDER"/$AUTOFIX_BRANCH}"
if ! run_fix_agent "$fix_prompt"; then
  err "fix agent failed"
  exit 1
fi

# The fix agent should only ADD commits, but an LLM can leave the tree dirty: a half-finished
# rebase/merge/cherry-pick, a stash-pop conflict, or plain uncommitted edits. Only its commits are
# wanted, so abort any in-progress operation and discard everything uncommitted. This also keeps the
# tree clean for filter-branch below (which refuses to run on a dirty tree) and for the push.
git -C "$WORKTREE" rebase --abort      2>/dev/null || true
git -C "$WORKTREE" merge --abort       2>/dev/null || true
git -C "$WORKTREE" cherry-pick --abort 2>/dev/null || true
git -C "$WORKTREE" reset --hard        2>/dev/null || true
git -C "$WORKTREE" clean -fd           2>/dev/null || true

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
