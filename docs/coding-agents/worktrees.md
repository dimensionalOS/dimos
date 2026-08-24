# Worktrees

`bin/worktree` creates a fully-provisioned git worktree (uv venv + `uv sync
--all-groups`, `.envrc` symlink, direnv allow) so `mypy`/`pytest`/hooks work
immediately — handy for forking off parallel work.

```sh skip
bin/worktree new feat/ivan/foo              # new branch off current HEAD -> ../dimos-foo
bin/worktree new feat/ivan/foo origin/main  # fork from main instead
bin/worktree new existing-branch            # check out a branch that already exists

bin/worktree ls                             # list worktrees (branch column = rm target)

bin/worktree rm feat/ivan/foo               # remove the worktree (keep the branch)
bin/worktree rm feat/ivan/foo -b            # remove worktree AND delete the branch
```

Run commands in a worktree from outside it (direnv isn't auto-loaded in a
non-interactive shell):

```sh skip
direnv exec ../dimos-foo python -m pytest dimos
direnv exec ../dimos-foo python -m mypy dimos
```
