I want you to automatically fix the following issues. First, scan the current branch and identify any code which breaks the following. Write down the issues to fix into a file called `issues.ignore.md`.

The branch is $$BRANCH$$. You can perform a diff with `git diff main...$$BRANCH$$ -- . ':(exclude)*.lock' ':(exclude)uv.lock' ':(exclude)package-lock.json' ':(exclude)yarn.lock' ':(exclude)pnpm-lock.yaml' ':(exclude)Cargo.lock' ':(exclude)*.pdf' ':(exclude)*.png' ':(exclude)*.jpg'
`
The `issues.ignore.md` file will be used as the input for a second agent which will fix the issues which have been identified. It's okay if there are no issues to fix. Just leave the file empty or don't create it at all.

What follows are the issues to scan for in the branch.

$$RULES$$
