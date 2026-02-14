# Writing Docs

1. Where to put your docs:
    - If it only matters to people who contribute to dimos (like this doc), put them in `docs/development`
    - Otherwise put them in `docs/usage`
2. Run `bin/gen_diagrams` to generate the svg's for your diagrams. We use [pikchr](https://pikchr.org/home/doc/trunk/doc/userman.md) as a diagram language.
3. Use [md-babel-py](https://github.com/leshy/md-babel-py/) (`md-babel-py run thing.md`) to make sure your code examples work.
