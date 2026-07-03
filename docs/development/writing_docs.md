---
title: "Writing DimOS docs, diagrams, and examples"
description: "Where to place DimOS docs, how to generate mermaid and pikchr diagrams with bin/gen-diagrams, and how to keep code examples runnable via md-babel-py."
---

1. Where to put your docs:
    - If it only matters to people who contribute to dimos (like this doc), put them in `docs/development`
    - Otherwise put them in `docs/usage`
2. Run `bin/gen-diagrams` to generate the svg's for your diagrams. We use [mermaid](https://mermaid.js.org/intro/) (no generation needed) and [pikchr](https://pikchr.org/home/doc/trunk/doc/userman.md) as diagrams languages.
3. Use [md-babel-py](https://github.com/leshy/md-babel-py/) (`md-babel-py run thing.md`) to make sure your code examples work.
