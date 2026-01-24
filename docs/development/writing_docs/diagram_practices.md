
# Code Blocks

**All code blocks must be executable.**
Never write illustrative/pseudocode blocks.
If you're showing an API usage pattern, create a minimal working example that actually runs. This ensures documentation stays correct as the codebase evolves.

After writing a code block in your markdown file, you can run it by executing:
```bash
md-babel-py run document.md
```

More information on this tool is in [codeblocks](/docs/agents/docs/codeblocks.md).


# Code or Docs Links

After adding a link to a doc, run

```bash
doclinks document.md
```

### Code file references
```markdown
See [`service/spec.py`](/dimos/protocol/service/spec.py) for the implementation.
```

After running doclinks, it becomes:
```markdown
See [`service/spec.py`](/dimos/protocol/service/spec.py) for the implementation.
```

### Symbol auto-linking
Mention a symbol on the same line to auto-link to its line number:
```markdown
The `Configurable` class is defined in [`service/spec.py`](/dimos/protocol/service/spec.py#L22).
```

Becomes:
```markdown
The `Configurable` class is defined in [`service/spec.py`](/dimos/protocol/service/spec.py#L22).
```
### Doc-to-doc references
Use `.md` as the link target:
```markdown
See [Configuration](/docs/api/configuration.md) for more details.
```

Becomes:
```markdown
See [Configuration](/docs/concepts/configuration.md) for more details.
```

More information on this is in [doclinks](/docs/agents/docs/doclinks.md).


# Pikchr

[Pikchr](https://pikchr.org/) is a diagram language from SQLite. Use it for flowcharts and architecture diagrams.

**Important:** Always wrap pikchr blocks in `<details>` tags so the source is collapsed by default on GitHub. The rendered SVG stays visible outside the fold. Code blocks (Python, etc.) should NOT be folded—they're meant to be read.

## Basic syntax

<details>
<summary>diagram source</summary>

```pikchr fold output=assets/pikchr_basic.svg
color = white
fill = none

A: box "Step 1" rad 5px fit wid 170% ht 170%
arrow right 0.3in
B: box "Step 2" rad 5px fit wid 170% ht 170%
arrow right 0.3in
C: box "Step 3" rad 5px fit wid 170% ht 170%
```

</details>

<!--Result:-->
![output](assets/pikchr_basic.svg)

## Box sizing

Use `fit` with percentage scaling to auto-size boxes with padding:

<details>
<summary>diagram source</summary>

```pikchr fold output=assets/pikchr_sizing.svg
color = white
fill = none

# fit wid 170% ht 170% = auto-size + padding
A: box "short" rad 5px fit wid 170% ht 170%
arrow right 0.3in
B: box ".subscribe()" rad 5px fit wid 170% ht 170%
arrow right 0.3in
C: box "two lines" "of text" rad 5px fit wid 170% ht 170%
```

</details>

<!--Result:-->
![output](assets/pikchr_sizing.svg)

The pattern `fit wid 170% ht 170%` means: auto-size to text, then scale width by 170% and height by 170%.

For explicit sizing (when you need consistent box sizes):

<details>
<summary>diagram source</summary>

```pikchr fold output=assets/pikchr_explicit.svg
color = white
fill = none

A: box "Step 1" rad 5px fit wid 170% ht 170%
arrow right 0.3in
B: box "Step 2" rad 5px fit wid 170% ht 170%
```

</details>

<!--Result:-->
![output](assets/pikchr_explicit.svg)

## Common settings

Always start with:

```
color = white    # text color
fill = none      # transparent box fill
```

## Branching paths

<details>
<summary>diagram source</summary>

```pikchr fold output=assets/pikchr_branch.svg
color = white
fill = none

A: box "Input" rad 5px fit wid 170% ht 170%
arrow
B: box "Process" rad 5px fit wid 170% ht 170%

# Branch up
arrow from B.e right 0.3in then up 0.35in then right 0.3in
C: box "Path A" rad 5px fit wid 170% ht 170%

# Branch down
arrow from B.e right 0.3in then down 0.35in then right 0.3in
D: box "Path B" rad 5px fit wid 170% ht 170%
```

</details>

<!--Result:-->
![output](assets/pikchr_branch.svg)

**Tip:** For tree/hierarchy diagrams, prefer left-to-right layout (root on left, children branching right). This reads more naturally and avoids awkward vertical stacking.

## Adding labels

<details>
<summary>diagram source</summary>

```pikchr fold output=assets/pikchr_labels.svg
color = white
fill = none

A: box "Box" rad 5px fit wid 170% ht 170%
text "label below" at (A.x, A.y - 0.4in)
```

</details>

<!--Result:-->
![output](assets/pikchr_labels.svg)

## Reference

| Element | Syntax |
|---------|--------|
| Box | `box "text" rad 5px wid Xin ht Yin` |
| Arrow | `arrow right 0.3in` |
| Oval | `oval "text" wid Xin ht Yin` |
| Text | `text "label" at (X, Y)` |
| Named point | `A: box ...` then reference `A.e`, `A.n`, `A.x`, `A.y` |

See [pikchr.org/home/doc/trunk/doc/userman.md](https://pikchr.org/home/doc/trunk/doc/userman.md) for full documentation.
