# Need-to-know Things

1. How we resolve file references (linking)
2. How we make of our code blocks executable and test them.
3. How we make diagrams


<br>
<br>

# 1. Use Doclinks to Resolve file references

## Syntax

<!-- doclinks-ignore-start -->
| Pattern     | Example                                             |
|-------------|-----------------------------------------------------|
| Code file   | `[`service/spec.py`]()` → resolves path             |
| With symbol | `Configurable` in `[`spec.py`]()` → adds `#L<line>` |
| Doc link    | `[Configuration](.md)` → resolves to doc            |
<!-- doclinks-ignore-end -->

## Usage

```bash
doclinks docs/guide.md   # single file
doclinks docs/           # directory
doclinks --dry-run ...   # preview only
```

## Full Documentation

<details>
<summary>Click to see full documentation</summary>

## What it does

When writing docs, you can use placeholder links like:

<!-- doclinks-ignore-start -->
```markdown
See [`service/spec.py`]() for the implementation.
```
<!-- doclinks-ignore-end -->

Running `doclinks` resolves these to actual paths:

<!-- doclinks-ignore-start -->
```markdown
See [`service/spec.py`](/dimos/protocol/service/spec.py) for the implementation.
```
<!-- doclinks-ignore-end -->

## Features

<!-- doclinks-ignore-start -->
- **Code file links**: `[`filename.py`]()` resolves to the file's path
- **Symbol line linking**: If another backticked term appears on the same line, it finds that symbol in the file and adds `#L<line>`:
  ```markdown
  See `Configurable` in [`config.py`]()
  → [`config.py`](/path/config.py#L42)
  ```
- **Doc-to-doc links**: `[Modules](.md)` resolves to `modules.md` or `modules/index.md`
<!-- doclinks-ignore-end -->
- **Multiple link modes**: absolute, relative, or GitHub URLs
- **Watch mode**: Automatically re-process on file changes
- **Ignore regions**: Skip sections with `<!-- doclinks-ignore-start/end -->` comments

## Usage

```bash
# Process a single file
doclinks docs/guide.md

# Process a directory recursively
doclinks docs/

# Relative links (from doc location)
doclinks --link-mode relative docs/

# GitHub links
doclinks --link-mode github \
  --github-url https://github.com/org/repo docs/

# Dry run (preview changes)
doclinks --dry-run docs/

# CI check (exit 1 if changes needed)
doclinks --check docs/

# Watch mode (auto-update on changes)
doclinks --watch docs/
```

## Options

| Option             | Description                                     |
|--------------------|-------------------------------------------------|
| `--root PATH`      | Repository root (default: auto-detect git root) |
| `--link-mode MODE` | `absolute` (default), `relative`, or `github`   |
| `--github-url URL` | Base GitHub URL (required for github mode)      |
| `--github-ref REF` | Branch/ref for GitHub links (default: `main`)   |
| `--dry-run`        | Show changes without modifying files            |
| `--check`          | Exit with error if changes needed (for CI)      |
| `--watch`          | Watch for changes and re-process                |

## Link patterns

<!-- doclinks-ignore-start -->
| Pattern              | Description                                    |
|----------------------|------------------------------------------------|
| `[`file.py`]()`      | Code file reference (empty or any link)        |
| `[`path/file.py`]()` | Code file with partial path for disambiguation |
| `[`file.py`](#L42)`  | Preserves existing line fragments              |
| `[Doc Name](.md)`    | Doc-to-doc link (resolves by name)             |
<!-- doclinks-ignore-end -->

## How resolution works

The tool builds an index of all files in the repo. For `/dimos/protocol/service/spec.py`, it creates lookup entries for:

- `spec.py`
- `service/spec.py`
- `protocol/service/spec.py`
- `dimos/protocol/service/spec.py`

Use longer paths when multiple files share the same name.

</details>



<br>
<br>

# 2. Code Blocks Must Be Executable

We use [md-babel-py](https://github.com/leshy/md-babel-py/) to execute code blocks in markdown and insert results.

## Golden Rule

**Never write illustrative/pseudo code blocks.** If you're showing an API usage pattern, create a minimal working example that actually runs. This ensures documentation stays correct as the codebase evolves.

## Installation

<details>
<summary>Click to see full installation instructions</summary>

### Nix (recommended)

```sh skip
# (assuming you have nix)

# Run directly from GitHub
nix run github:leshy/md-babel-py -- run README.md --stdout

# run locally
nix run . -- run README.md --stdout
```

### Docker

```sh skip
# Pull from Docker Hub
docker run -v $(pwd):/work lesh/md-babel-py:main run /work/README.md --stdout

# Or build locally via Nix
nix build .#docker     # builds tarball to ./result
docker load < result   # loads image from tarball
docker run -v $(pwd):/work md-babel-py:latest run /work/file.md --stdout
```

### pipx

```sh skip
pipx install md-babel-py
# or: uv pip install md-babel-py
md-babel-py run README.md --stdout
```

If not using nix or docker, evaluators require system dependencies:

| Language  | System packages             |
|-----------|-----------------------------|
| python    | python3                     |
| node      | nodejs                      |
| dot       | graphviz                    |
| asymptote | asymptote, texlive, dvisvgm |
| pikchr    | pikchr                      |
| openscad  | openscad, xvfb, imagemagick |
| diagon    | diagon                      |

```sh skip
# Arch Linux
sudo pacman -S python nodejs graphviz asymptote texlive-basic openscad xorg-server-xvfb imagemagick

# Debian/Ubuntu
sudo apt-get install python3 nodejs graphviz asymptote texlive xvfb imagemagick openscad
```

Note: pikchr and diagon may need to be built from source. Use Docker or Nix for full evaluator support.

## Usage

```sh skip
# Edit file in-place
md-babel-py run document.md

# Output to separate file
md-babel-py run document.md --output result.md

# Print to stdout
md-babel-py run document.md --stdout

# Only run specific languages
md-babel-py run document.md --lang python,sh

# Dry run - show what would execute
md-babel-py run document.md --dry-run
```

</details>


## Running

```sh skip
md-babel-py run document.md          # edit in-place
md-babel-py run document.md --stdout # preview to stdout
md-babel-py run document.md --dry-run # show what would run
```

## Supported Languages

Python, Shell (sh), Node.js, plus visualization: Matplotlib, Graphviz, Pikchr, Asymptote, OpenSCAD, Diagon.

## Code Block Flags

Add flags after the language identifier:

| Flag | Effect |
|------|--------|
| `session=NAME` | Share state between blocks with same session name |
| `output=path.png` | Write output to file instead of inline |
| `no-result` | Execute but don't insert result |
| `skip` | Don't execute this block |
| `expected-error` | Block is expected to fail |

## Examples

# md-babel-py

Execute code blocks in markdown files and insert the results.

![Demo](assets/screencast.gif)

**Use cases:**
- Keep documentation examples up-to-date automatically
- Validate code snippets in docs actually work
- Generate diagrams and charts from code in markdown
- Literate programming with executable documentation

## Languages

### Shell

```sh
echo "cwd: $(pwd)"
```

<!--Result:-->
```
cwd: /work
```

### Python

```python session=example
a = "hello world"
print(a)
```

<!--Result:-->
```
hello world
```

Sessions preserve state between code blocks:

```python session=example
print(a, "again")
```

<!--Result:-->
```
hello world again
```

### Node.js

```node
console.log("Hello from Node.js");
console.log(`Node version: ${process.version}`);
```

<!--Result:-->
```
Hello from Node.js
Node version: v22.21.1
```

### Matplotlib

```python output=assets/matplotlib-demo.svg
import matplotlib.pyplot as plt
import numpy as np
plt.style.use('dark_background')
x = np.linspace(0, 4 * np.pi, 200)
plt.figure(figsize=(8, 4))
plt.plot(x, np.sin(x), label='sin(x)', linewidth=2)
plt.plot(x, np.cos(x), label='cos(x)', linewidth=2)
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.grid(alpha=0.3)
plt.savefig('{output}', transparent=True)
```

<!--Result:-->
![output](assets/matplotlib-demo.svg)






<br>
<br>

# 3. Diagrams

We have many diagraming tools. View source code of this page to see examples.


### Pikchr

SQLite's diagram language:

<details>
<summary>diagram source</summary>

```pikchr fold output=assets/pikchr-demo.svg
color = white
fill = none
linewid = 0.4in

# Input file
In: file "README.md" fit
arrow

# Processing
Parse: box "Parse" rad 5px fit
arrow
Exec: box "Execute" rad 5px fit

# Fan out to languages
arrow from Exec.e right 0.3in then up 0.4in then right 0.3in
Sh: oval "Shell" fit
arrow from Exec.e right 0.3in then right 0.3in
Node: oval "Node" fit
arrow from Exec.e right 0.3in then down 0.4in then right 0.3in
Py: oval "Python" fit

# Merge back
X: dot at (Py.e.x + 0.3in, Node.e.y) invisible
line from Sh.e right until even with X then down to X
line from Node.e to X
line from Py.e right until even with X then up to X
Out: file "README.md" fit with .w at (X.x + 0.3in, X.y)
arrow from X to Out.w
```

</details>

<!--Result:-->
![output](assets/pikchr-demo.svg)

### Asymptote

Vector graphics:

```asymptote output=assets/histogram.svg
import graph;
import stats;

size(400,200,IgnoreAspect);
defaultpen(white);

int n=10000;
real[] a=new real[n];
for(int i=0; i < n; ++i) a[i]=Gaussrand();

draw(graph(Gaussian,min(a),max(a)),orange);

int N=bins(a);

histogram(a,min(a),max(a),N,normalize=true,low=0,rgb(0.4,0.6,0.8),rgb(0.2,0.4,0.6),bars=true);

xaxis("$x$",BottomTop,LeftTicks,p=white);
yaxis("$dP/dx$",LeftRight,RightTicks(trailingzero),p=white);
```

<!--Result:-->
![output](assets/histogram.svg)

### Graphviz

```dot output=assets/graph.svg
A -> B -> C
A -> C
```

<!--Result:-->
![output](assets/graph.svg)

### OpenSCAD

```openscad output=assets/cube-sphere.png
cube([10, 10, 10]);
sphere(r=7);
```

<!--Result:-->
![output](assets/cube-sphere.png)

### Diagon

ASCII art diagrams:

```diagon mode=Math
1 + 1/2 + sum(i,0,10)
```

<!--Result:-->
```
        10
        ___
    1   ╲
1 + ─ + ╱   i
    2   ‾‾‾
         0
```

```diagon mode=GraphDAG
A -> B -> C
A -> C
```

<!--Result:-->
```
┌───┐
│A  │
└┬─┬┘
 │┌▽┐
 ││B│
 │└┬┘
┌▽─▽┐
│C  │
└───┘
```

## Reference

| Element | Syntax |
|---------|--------|
| Box | `box "text" rad 5px wid Xin ht Yin` |
| Arrow | `arrow right 0.3in` |
| Oval | `oval "text" wid Xin ht Yin` |
| Text | `text "label" at (X, Y)` |
| Named point | `A: box ...` then reference `A.e`, `A.n`, `A.x`, `A.y` |

See [pikchr.org/home/doc/trunk/doc/userman.md](https://pikchr.org/home/doc/trunk/doc/userman.md) for full documentation.
