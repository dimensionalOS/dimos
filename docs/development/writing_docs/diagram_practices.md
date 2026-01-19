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
