# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

from collections.abc import Iterable, Mapping, MutableMapping, Sequence

# NOTE: this dependency mapping doesn't need to be perfect. It'd be nice if we could generate it from a
#       package repo like apt or nix, but until then I just used chatGPT to estimate the obvious dependencies
#       for each package. This is just to minimize this list of human-printed out dependencies.

Graph = Mapping[str, Sequence[str]]
MutableGraph = MutableMapping[str, list[str]]
ClosureCache = MutableMapping[str, set[str]]


dependency_dag = {
    "assimp": ["cmake", "zlib", "eigen"],
    "cairo": ["fontconfig", "freetype", "xrender"],
    "can-utils": [],
    "ccd": [],
    "cmake": [],
    "eigen": [],
    "ethtool": [],
    "fcl": ["eigen", "octomap", "ccd", "qhull"],
    "ffmpeg": ["zlib", "lib jpeg", "lib png", "vpx", "webp", "opus"],
    "fontconfig": ["freetype"],
    "freeglut": ["x11-sm", "xext"],
    "freetype": ["zlib", "lib png"],
    "gcc": ["gmp", "mpfr", "mpc"],
    "geos": [],
    "git": ["openssl"],
    "git-lfs": ["git", "openssl"],
    "glew": [],
    "glfw": ["x11-sm", "xext"],
    "glib": [],
    "gmp": [],
    "gnutls": ["nettle", "gmp", "tasn1", "lib idn2"],
    "gobject-introspection": ["glib"],
    "gtk": ["glib", "gobject-introspection", "cairo"],
    "hdf5": ["zlib"],
    "lapack": ["openblas"],
    "lib idn2": ["lib unistring"],
    "lib jpeg": [],
    "lib png": ["zlib"],
    "lib sodium": [],
    "lib unistring": [],
    "lib xml 2": ["zlib"],
    "lib xslt": ["lib xml 2"],
    "lib yaml": [],
    "mesa": ["x11-sm", "xext"],
    "mpc": ["mpfr", "gmp"],
    "mpfr": ["gmp"],
    "nettle": ["gmp"],
    "ninja": [],
    "octomap": ["eigen"],
    "openblas": [],
    "opencv": [
        "eigen",
        "openblas",
        "ffmpeg",
        "tbb",
        "gtk",
        "zlib",
        "lib jpeg",
        "lib png",
        "tiff",
        "webp",
    ],
    "openmp": ["gcc"],
    "openssl": ["zlib"],
    "opus": [],
    "ossp-uuid": [],
    "pkg-config": [],
    "portaudio": [],
    "pygobject": ["python", "glib", "gobject-introspection"],
    "python": ["openssl", "zlib"],
    "qhull": [],
    "spatialindex": [],
    "tasn1": [],
    "tbb": [],
    "tcl-tk": [],
    "tiff": ["zlib", "lib jpeg"],
    "vpx": [],
    "webp": ["lib png", "lib jpeg", "tiff"],
    "x11-sm": [],
    "xext": ["x11-sm"],
    "xft": ["fontconfig", "freetype", "xrender"],
    "xrender": ["xext"],
    "zlib": [],
}  # type: dict[str, list[str]]


def dependency_closure(name: str, graph: Graph, _cache: ClosureCache | None = None) -> set[str]:
    """
    Return the transitive dependency set for `name`, including `name` itself.
    Results are memoized in `_cache` when provided to avoid recomputation.
    """
    if _cache is None:
        _cache = {}
    if name in _cache:
        return _cache[name]
    deps = set([name])
    for dep in graph.get(name, []):
        deps |= dependency_closure(dep, graph, _cache)
    _cache[name] = deps
    return deps


def find_cycle(graph: Graph) -> list[str] | None:
    """
    Return a representative cycle as a list of nodes ending where it starts, or None if acyclic.
    Dependencies not present as keys are treated as leaves (ignored for cycles).
    """
    visiting: set[str] = set()
    visited: set[str] = set()
    stack: list[str] = []
    index: dict[str, int] = {}

    def dfs(node: str) -> list[str] | None:
        visiting.add(node)
        index[node] = len(stack)
        stack.append(node)
        for dep in graph.get(node, []):
            if dep not in graph:
                continue
            if dep in visiting:
                start = index[dep]
                return [*stack[start:], dep]
            if dep not in visited:
                cycle = dfs(dep)
                if cycle:
                    return cycle
        visiting.remove(node)
        visited.add(node)
        stack.pop()
        index.pop(node, None)
        return None

    for node in graph:
        if node not in visited:
            cycle = dfs(node)
            if cycle:
                return cycle
    return None


def topological_order(graph: Graph) -> list[str]:
    """
    Kahn topological sort; nodes with no prerequisites come first.
    Missing dependencies are ignored (treated as external roots). Raises with cycle details.
    """
    dependents = {node: [] for node in graph}
    indegree = {node: 0 for node in graph}
    for node, deps in graph.items():
        for dep in deps:
            if dep not in graph:
                # Ignore dangling/missing dependencies; treated as external roots.
                continue
            indegree[node] += 1
            dependents.get(dep, []).append(node)
    ready = [node for node, degree in indegree.items() if degree == 0]
    order = []
    while ready:
        node = ready.pop()
        order.append(node)
        for nxt in dependents.get(node, []):
            indegree[nxt] -= 1
            if indegree[nxt] == 0:
                ready.append(nxt)
    if len(order) != len(graph):
        cycle = find_cycle(graph)
        if cycle:
            raise ValueError(f"Graph contains a cycle: {' -> '.join(cycle)}")
        raise ValueError("Graph contains a cycle")
    return order


def minimize_deps_based_on_prerequisites(names: Iterable[str]) -> list[str]:
    """
    Given a list of dependency names, return a minimal set of install targets
    whose prerequisites cover the full list. Uses a greedy reverse topological
    sweep; ties are broken by descending transitive dependency count. Unknown
    names are treated as leaves.
    """
    selected_graph: MutableGraph = {}
    for each in names:
        if each not in dependency_dag:
            selected_graph[each] = []
        else:
            selected_graph[each] = dependency_dag[each]

    order = topological_order(selected_graph)
    closures: ClosureCache = {}
    covered: set[str] = set()
    chosen: list[str] = []
    for name in reversed(order):
        closure = dependency_closure(name, selected_graph, closures)
        if name in covered:
            continue
        chosen.append(name)
        covered |= closure
        if len(covered) == len(selected_graph):
            break
    return sorted(
        chosen, key=lambda n: len(dependency_closure(n, selected_graph, closures)), reverse=True
    )


__all__ = [
    "dependency_closure",
    "dependency_dag",
    "find_cycle",
    "minimize_deps_based_on_prerequisites",
    "topological_order",
]


if __name__ == "__main__":
    roots = minimize_deps_based_on_prerequisites(dependency_dag.keys())
    closures: ClosureCache = {}
    covered: set[str] = set()
    for name in roots:
        covered |= dependency_closure(name, dependency_dag, closures)
    assert len(covered) == len(dependency_dag), "Coverage check failed"
    print("Minimal install roots:", roots)

    # Cycle detection smoke test
    cycle_graph = {"a": ["b"], "b": ["c"], "c": ["a"]}
    try:
        topological_order(cycle_graph)
        raise AssertionError("Expected cycle detection to raise")
    except ValueError as exc:
        msg = str(exc)
        assert "a -> b -> c -> a" in msg, f"Unexpected cycle message: {msg}"
        print(f"Cycle detection test passed; msg = {msg}")
