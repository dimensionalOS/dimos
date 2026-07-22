# Copyright 2025-2026 Dimensional Inc.
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
from pathlib import Path

from docutils import nodes
import tomllib

# -- Project information -----------------------------------------------------

_PYPROJECT = Path(__file__).parents[1] / "pyproject.toml"
# Parse the version from pyproject.toml rather than importing the package.
release = tomllib.loads(_PYPROJECT.read_text("utf-8"))["project"]["version"]
version = ".".join(release.split(".")[:2])

project = "dimos"
copyright = "2025-2026, Dimensional Inc."
author = "Dimensional Inc."

github_url = "https://github.com"
github_repo_org = "dimensionalOS"
github_repo_name = "dimos"
github_repo_slug = f"{github_repo_org}/{github_repo_name}"
github_repo_url = f"{github_url}/{github_repo_slug}"

# -- General configuration ---------------------------------------------------

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",  # parse the Google-style ``Args:``/``Returns:`` docstrings
    "sphinx.ext.extlinks",
    "sphinx.ext.intersphinx",
    "sphinx.ext.viewcode",
]

try:
    import sphinxcontrib.spelling  # noqa: F401

    extensions.append("sphinxcontrib.spelling")
except ImportError:
    # Spelling is optional locally (e.g. unavailable in Windows), checked in CI.
    pass

source_suffix = ".rst"
master_doc = "index"
exclude_patterns = ["_build"]

# The default language to highlight source code in.
highlight_language = "python3"

# An inline ``:bash:`` role (a code role with Bash highlighting) for shell commands
# referenced in prose, matching the highlighting of ``.. code-block:: bash`` blocks.
rst_prolog = """
.. role:: bash(code)
   :language: bash
"""

# -- Options for autodoc -----------------------------------------------------

# The API reference is generated from docstrings, with type annotations rendered
# in the signatures. Every annotation becomes a cross-reference resolved under
# ``nitpicky``; dimos types are documented below, external ones via intersphinx
# or ``nitpick_ignore``.
autodoc_member_order = "bysource"
autodoc_typehints = "signature"
autoclass_content = "class"
add_module_names = False
autodoc_default_options = {
    "members": True,
    "undoc-members": True,
    # model_config is pydantic's internal ClassVar and not part of the dimos API.
    "exclude-members": "model_config",
}

# -- Options for intersphinx -------------------------------------------------

intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "numpy": ("https://numpy.org/doc/stable/", None),
    "pydantic": ("https://docs.pydantic.dev/latest/", None),
    "dask": ("https://distributed.dask.org/en/stable/", None),
    "reactivex": ("https://rxpy.readthedocs.io/en/latest/", None),
    "rerun": ("https://ref.rerun.io/docs/python/stable/", None),
}

# -- Options for extlinks ----------------------------------------------------

extlinks = {
    "issue": (f"{github_repo_url}/issues/%s", "#%s"),
    "pr": (f"{github_repo_url}/pull/%s", "PR #%s"),
    "commit": (f"{github_repo_url}/commit/%s", "%s"),
    "gh": (f"{github_url}/%s", "GitHub: %s"),
    "user": (f"{github_url}/%s", "@%s"),
}

# -- Options for HTML output -------------------------------------------------

# The default theme for now; a dedicated theme is added in a later PR.
html_theme = "alabaster"

# -- Options for the spelling builder ----------------------------------------

spelling_warning = True
# Acronyms (LCM, ROS, RPC, SLAM) and CamelCase names (DimOS, MuJoCo, NumPy, WebRTC)
# are skipped automatically. Code entities use their Python-domain roles (:mod:,
# :func:, :class:), abbreviations/programs use :abbr:/:program:, and product names use
# the custom :brand: role (registered in setup() below) — all skipped by the checker —
# so the wordlist only holds genuine prose vocabulary.
spelling_ignore_acronyms = True
spelling_ignore_wiki_words = True

# -- Nitpicky mode -----------------------------------------------------------

nitpicky = True
nitpick_ignore: list[tuple[str, str]] = [
    # TypeVars / ParamSpecs rendered in signatures — type parameters, not
    # documentable targets.
    ("py:class", "T"),
    ("py:class", "P"),
    ("py:class", "R"),
    ("py:class", "dimos.core.stream.T"),
    ("py:class", "dimos.core.core.T"),
    # Upstream entities that are not documented.
    ("py:class", "langchain_core.messages.base.BaseMessage"),
    ("py:class", "distributed.ActorFuture"),
    # TODO(PY315): Fix these references with lazy imports.
    ("py:class", "Observable"),
    ("py:class", "DisposableBase"),
    ("py:class", "np.ndarray"),
    ("py:class", "np.dtype"),
    ("py:class", "Connection"),
]

nitpick_ignore_regex = [
    # TODO: Add Sphinx docs to dimos_lcm.
    ("py:class", r"dimos_lcm([._].*)?"),
    # TODO(PY315): Fix these references with lazy imports.
    ("py:class", r"rr([._].*)?"),
    ("py:class", r"Archetype"),
]


def setup(app):
    """Register a ``:brand:`` role for product names that have no built-in role.

    The role wraps its text in an inline node — a ``<span class="brand">`` — so it
    renders as ordinary prose, can be styled via CSS, and is skipped by the spelling
    checker.
    """

    def brand(name, rawtext, text, lineno, inliner, options=None, content=None):
        # A structural ``inline`` node (rather than tagging a Text node) survives
        # doctree pickling — CI builds html before spelling — and carries a
        # ``brand`` class for styling.
        return [nodes.inline(rawtext, text, classes=["brand"])], []

    app.add_role("brand", brand)
