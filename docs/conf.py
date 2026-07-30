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
import sys

import tomllib

_REPO_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(_REPO_ROOT))

# -- Project information -----------------------------------------------------

_PYPROJECT = _REPO_ROOT / "pyproject.toml"
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
    "myst_parser",
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",  # parse the Google-style ``Args:``/``Returns:`` docstrings
    "sphinx.ext.extlinks",
    "sphinx.ext.intersphinx",
    "sphinx.ext.viewcode",
    "sphinx_codeautolink",
    "sphinx_design",
]

try:
    import sphinxcontrib.spelling  # noqa: F401

    extensions.append("sphinxcontrib.spelling")
except ImportError:
    # Spelling is optional locally (e.g. unavailable in Windows), checked in CI.
    pass

source_suffix = {
    ".md": "markdown",
    ".rst": "restructuredtext",
}
master_doc = "index"
exclude_patterns = [
    "_build",
    # Empty placeholder: keep its source intact, but do not publish a blank page.
    "capabilities/perception/index.md",
]

# MyST keeps Markdown authoring compatible with Sphinx roles and directives.
# ``colon_fence`` makes nested directives (cards, grids, notes) readable without
# backtick-fence collisions, while substitutions preserve the diagrams inherited
# from the original documentation.
myst_enable_extensions = ["colon_fence", "substitution"]
myst_heading_anchors = 3
myst_substitutions = {"release": release}

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

html_theme = "pydata_sphinx_theme"
html_title = "Dimensional · DimOS"
html_logo = "_static/dimensional-logo-master-transparent.png"
html_favicon = "_static/favicon.png"
html_static_path = ["_static"]
html_css_files = ["dimensional.css"]
templates_path = ["_templates"]
html_sidebars = {
    "**": ["sidebar-collapse.html", "sidebar-global-nav.html"],
}
html_context = {
    # Match the hosted documentation while still respecting the reader's saved
    # theme choice. PyData's switcher exposes light, dark, and system modes.
    "default_mode": "dark",
}
html_theme_options = {
    # Use supported theme regions instead of positioning sidebar fragments as
    # a custom header. This keeps the mobile menu and theme switcher native.
    "navbar_start": ["navbar-logo"],
    "navbar_center": ["search-button-field"],
    "navbar_end": ["navbar-icon-links"],
    # Persistent items stay in the top bar at mobile widths instead of moving
    # into the navigation drawer.
    "navbar_persistent": ["theme-switcher"],
    "navbar_align": "content",
    "search_bar_text": "Search docs",
    "icon_links": [
        {
            "name": "GitHub",
            "url": github_repo_url,
            "icon": "fa-brands fa-github",
            "type": "fontawesome",
        },
    ],
    # Avoid a second toolbar between the global header and the document title.
    "article_header_start": [],
    "article_header_end": [],
    "primary_sidebar_end": [],
    "secondary_sidebar_items": ["page-toc"],
    "show_nav_level": 1,
    "show_toc_level": 2,
    "navigation_with_keys": True,
    "back_to_top_button": True,
}

# -- Options for the spelling builder ----------------------------------------

spelling_warning = True
# Acronyms (LCM, ROS, RPC, SLAM) and CamelCase names (DimOS, MuJoCo, NumPy, WebRTC)
# are skipped automatically. API names use linked inline code, and the wordlist holds
# product names and genuine prose vocabulary.
spelling_ignore_acronyms = True
spelling_ignore_wiki_words = True

# -- Nitpicky mode -----------------------------------------------------------

nitpicky = True
# ``highlight_language`` makes otherwise untyped literal blocks use the Python
# lexer. Code-autolink safely skips non-Python examples (logs, file trees, and
# configuration snippets); do not promote those expected parse failures to
# strict-build warnings.
suppress_warnings = ["codeautolink.parse_block"]
nitpick_ignore: list[tuple[str, str]] = [
    # TypeVars / ParamSpecs rendered in signatures — type parameters, not
    # documentable targets.
    ("py:class", "T"),
    ("py:class", "P"),
    ("py:class", "R"),
    ("py:class", "dimos.core.stream.T"),
    ("py:class", "dimos.core.core.T"),
    ("py:class", "dimos.agents.annotation.F"),
    ("py:class", "dimos.types.timestamped.PRIMARY"),
    ("py:class", "dimos.types.timestamped.SECONDARY"),
    ("py:class", "dimos.utils.reactive.T"),
    ("py:class", "dimos.utils.reactive.LatestReader"),
    ("py:class", "dimos.memory2.transform.T"),
    # Upstream entities that are not documented.
    ("py:class", "langchain_core.messages.base.BaseMessage"),
    ("py:class", "distributed.ActorFuture"),
    ("py:class", "NDArray"),
    ("py:class", "np.int8"),
    ("py:class", "open3d.cuda.pybind.geometry.PointCloud"),
    ("py:class", "open3d.cuda.pybind.t.geometry.PointCloud"),
    # Types referenced by documented signatures but not part of this API slice.
    ("py:class", "dimos.msgs.protocol.DimosMsg"),
    ("py:class", "dimos.protocol.pubsub.impl.zenohpubsub.Topic"),
    ("py:class", "dimos.msgs.trajectory_msgs.TrajectoryPoint.TrajectoryPoint"),
    ("py:class", "VoxelGrid"),
    ("py:class", "dimos.hardware.whole_body.spec.WholeBodyConfig"),
    ("py:class", "WorldRobotID"),
    ("py:class", "PlanningGroup"),
    ("py:class", "JointState"),
    # A prose return condition in align_timestamped's Google-style docstring.
    ("py:class", "If single secondary observable"),
    # Supporting memory and visualization types outside the documented slice.
    ("py:class", "LayoutAlgo"),
    ("py:class", "EmbeddingModel"),
    ("py:class", "EmbeddedObservation"),
    ("py:class", "dimos.memory2.backend.Backend"),
    ("py:class", "dimos.memory2.transform.Transformer"),
    ("py:class", "dimos.memory2.type.filter.StreamQuery"),
    ("py:class", "dimos.memory2.transform.FnIterTransformer"),
    ("py:class", "FnIterTransformer"),
    ("py:class", "Observation"),
    ("py:class", "DeferredColor"),
    ("py:class", "dimos.memory2.vis.plot.plot.TimeAxis"),
    ("py:class", "GeoPoint"),
    ("py:class", "GeoPose"),
    ("py:class", "ColorLike"),
    ("py:class", "dimos.perception.detection.type.detection2d.base.Detection2D"),
    ("py:class", "dimos.memory2.type.observation.Observation"),
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
    def shorten_long_class_signatures(app, what, name, obj, options, signature, return_annotation):
        """Collapse generated constructors that would overwhelm the API page."""
        if what == "class" and signature and len(signature) > 120:
            return "(...)", return_annotation
        return None

    app.connect("autodoc-process-signature", shorten_long_class_signatures)
