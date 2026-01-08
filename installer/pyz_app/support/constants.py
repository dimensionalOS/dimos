#!/usr/bin/env python3
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

discord_url = "https://discord.gg/S6E9MHsu"

minimum_nix_version = "2.24.12"

dimos_env_vars = {
    "OPENAI_API_KEY": "",
    "HUGGINGFACE_ACCESS_TOKEN": "",
    "ALIBABA_API_KEY": "",
    "ANTHROPIC_API_KEY": "",
    "HF_TOKEN": "",
    "HUGGINGFACE_PRV_ENDPOINT": "",
    "ROBOT_IP": "",
    "CONN_TYPE": "webrtc",
    "WEBRTC_SERVER_HOST": "0.0.0.0",
    "WEBRTC_SERVER_PORT": "9991",
    "DISPLAY": ":0",
}

PLACEHOLDERS = ("YOUR_DIMOS_PROJECT_NAME", "YOUR_DIMOS_PROJECT_DESCRIPTION")

# NOTE: these hardcoded lists are added to pip-package-derived dependencies
#       there is a pip module name => system dependencies mapping
#       THEN based on the features that a user picked, we find all the
#       pip packages that are needed for those features
#       then calculate the system dependencies needed for that set of packages
#       This is actually already being done for apt-get and brew,
# however sometimes stuff is still missed from there, so we add those here
DEPENDENCY_HUMAN_NAMES_SET = set(
    (
        "git",
        "git-lfs",
        "python (version 3.10 or higher)",
        "cmake",
        "ffmpeg",
        "portaudio",
        "pkg-config",
        "ninja",
        "opencv",
        # "rust",
        # "zlib", # the tools above are almost certainly going to download this anyways
        # "libpng", # opencv is almost certainly going to download this anyways
        # "libjpeg",# opencv is almost certainly going to download this anyways
        # "portmidi",
        # "eigen",
        # "jsoncpp",
        # "libsndfile",
        # "opus",
        # "libvpx",
        # "jpeg-turbo",
        # "openblas",
        # "lapack",
        # "protobuf",
        # "sdl2",
        # "sdl2_image",
        # "sdl2_mixer",
        # "sdl2_ttf",
    )
)

DEPENDENCY_BREW_SET_MINIMAL = set(
    (
        "git",
        "git-lfs",
        "ffmpeg",
        # "portaudio",
        # "pkg-config",
        # "cmake",
        # "ninja",
        # "opencv",
        # "rust",
        # "zlib", # the tools above are almost certainly going to download this anyways
        # "libpng", # opencv is almost certainly going to download this anyways
        # "libjpeg",# opencv is almost certainly going to download this anyways
        # "portmidi",
        # "eigen",
        # "jsoncpp",
        # "libsndfile",
        # "opus",
        # "libvpx",
        # "jpeg-turbo",
        # "openblas",
        # "lapack",
        # "protobuf",
        # "sdl2",
        # "sdl2_image",
        # "sdl2_mixer",
        # "sdl2_ttf",
    )
)


DEPENDENCY_NIX_PACKAGES_SET_MINIMAL = set(
    (
        "pkgs.git",
        "pkgs.git-lfs",
        "pkgs.ffmpeg_6",
        "pkgs.ffmpeg_6.dev",
        # "pkgs.cmake",
        # # "pkgs.pcre2",
        # # "pkgs.gnugrep",
        # # "pkgs.gnused",
        # "pkgs.pkg-config",
        # "pkgs.unixtools.ifconfig",
        # "pkgs.unixtools.netstat",
        # "pkgs.python312",
        # "pkgs.python312Packages.pip",
        # "pkgs.python312Packages.setuptools",
        # "pkgs.python312Packages.virtualenv",
        # "pkgs.python312Packages.gst-python",
        # # "pkgs.pre-commit",
        # "pkgs.portaudio",
        # "pkgs.mesa",
        # "pkgs.glfw",
        # "pkgs.udev",
        # "pkgs.SDL2",
        # "pkgs.SDL2.dev",
        # "pkgs.gtk3",
        # "pkgs.gdk-pixbuf",
        # "pkgs.gobject-introspection",
        # "pkgs.gst_all_1.gstreamer",
        # "pkgs.gst_all_1.gst-plugins-base",
        # "pkgs.gst_all_1.gst-plugins-good",
        # "pkgs.gst_all_1.gst-plugins-bad",
        # "pkgs.gst_all_1.gst-plugins-ugly",
        # "pkgs.eigen",
        # "pkgs.ninja",
        # "pkgs.jsoncpp",
        # "pkgs.lcm",
        # "pkgs.libGL",
        # "pkgs.libGLU",
        # "pkgs.xorg.libX11",
        # "pkgs.xorg.libXi",
        # "pkgs.xorg.libXext",
        # "pkgs.xorg.libXrandr",
        # "pkgs.xorg.libXinerama",
        # "pkgs.xorg.libXcursor",
        # "pkgs.xorg.libXfixes",
        # "pkgs.xorg.libXrender",
        # "pkgs.xorg.libXdamage",
        # "pkgs.xorg.libXcomposite",
        # "pkgs.xorg.libxcb",
        # "pkgs.xorg.libXScrnSaver",
        # "pkgs.xorg.libXxf86vm",
        # "pkgs.zlib",
        # "pkgs.glib",
        # "pkgs.libjpeg",
        # "pkgs.libjpeg_turbo",
        # "pkgs.libpng",
    )
)


DEPENDENCY_APT_PACKAGES_SET_MINIMAL = set(
    (
        "git",
        "git-lfs",
        "ffmpeg",
        "iproute2",
        "net-tools",
        "curl",
        # "cmake",
        # "pkg-config",
        # "portaudio",
        # "build-essential",
        # "gnupg2",
        # "python3-pip",
        # "mesa-utils",
        # "rustc",
        # the remainder are a combination of ones from the docker file(s), from pip packages, and hand-picked ones
        # "lsb-release",
        # "clang",
        # "portaudio19-dev",
        # "libgl1-mesa-glx",
        # "libgl1-mesa-dri",
        # "software-properties-common",
        # "libxcb1-dev",
        # "libxcb-keysyms1-dev",
        # "libxcb-util0-dev",
        # "libxcb-icccm4-dev",
        # "libxcb-image0-dev",
        # "libxcb-randr0-dev",
        # "libxcb-shape0-dev",
        # "libxcb-xinerama0-dev",
        # "libxcb-xkb-dev",
        # "libxkbcommon-x11-dev",
        # "qtbase5-dev",
        # "qtchooser",
        # "qt5-qmake",
        # "qtbase5-dev-tools",
        # "supervisor",
        # "liblcm-d",
        # "ninja-build",
        # "python3-dev",
        # "python3-setuptools",
        # "python3-wheel",
        # "gfortran",
        # "cargo",
        # "cython3",
        # "libgl1",
        # "libglib2.0-0",
        # "libgomp1",
        # "libportaudio2",
        # "libasound2-dev",
        # "libavcodec-dev",
        # "libavformat-dev",
        # "libavdevice-dev",
        # "libavutil-dev",
        # "libswscale-dev",
        # "libswresample-dev",
        # "libavfilter-dev",
        # "libopus-dev",
        # "libvpx-dev",
        # "libsndfile1",
        # "libsndfile1-dev",
        # "zlib1g-dev",
        # "libjpeg8-dev",
        # "libtiff5-dev",
        # "libopenjp2-7-dev",
        # "libfreetype6-dev",
        # "liblcms2-dev",
        # "libwebp-dev",
        # "tcl8.6-dev",
        # "tk8.6-dev",
        # "python3-tk",
        # "libharfbuzz-dev",
        # "libfribidi-dev",
        # "libturbojpeg0",
        # "libturbojpeg0-dev",
        # "libopenblas-dev",
        # "liblapack-dev",
        # "protobuf-compiler",
        # "libprotobuf-dev",
        # "libsdl2-dev",
        # "libsdl2-image-dev",
        # "libsdl2-mixer-dev",
        # "libsdl2-ttf-dev",
        # "libportmidi-dev",
    )
)


DEFAULT_GITIGNORE_CONTENT = """
# generic ignore pattern
**/*.ignore
**/*.ignore.*

# MacOS
**/.DS_Store

# direnv/dotenv
.env
# .envrc
.direnv/

# Byte-compiled / optimized / DLL files
__pycache__/
*.py[codz]
*$py.class

# C extensions
*.so

# Distribution / packaging
.Python
build/
develop-eggs/
dist/
downloads/
eggs/
.eggs/
lib/
lib64/
parts/
sdist/
var/
wheels/
share/python-wheels/
*.egg-info/
.installed.cfg
*.egg
MANIFEST

# PyInstaller
#  Usually these files are written by a python script from a template
#  before PyInstaller builds the exe, so as to inject date/other infos into it.
*.manifest
*.spec

# Installer logs
pip-log.txt
pip-delete-this-directory.txt

# Unit test / coverage reports
htmlcov/
.tox/
.nox/
.coverage
.coverage.*
.cache
nosetests.xml
coverage.xml
*.cover
*.py.cover
.hypothesis/
.pytest_cache/
cover/

# Translations
*.mo
*.pot

# Django stuff:
*.log
local_settings.py
db.sqlite3
db.sqlite3-journal

# Flask stuff:
instance/
.webassets-cache

# Scrapy stuff:
.scrapy

# Sphinx documentation
docs/_build/

# PyBuilder
.pybuilder/
target/

# Jupyter Notebook
.ipynb_checkpoints

# IPython
profile_default/
ipython_config.py

# pyenv
#   For a library or package, you might want to ignore these files since the code is
#   intended to run in multiple environments; otherwise, check them in:
# .python-version

# pipenv
#   According to pypa/pipenv#598, it is recommended to include Pipfile.lock in version control.
#   However, in case of collaboration, if having platform-specific dependencies or dependencies
#   having no cross-platform support, pipenv may install dependencies that don't work, or not
#   install all needed dependencies.
#Pipfile.lock

# UV
#   Similar to Pipfile.lock, it is generally recommended to include uv.lock in version control.
#   This is especially recommended for binary packages to ensure reproducibility, and is more
#   commonly ignored for libraries.
#uv.lock

# poetry
#   Similar to Pipfile.lock, it is generally recommended to include poetry.lock in version control.
#   This is especially recommended for binary packages to ensure reproducibility, and is more
#   commonly ignored for libraries.
#   https://python-poetry.org/docs/basic-usage/#commit-your-poetrylock-file-to-version-control
#poetry.lock
#poetry.toml

# pdm
#   Similar to Pipfile.lock, it is generally recommended to include pdm.lock in version control.
#   pdm recommends including project-wide configuration in pdm.toml, but excluding .pdm-python.
#   https://pdm-project.org/en/latest/usage/project/#working-with-version-control
#pdm.lock
#pdm.toml
.pdm-python
.pdm-build/

# pixi
#   Similar to Pipfile.lock, it is generally recommended to include pixi.lock in version control.
#pixi.lock
#   Pixi creates a virtual environment in the .pixi directory, just like venv module creates one
#   in the .venv directory. It is recommended not to include this directory in version control.
.pixi

# PEP 582; used by e.g. github.com/David-OConnor/pyflow and github.com/pdm-project/pdm
__pypackages__/

# Celery stuff
celerybeat-schedule
celerybeat.pid

# SageMath parsed files
*.sage.py

# Environments
.venv
env/
venv/
ENV/
env.bak/
venv.bak/

# Spyder project settings
.spyderproject
.spyproject

# Rope project settings
.ropeproject

# mkdocs documentation
/site

# mypy
.mypy_cache/
.dmypy.json
dmypy.json

# Pyre type checker
.pyre/

# pytype static type analyzer
.pytype/

# Cython debug symbols
cython_debug/

# PyCharm
#  JetBrains specific template is maintained in a separate JetBrains.gitignore that can
#  be found at https://github.com/github/gitignore/blob/main/Global/JetBrains.gitignore
#  and can be added to the global gitignore or merged into this file.  For a more nuclear
#  option (not recommended) you can uncomment the following to ignore the entire idea folder.
#.idea/

# Abstra
# Abstra is an AI-powered process automation framework.
# Ignore directories containing user credentials, local state, and settings.
# Learn more at https://abstra.io/docs
.abstra/

# Visual Studio Code
#  Visual Studio Code specific template is maintained in a separate VisualStudioCode.gitignore
#  that can be found at https://github.com/github/gitignore/blob/main/Global/VisualStudioCode.gitignore
#  and can be added to the global gitignore or merged into this file. However, if you prefer,
#  you could uncomment the following to ignore the entire vscode folder
# .vscode/

# Ruff stuff:
.ruff_cache/

# PyPI configuration file
.pypirc

# Cursor
#  Cursor is an AI-powered code editor. `.cursorignore` specifies files/directories to
#  exclude from AI features like autocomplete and code analysis. Recommended for sensitive data
#  refer to https://docs.cursor.com/context/ignore-files
.cursorignore
.cursorindexingignore

# Marimo
marimo/_static/
marimo/_lsp/
__marimo__/
"""
