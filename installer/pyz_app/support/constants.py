#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

discord_url = "https://discord.gg/S6E9MHsu"

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

# NOTE: these hardcoded lists are added to pip-package-derived dependencies
#       there is a pip module name => system dependencies mapping
#       THEN based on the features that a user picked, we find all the
#       pip packages that are needed for those features
#       then calculate the system dependencies needed for that set of packages
#       This is actually already being done for apt-get and brew, 
# however sometimes stuff is still missed from there, so we add those here
dependency_human_names_set = set((
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
))

dependency_brew_set_minimal = set((
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
))


dependency_nix_packages_set_minimal = set((
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
))


dependency_apt_packages_set_minimal = set((
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
))


__all__ = [
    "dependencies_nix_names",
    "dependency_apt_packages_set_minimal",
    "dependency_human_names_set",
    "dimos_env_vars",
    "discord_url",
]
