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

# NOTE: instead of these haradcoded lists, ideally there would be a 
#       pip module name => system dependencies mapping
#       THEN based on the features that a user picked, we find all the
#       pip packages that are needed for those features
#       then calculate the system dependencies needed for that set of packages
#       This is actually already being done for apt-get and brew, 
#       but not human-readable names (yet) which is the most important one
#       as a fallback we have a hardcoded list here
dependency_list_human_names = [
    "git",
    "git-lfs",
    "portaudio",
    "pkg-config",
    "cmake",
    "ninja",
    "python (version 3.12 or higher)",
    "opencv",
    "rust",
    "ffmpeg",
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
]


dependencies_nix_names = [
    "pkgs.git",
    "pkgs.git-lfs",
    "pkgs.cmake",
    "pkgs.pcre2",
    "pkgs.gnugrep",
    "pkgs.gnused",
    "pkgs.pkg-config",
    "pkgs.unixtools.ifconfig",
    "pkgs.unixtools.netstat",
    "pkgs.python312",
    "pkgs.python312Packages.pip",
    "pkgs.python312Packages.setuptools",
    "pkgs.python312Packages.virtualenv",
    "pkgs.python312Packages.gst-python",
    "pkgs.pre-commit",
    "pkgs.portaudio",
    "pkgs.ffmpeg_6",
    "pkgs.ffmpeg_6.dev",
    "pkgs.mesa",
    "pkgs.glfw",
    "pkgs.udev",
    "pkgs.SDL2",
    "pkgs.SDL2.dev",
    "pkgs.gtk3",
    "pkgs.gdk-pixbuf",
    "pkgs.gobject-introspection",
    "pkgs.gst_all_1.gstreamer",
    "pkgs.gst_all_1.gst-plugins-base",
    "pkgs.gst_all_1.gst-plugins-good",
    "pkgs.gst_all_1.gst-plugins-bad",
    "pkgs.gst_all_1.gst-plugins-ugly",
    "pkgs.eigen",
    "pkgs.ninja",
    "pkgs.jsoncpp",
    "pkgs.lcm",
    "pkgs.libGL",
    "pkgs.libGLU",
    "pkgs.xorg.libX11",
    "pkgs.xorg.libXi",
    "pkgs.xorg.libXext",
    "pkgs.xorg.libXrandr",
    "pkgs.xorg.libXinerama",
    "pkgs.xorg.libXcursor",
    "pkgs.xorg.libXfixes",
    "pkgs.xorg.libXrender",
    "pkgs.xorg.libXdamage",
    "pkgs.xorg.libXcomposite",
    "pkgs.xorg.libxcb",
    "pkgs.xorg.libXScrnSaver",
    "pkgs.xorg.libXxf86vm",
    "pkgs.zlib",
    "pkgs.glib",
    "pkgs.libjpeg",
    "pkgs.libjpeg_turbo",
    "pkgs.libpng",
]


dependency_list_apt_packages = [
    "git",
    "git-lfs",
    "build-essential",
    "pkg-config",
    "cmake",
    "ninja-build",
    "git",
    "python3-dev",
    "python3-pip",
    "python3-setuptools",
    "python3-wheel",
    "gfortran",
    "rustc",
    "cargo",
    "cython3",
    "libgl1",
    "libglib2.0-0",
    "libgomp1",
    "portaudio19-dev",
    "libportaudio2",
    "libasound2-dev",
    "ffmpeg",
    "libavcodec-dev",
    "libavformat-dev",
    "libavdevice-dev",
    "libavutil-dev",
    "libswscale-dev",
    "libswresample-dev",
    "libavfilter-dev",
    "libopus-dev",
    "libvpx-dev",
    "libsndfile1",
    "libsndfile1-dev",
    "zlib1g-dev",
    "libjpeg8-dev",
    "libtiff5-dev",
    "libopenjp2-7-dev",
    "libfreetype6-dev",
    "liblcms2-dev",
    "libwebp-dev",
    "tcl8.6-dev",
    "tk8.6-dev",
    "python3-tk",
    "libharfbuzz-dev",
    "libfribidi-dev",
    "libxcb1-dev",
    "libturbojpeg0",
    "libturbojpeg0-dev",
    "libopenblas-dev",
    "liblapack-dev",
    "protobuf-compiler",
    "libprotobuf-dev",
    "libsdl2-dev",
    "libsdl2-image-dev",
    "libsdl2-mixer-dev",
    "libsdl2-ttf-dev",
    "libportmidi-dev",
]


__all__ = [
    "dependencies_nix_names",
    "dependency_list_apt_packages",
    "dependency_list_human_names",
    "dimos_env_vars",
    "discord_url",
]
