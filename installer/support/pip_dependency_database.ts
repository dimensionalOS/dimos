export default {
  "sounddevice": {
    "package": "sounddevice",
    "apt_dependencies": [
      "libportaudio2",
      "portaudio19-dev",
      "python3-dev",
      "libasound2-dev"
    ],
    "description": "System dependencies required for sounddevice pip package on Ubuntu/Debian systems",
    "notes": "sounddevice is a Python wrapper for PortAudio. libportaudio2 provides the runtime library for PortAudio. portaudio19-dev contains the development headers needed for building the Python bindings. python3-dev provides Python headers required for compiling the C extensions. libasound2-dev is the ALSA development library which PortAudio uses for audio I/O on Linux systems."
  },
  "fasttext": {
    "package": "fasttext",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "g++",
      "libstdc++6"
    ],
    "description": "System dependencies required for fasttext pip package on Ubuntu/Debian systems",
    "notes": "FastText is a library for efficient learning of word representations and sentence classification. It requires C++ compilation tools (g++, build-essential) to build the native extension modules. libstdc++6 provides the C++ standard library runtime. For pre-built wheels on common platforms (x86_64, arm64), only python3 and python3-pip are strictly required at runtime, but the build dependencies are included for systems that need to compile from source or for platforms without wheel support."
  },
  "tensorzero": {
    "package": "tensorzero==2025.7.5",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "ca-certificates",
      "openssl",
      "libssl-dev",
      "build-essential",
      "curl",
      "pkg-config"
    ],
    "description": "System dependencies required for tensorzero==2025.7.5 pip package on Ubuntu/Debian systems",
    "notes": "TensorZero is a Python package with Rust extensions built using PyO3 and maturin. For most x86_64 and aarch64 Linux systems, pre-built wheels are available on PyPI, which require minimal runtime dependencies (python3, python3-pip, ca-certificates, openssl). However, if pip needs to build from source (for unsupported architectures or when using source distributions), additional build dependencies are required: build-essential (gcc, g++, make), curl (for downloading Rust toolchain), pkg-config, and libssl-dev. The Rust toolchain itself will be installed automatically by maturin during the build process if not present. Python dependencies (httpx>=0.27.0, typing-extensions>=4.12.2, uuid-utils>=0.9.0) are managed by pip and do not require system packages.",
    "build_dependencies": [
      "build-essential",
      "curl",
      "pkg-config",
      "libssl-dev",
      "clang",
      "libc++-dev"
    ],
    "runtime_dependencies": [
      "python3",
      "python3-pip",
      "ca-certificates",
      "openssl"
    ]
  },
  "langchain-core": {
    "package": "langchain-core>=1,<2",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "cargo",
      "rustc",
      "libyaml-dev",
      "libssl-dev",
      "libffi-dev",
      "pkg-config"
    ],
    "description": "System dependencies required for langchain-core>=1,<2 pip package on Ubuntu/Debian systems",
    "notes": "langchain-core is the core abstractions and base classes for the LangChain framework. Key system dependencies: (1) cargo/rustc are required for building pydantic-core (Pydantic v2's Rust-based validation core), (2) libyaml-dev for PyYAML C extensions, (3) build-essential for uuid-utils C extensions, (4) libssl-dev and libffi-dev for cryptographic dependencies. When installing from pre-built wheels on common platforms (x86_64 Linux with recent pip), only python3 and python3-pip are strictly required at runtime, but the build dependencies ensure compatibility across all architectures and when building from source."
  },
  "dask[complete]": {
    "package": "dask[complete]==2025.5.1",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "gfortran",
      "libopenblas-dev",
      "liblapack-dev",
      "pkg-config"
    ],
    "description": "System dependencies required for dask[complete]==2025.5.1 pip package on Ubuntu/Debian systems",
    "notes": "Dask itself is a pure Python package for parallel computing, but dask[complete] includes many optional dependencies (numpy, pandas, scipy, scikit-learn, etc.) that require compilation. The listed dependencies are needed for building/running these numerical computation packages. For pre-built wheels (most common installation method), only python3 and python3-pip are strictly required at runtime, but the build dependencies are included for systems that need to compile from source or for architectures without wheel support. The [complete] extra installs all optional dependencies for full Dask functionality including dataframe, array, bag, delayed, and distributed computing features."
  },
  "onnx": {
    "package": "onnx",
    "apt_dependencies": [
      "python3-dev",
      "python3-pip",
      "build-essential",
      "cmake",
      "libprotobuf-dev",
      "protobuf-compiler",
      "pkg-config"
    ],
    "description": "System dependencies required for onnx>=1.20.0 pip package on Ubuntu/Debian systems",
    "notes": "ONNX (Open Neural Network Exchange) is a C++ based library with Python bindings that uses Protocol Buffers for model serialization. build-essential provides g++ and compilation tools needed for building C++ extensions. cmake (>=3.18) is required for the build system. libprotobuf-dev and protobuf-compiler are needed for Protocol Buffers support which ONNX uses extensively for its model format. python3-dev provides Python header files for building the extension modules. While pre-built wheels are available for common platforms (x86_64, ARM64), these dependencies are essential for building from source or on systems without wheel support."
  },
  "pyyaml": {
    "package": "pyyaml",
    "apt_dependencies": [
      "python3-dev",
      "python3-pip",
      "libyaml-dev"
    ],
    "description": "System dependencies required for pyyaml>=6.0 pip package on Ubuntu/Debian systems",
    "notes": "PyYAML is a YAML parser and emitter for Python. While PyYAML can work with a pure Python implementation, it uses libyaml (via the LibYAML C library) for significantly better performance when available. libyaml-dev is the development package that provides the C library and headers needed for building PyYAML with C extensions. python3-dev is required for compiling the C extension module. For pre-built wheels (available on most platforms), only python3 and python3-pip are strictly required at runtime, but libyaml-dev ensures optimal performance and is necessary when building from source."
  },
  "pyaudio": {
    "package": "pyaudio",
    "apt_dependencies": [
      "portaudio19-dev",
      "python3-pyaudio",
      "libasound2-dev",
      "libportaudio2",
      "libportaudiocpp0"
    ],
    "description": "System dependencies required for pyaudio pip package on Ubuntu/Debian systems",
    "notes": "PyAudio provides Python bindings for PortAudio, the cross-platform audio I/O library. The portaudio19-dev package is essential for building/installing pyaudio from pip. python3-pyaudio is the system package alternative. libasound2-dev provides ALSA (Advanced Linux Sound Architecture) development files which PortAudio uses as a backend on Linux systems."
  },
  "cerebras-cloud-sdk": {
    "package": "cerebras-cloud-sdk",
    "apt_dependencies": [],
    "description": "System dependencies required for cerebras-cloud-sdk pip package on Ubuntu/Debian systems",
    "notes": "The cerebras-cloud-sdk is a pure Python package that provides an API client for Cerebras Cloud services. It does not require any additional system-level apt-get packages beyond a standard Python installation. All dependencies are Python packages installed via pip."
  },
  "timm": {
    "package": "timm>=1.0.15",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "python3-dev",
      "build-essential",
      "libgomp1",
      "libopenblas-dev",
      "liblapack-dev",
      "gfortran",
      "pkg-config",
      "libjpeg-dev",
      "zlib1g-dev",
      "libpng-dev"
    ],
    "description": "System dependencies required for timm>=1.0.15 (PyTorch Image Models) pip package on Ubuntu/Debian systems",
    "notes": "timm (PyTorch Image Models) is a collection of image models, layers, utilities, optimizers, schedulers, data-loaders / augmentations, and training scripts. It depends on PyTorch and PIL/Pillow for image processing. Core dependencies include: python3-dev (Python headers for building C extensions), build-essential (gcc, g++, make for compiling native extensions), libgomp1 (OpenMP runtime for parallel processing in PyTorch), libopenblas-dev and liblapack-dev (BLAS/LAPACK for numerical operations), gfortran (Fortran compiler for scientific libraries), pkg-config (library configuration), libjpeg-dev, zlib1g-dev, and libpng-dev (image format support for Pillow/PIL). For GPU support, additional CUDA dependencies would be needed. Most users installing from pre-built wheels will primarily need python3, python3-pip, and libgomp1 at runtime, but build dependencies are included for systems requiring source compilation or lacking wheel support."
  },
  "opencv-contrib-python": {
    "package": "opencv-contrib-python",
    "version": "4.10.0.84",
    "apt_dependencies": [
      "python3-opencv",
      "libopencv-dev",
      "libgl1-mesa-glx",
      "libglib2.0-0",
      "libsm6",
      "libxext6",
      "libxrender-dev",
      "libgomp1",
      "libgstreamer1.0-0",
      "libgstreamer-plugins-base1.0-0",
      "libavcodec58",
      "libavformat58",
      "libswscale5",
      "libtbb2",
      "libtbb-dev",
      "libatlas-base-dev",
      "liblapack-dev",
      "libhdf5-dev",
      "libprotobuf-dev",
      "protobuf-compiler",
      "libgoogle-glog-dev",
      "libgflags-dev",
      "libgtk-3-dev",
      "libdc1394-dev",
      "libv4l-dev",
      "libtesseract-dev",
      "libleptonica-dev"
    ],
    "description": "System dependencies required for opencv-contrib-python pip package on Ubuntu/Debian systems",
    "notes": "opencv-contrib-python includes all opencv-python modules plus extra contributed modules. Additional dependencies beyond opencv-python include: Intel TBB for parallel processing (libtbb2, libtbb-dev), optimized linear algebra (libatlas-base-dev, liblapack-dev), HDF5 for data storage (libhdf5-dev), Protocol Buffers for DNN models (libprotobuf-dev, protobuf-compiler), logging and flags (libgoogle-glog-dev, libgflags-dev), GTK3 for GUI features (libgtk-3-dev), camera support (libdc1394-dev, libv4l-dev), and OCR capabilities (libtesseract-dev, libleptonica-dev). The exact package versions may vary by Ubuntu/Debian release."
  },
  "nltk": {
    "package": "nltk",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for nltk pip package on Ubuntu/Debian systems",
    "notes": "NLTK (Natural Language Toolkit) is a pure Python library and does not require additional system libraries beyond Python itself. However, some NLTK data packages and optional features may benefit from additional tools like java (for Stanford NLP tools), perl, or specific tokenizers, but these are not required for basic NLTK functionality. The library primarily works with downloaded corpora and models accessed through nltk.download()."
  },
  "numpy": {
    "package": "numpy",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "gfortran",
      "libopenblas-dev",
      "liblapack-dev",
      "pkg-config"
    ],
    "description": "System dependencies required for numpy>=1.26.4 pip package on Ubuntu/Debian systems",
    "notes": "NumPy requires BLAS/LAPACK libraries for numerical operations. libopenblas-dev provides optimized BLAS implementation. gfortran is needed for building from source. For pre-built wheels (most common installation method), only python3 and python3-pip are strictly required at runtime, but the build dependencies are included for systems that need to compile from source or for older architectures without wheel support."
  },
  "anthropic": {
    "package": "anthropic",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential"
    ],
    "description": "System dependencies required for anthropic>=0.19.0 pip package on Ubuntu/Debian systems",
    "notes": "The anthropic package is primarily pure Python with minimal system dependencies. The build-essential package is included to handle potential compilation of dependencies like jiter (JSON parser) which may have C extensions. For runtime-only usage without building from source, only python3 and python3-pip are strictly required."
  },
  "aiortc": {
    "package": "aiortc==1.9.0",
    "apt_dependencies": [
      "libavcodec-dev",
      "libavdevice-dev",
      "libavfilter-dev",
      "libavformat-dev",
      "libavutil-dev",
      "libswscale-dev",
      "libswresample-dev",
      "libopus-dev",
      "libvpx-dev",
      "libsrtp2-dev",
      "pkg-config",
      "python3-dev",
      "build-essential"
    ],
    "description": "System dependencies required for aiortc==1.9.0 pip package on Ubuntu/Debian systems",
    "notes": "aiortc is a Python library for WebRTC and ORTC (Object Real-Time Communication). It requires FFmpeg libraries (libav*) for media encoding/decoding, libopus for Opus audio codec support, libvpx for VP8/VP9 video codec support, and libsrtp2 for Secure Real-time Transport Protocol. The pkg-config, python3-dev, and build-essential packages are needed for building the native C extensions during pip installation."
  },
  "pyrender": {
    "package": "pyrender",
    "apt_dependencies": [
      "libosmesa6-dev",
      "libgl1-mesa-glx",
      "libgl1-mesa-dev",
      "libglu1-mesa-dev",
      "freeglut3-dev",
      "libglfw3",
      "libglfw3-dev",
      "libglew-dev",
      "libxrandr-dev",
      "libxinerama-dev",
      "libxcursor-dev",
      "libxi-dev",
      "libxxf86vm-dev"
    ],
    "description": "System dependencies required for pyrender>=0.1.45 pip package on Ubuntu/Debian systems",
    "notes": "Pyrender is a pure Python rendering library built on top of OpenGL for rendering trimeshes and point clouds. Key dependencies: libosmesa6-dev enables off-screen rendering without a display (essential for headless servers), libgl1-mesa-glx and libgl1-mesa-dev provide OpenGL support, libglu1-mesa-dev provides OpenGL utility libraries, freeglut3-dev offers window management utilities, libglfw3/libglfw3-dev provide modern OpenGL context creation and window handling, libglew-dev manages OpenGL extension loading. The X11 libraries (libxrandr-dev, libxinerama-dev, libxcursor-dev, libxi-dev, libxxf86vm-dev) support windowing and input on Linux systems. OSMesa is particularly important for rendering in Docker containers or SSH sessions without GPU access."
  },
  "ipykernel": {
    "package": "ipykernel",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "libzmq3-dev",
      "build-essential"
    ],
    "description": "System dependencies required for ipykernel>=7.1.0 pip package on Ubuntu/Debian systems",
    "notes": "ipykernel is the IPython kernel for Jupyter. The main system dependency is libzmq3-dev for the pyzmq package (ZeroMQ messaging library). Most other dependencies (debugpy, psutil, tornado, ipython, jupyter-client, etc.) are pure Python packages. build-essential is included for compiling native extensions during installation. For pre-built wheels (most common), only python3, python3-pip, and libzmq3-dev runtime libraries are strictly required."
  },
  "torchreid": {
    "package": "torchreid",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "python3-dev",
      "build-essential",
      "libgl1-mesa-glx",
      "libglib2.0-0",
      "libsm6",
      "libxext6",
      "libxrender-dev",
      "libgomp1",
      "libgfortran5",
      "libjpeg-dev",
      "libpng-dev",
      "libtiff-dev",
      "libavcodec-dev",
      "libavformat-dev",
      "libswscale-dev",
      "libv4l-dev",
      "libxvidcore-dev",
      "libx264-dev",
      "libatlas-base-dev",
      "gfortran"
    ],
    "description": "System dependencies required for torchreid==0.2.5 pip package on Ubuntu/Debian systems",
    "notes": "Torchreid (Torch Re-Identification) is a PyTorch-based deep learning library for person re-identification in computer vision. It depends on PyTorch, torchvision, and various image processing libraries. The OpenGL libraries (libgl1-mesa-glx, libglib2.0-0, libsm6, libxext6, libxrender-dev) are required for GUI and visualization features. Image codec libraries (libjpeg-dev, libpng-dev, libtiff-dev) support various image formats. Video codec libraries (libavcodec-dev, libavformat-dev, libswscale-dev, libv4l-dev, libxvidcore-dev, libx264-dev) enable video processing capabilities. Math libraries (libatlas-base-dev, gfortran, libgfortran5, libgomp1) provide optimized numerical operations for PyTorch. Build tools (build-essential, python3-dev) are needed for compiling extensions and dependencies."
  },
  "openai": {
    "package": "openai",
    "apt_dependencies": [
      "python3-dev",
      "build-essential",
      "libssl-dev",
      "libffi-dev",
      "ca-certificates"
    ],
    "description": "System dependencies required for openai pip package on Ubuntu/Debian systems",
    "notes": "The openai package is primarily pure Python with HTTP/HTTPS capabilities. Core dependencies include SSL/TLS support for secure API communication and build tools for compiling optional native extensions in dependencies like pydantic and jiter. Additional packages may be needed for optional extras: libasound2-dev and libportaudio2 for voice-helpers (sounddevice), python3-numpy for datalib/voice features."
  },
  "ruff": {
    "package": "ruff==0.14.3",
    "apt_dependencies": [],
    "description": "System dependencies required for ruff==0.14.3 pip package on Ubuntu/Debian systems",
    "notes": "Ruff is an extremely fast Python linter and code formatter written in Rust. It is distributed as pre-compiled platform-specific wheels that contain statically-linked binaries. Ruff has no system library dependencies or build requirements - it only requires Python itself to be installed. The wheel includes all necessary compiled code and runs out of the box without any additional apt packages."
  },
  "ctransformers": {
    "package": "ctransformers",
    "version": "0.2.27",
    "apt_dependencies": [
      "python3-dev",
      "python3-pip",
      "build-essential",
      "gcc",
      "g++",
      "make",
      "cmake",
      "libstdc++6"
    ],
    "description": "System dependencies required for ctransformers==0.2.27 pip package on Ubuntu/Debian systems",
    "notes": "ctransformers is a Python library providing bindings for transformer models using GGML/GGUF format (similar to llama.cpp). It requires C/C++ compilation toolchain as it includes native extensions. build-essential provides essential compilation tools (gcc, g++, make). cmake is needed for the build process. python3-dev provides Python header files needed to compile C extensions. libstdc++6 provides the standard C++ library runtime. While pre-built wheels may be available for some platforms, these dependencies ensure the package can be built from source if needed."
  },
  "types-PySocks": {
    "package": "types-PySocks",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for types-PySocks>=1.7.1.20251001,<2 pip package on Ubuntu/Debian systems",
    "notes": "The types-PySocks package is a type stub package that provides type information for PySocks. It is a pure Python package with no compiled components or external system dependencies beyond Python itself. No build tools or additional libraries are required."
  },
  "types-gevent": {
    "package": "types-gevent>=25.4.0.20250915,<26",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for types-gevent>=25.4.0.20250915,<26 pip package on Ubuntu/Debian systems",
    "notes": "types-gevent is a type stubs package that provides type hints for the gevent library. It is used for static type checking with mypy, pyright, and other type checkers. As a pure Python package containing only type stub files (.pyi), it has no compiled components or system library dependencies beyond Python itself. The package is installed at development/type-checking time and has no runtime dependencies. Note that this package only provides type information and does not include the actual gevent implementation - you must install gevent separately for runtime async I/O functionality. While gevent itself requires system libraries like libevent and greenlet (with C extensions), types-gevent as a stub-only package does not inherit these requirements."
  },
  "open_clip_torch": {
    "package": "open_clip_torch",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "python3-dev",
      "build-essential",
      "git",
      "libjpeg-dev",
      "libjpeg8-dev",
      "libpng-dev",
      "zlib1g-dev",
      "libtiff-dev",
      "libfreetype6-dev",
      "liblcms2-dev",
      "libwebp-dev",
      "libharfbuzz-dev",
      "libfribidi-dev",
      "libopenjp2-7-dev",
      "libxcb1-dev"
    ],
    "description": "System dependencies required for open_clip_torch==3.2.0 pip package on Ubuntu/Debian systems",
    "notes": "open_clip_torch is an open source implementation of OpenAI's CLIP (Contrastive Language-Image Pre-training). It requires PyTorch and torchvision as Python dependencies (installed via pip). The apt packages listed here are primarily for: (1) build-essential and python3-dev for compiling native extensions, (2) image processing libraries (libjpeg, libpng, libtiff, libwebp, etc.) required by Pillow which is a core dependency for image loading and preprocessing, and (3) font rendering libraries (libfreetype6, libharfbuzz, libfribidi) for text processing capabilities. For GPU support, CUDA libraries are required but are typically handled separately through PyTorch's CUDA-enabled wheel or system CUDA installation. Git may be needed for some dependency installations."
  },
  "asyncio": {
    "package": "asyncio==3.4.3",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for asyncio==3.4.3 pip package on Ubuntu/Debian systems",
    "notes": "The asyncio==3.4.3 package is a backport of Python's asyncio library for Python 3.3. Starting from Python 3.4+, asyncio is included in the standard library and this pip package is not needed. This is a pure Python package with no C extensions or additional system-level dependencies beyond Python itself and pip. For modern Python versions (3.4+), asyncio is available as a built-in module without requiring pip installation."
  },
  "clip": {
    "package": "clip",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "git",
      "libjpeg-dev",
      "libpng-dev",
      "zlib1g-dev",
      "libfreetype6-dev",
      "liblcms2-dev",
      "libopenjp2-7-dev",
      "libtiff-dev",
      "libwebp-dev",
      "libharfbuzz-dev",
      "libfribidi-dev",
      "libxcb1-dev"
    ],
    "description": "System dependencies required for OpenAI CLIP (openai-clip) pip package on Ubuntu/Debian systems",
    "notes": "CLIP (Contrastive Language-Image Pre-Training) requires PyTorch and Pillow as main Python dependencies. The apt packages listed here are primarily for Pillow's image processing capabilities and build tools. For GPU support, additional CUDA-related packages would be needed (these are typically handled separately through PyTorch installation). The git package is required because the official installation is done via 'pip install git+https://github.com/openai/CLIP.git'."
  },
  "rtree": {
    "package": "rtree",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "libspatialindex-dev",
      "libspatialindex6"
    ],
    "description": "System dependencies required for rtree>=0.9.0 pip package on Ubuntu/Debian systems",
    "notes": "Rtree is a Python wrapper for libspatialindex, a C++ library for spatial indexing of points and rectangles. libspatialindex-dev provides the development headers and static libraries needed to build the rtree Python bindings. libspatialindex6 (or libspatialindex-c6 on some systems) provides the runtime shared library. The rtree package uses ctypes to interface with libspatialindex at runtime, so both the development and runtime libraries are beneficial. For pre-built wheels on common architectures, only python3 and python3-pip may be sufficient, but the libspatialindex packages ensure full functionality and enable building from source when needed."
  },
  "ffmpeg-python": {
    "package": "ffmpeg-python",
    "apt_dependencies": [
      "ffmpeg",
      "libavcodec-dev",
      "libavformat-dev",
      "libavutil-dev",
      "libswscale-dev",
      "libswresample-dev",
      "libavfilter-dev",
      "libavdevice-dev"
    ],
    "description": "System dependencies required for ffmpeg-python pip package on Ubuntu/Debian systems",
    "notes": "ffmpeg-python is a Python wrapper for FFmpeg. The main runtime dependency is the 'ffmpeg' package which provides the ffmpeg command-line tool. The lib*-dev packages are development libraries that may be needed for compilation or advanced use cases. At minimum, 'ffmpeg' must be installed for ffmpeg-python to work."
  },
  "h5py": {
    "package": "h5py",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "libhdf5-dev",
      "libhdf5-serial-dev",
      "pkg-config"
    ],
    "description": "System dependencies required for h5py>=3.7.0 pip package on Ubuntu/Debian systems",
    "notes": "h5py is a Python interface to the HDF5 library and requires HDF5 development headers and libraries. libhdf5-dev provides the core HDF5 C library development files. libhdf5-serial-dev provides the serial (non-MPI) version of HDF5. pkg-config is used to locate the HDF5 libraries during installation. For pre-built wheels (most common installation method on x86_64), only python3 and python3-pip are required at runtime, but build dependencies are included for systems that need to compile from source or for architectures without wheel support."
  },
  "pydantic": {
    "package": "pydantic",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for pydantic pip package on Ubuntu/Debian systems",
    "notes": "Pydantic is primarily a pure Python package with pydantic-core (Rust-based) as its main dependency. Pre-built binary wheels are available for most platforms and Python versions, so typically only python3 and python3-pip are required. If building from source is needed (rare cases like unsupported architectures), additional dependencies would include: build-essential, rustc, cargo (Rust toolchain). However, for standard installations using pip with wheel support, no additional system packages are required beyond the Python runtime."
  },
  "types-greenlet": {
    "package": "types-greenlet>=3.2.0.20250915,<4",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for types-greenlet>=3.2.0.20250915,<4 pip package on Ubuntu/Debian systems",
    "notes": "types-greenlet is a type stubs package that provides type hints for the greenlet library. It is used for static type checking with mypy, pyright, and other type checkers. As a pure Python package containing only type stub files (.pyi), it has no compiled components or system library dependencies beyond Python itself. The package is installed at development/type-checking time and has no runtime dependencies. Note that this package only provides type information and does not include the actual greenlet implementation - you must install greenlet separately for runtime greenlet functionality."
  },
  "catkin_pkg": {
    "package": "catkin_pkg",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for catkin_pkg pip package on Ubuntu/Debian systems",
    "notes": "catkin_pkg is a pure Python package used in ROS for handling catkin workspace packages. It has minimal system dependencies - only requiring Python 3 and pip. The package itself depends on other Python packages (python-dateutil, pyparsing, docutils) which are installed via pip and don't require additional system libraries. While catkin_pkg is commonly used in ROS environments, the pip package itself doesn't require ROS system packages to be installed."
  },
  "lvis": {
    "package": "lvis",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "python3-dev",
      "build-essential",
      "cython3",
      "pkg-config",
      "gfortran",
      "libopenblas-dev",
      "liblapack-dev",
      "libfreetype6-dev",
      "libpng-dev",
      "libjpeg-dev",
      "libjpeg8-dev",
      "zlib1g-dev",
      "libqhull-dev",
      "libfontconfig1-dev",
      "libxft-dev",
      "libxcb-render0-dev",
      "libxcb-shm0-dev",
      "libxcb1-dev",
      "libxrender-dev",
      "libxext-dev",
      "libx11-dev",
      "tk-dev",
      "tcl-dev",
      "libgtk-3-dev",
      "libcairo2-dev",
      "libgirepository1.0-dev",
      "gir1.2-gtk-3.0",
      "python3-opencv",
      "libopencv-dev",
      "libgl1-mesa-glx",
      "libglib2.0-0",
      "libsm6",
      "libgomp1",
      "libgstreamer1.0-0",
      "libgstreamer-plugins-base1.0-0",
      "libavcodec58",
      "libavformat58",
      "libswscale5"
    ],
    "description": "System dependencies required for lvis (LVIS Dataset API) pip package on Ubuntu/Debian systems",
    "notes": "LVIS (pronounced 'el-vis') is the Python API for the Large Vocabulary Instance Segmentation dataset. The package requires Cython for building native extensions, NumPy for numerical operations (requiring BLAS/LAPACK via libopenblas-dev), matplotlib for visualization (requiring FreeType, PNG, JPEG support and X11/GTK3 libraries for interactive backends), and opencv-python for computer vision operations (requiring OpenCV libraries, GStreamer for video I/O, and ffmpeg libraries). The cython3 package provides the Cython compiler needed during installation. While pre-built wheels may be available for common platforms, these dependencies ensure the package can be built from source and all features function correctly. Version compatibility note: libavcodec58, libavformat58, and libswscale5 are for Ubuntu 20.04/Debian 11; newer versions may use libavcodec59/60, libavformat59/60, and libswscale6/7."
  },
  "pyquaternion": {
    "package": "pyquaternion",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for pyquaternion>=0.9.9 pip package on Ubuntu/Debian systems",
    "notes": "pyquaternion is a pure Python library for quaternion math that depends only on numpy. It requires no additional system libraries beyond basic Python. All mathematical operations are handled by numpy, so only python3 and python3-pip are needed. The package uses numpy's array operations and does not have any compiled extensions or external library dependencies."
  },
  "xformers": {
    "package": "xformers",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "gcc",
      "g++",
      "make",
      "cmake",
      "ninja-build",
      "nvidia-cuda-toolkit",
      "libcudnn8",
      "libcudnn8-dev",
      "git"
    ],
    "description": "System dependencies required for xformers>=0.0.20 pip package on Ubuntu/Debian systems",
    "notes": "xformers is a PyTorch extension library for memory-efficient attention mechanisms. It requires CUDA toolkit (nvidia-cuda-toolkit) for GPU support and cuDNN (libcudnn8) for optimized deep learning operations. Build tools (gcc, g++, make, cmake, ninja-build) are essential for compiling the C++/CUDA extensions that xformers provides. The package requires PyTorch to be installed first with matching CUDA version. For CUDA 11.x systems, use nvidia-cuda-toolkit; for CUDA 12.x, package names may vary (e.g., nvidia-cuda-toolkit-12-x). git is required for potential source builds. Note: Pre-built wheels are available for common platforms (Linux x86_64 with CUDA 11.8/12.1), but compilation dependencies are needed for other configurations or building from source."
  },
  "python-fcl": {
    "package": "python-fcl",
    "version": ">=0.7.0.4",
    "apt_dependencies": [
      "libfcl-dev",
      "liboctomap-dev",
      "libeigen3-dev",
      "libccd-dev",
      "build-essential",
      "cmake",
      "pkg-config",
      "python3-dev",
      "libassimp-dev"
    ],
    "description": "System dependencies required for python-fcl>=0.7.0.4 pip package on Ubuntu/Debian systems",
    "notes": "python-fcl is a Python binding for the Flexible Collision Library (FCL), used for performing collision detection and proximity queries. Key dependencies: libfcl-dev provides the core FCL library for collision detection, liboctomap-dev enables octree-based collision checking for 3D environments, libeigen3-dev provides linear algebra operations used throughout FCL, libccd-dev implements the GJK-based continuous collision detection algorithm, build-essential and cmake are required for compiling the Python bindings from source, pkg-config helps locate library dependencies during compilation, python3-dev provides Python headers needed for building C++ extensions, and libassimp-dev enables loading various 3D model formats for collision meshes. Note that python-fcl typically needs to be built from source, so all build dependencies are essential."
  },
  "nvidia-nvimgcodec-cu12[all]": {
    "package": "nvidia-nvimgcodec-cu12[all]",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "libgomp1",
      "libjpeg-dev",
      "libjpeg8-dev",
      "libpng-dev",
      "libtiff-dev",
      "libwebp-dev"
    ],
    "description": "System dependencies required for nvidia-nvimgcodec-cu12[all] pip package on Ubuntu/Debian systems",
    "notes": "nvidia-nvimgcodec-cu12 is NVIDIA's nvImageCodec library providing GPU-accelerated image decoding and encoding for CUDA 12. The package comes as a binary wheel with bundled CUDA libraries, so CUDA installation is not required as a system dependency. The [all] extra includes support for all available image codecs (JPEG, PNG, TIFF, WebP, etc.). libgomp1 is required for OpenMP support. The image format libraries (libjpeg-dev, libpng-dev, libtiff-dev, libwebp-dev) provide codec support at the system level. For basic functionality with pre-built wheels, only python3, python3-pip, and libgomp1 are strictly necessary, but the additional libraries enable full codec support."
  },
  "pycryptodome": {
    "package": "pycryptodome",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "python3-dev"
    ],
    "description": "System dependencies required for pycryptodome pip package on Ubuntu/Debian systems",
    "notes": "PyCryptodome is a self-contained cryptographic library. For most installations using pre-built wheels, only python3 and python3-pip are required. build-essential (gcc, make) and python3-dev are needed when building from source or when pre-built wheels are unavailable for the platform. The library includes its own C implementations and does not depend on external cryptographic libraries like OpenSSL."
  },
  "pydantic-settings": {
    "package": "pydantic-settings",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for pydantic-settings pip package on Ubuntu/Debian systems",
    "notes": "pydantic-settings is a pure Python package that extends pydantic for settings management. It relies on pydantic (which includes pydantic-core with Rust-based components) but pre-built binary wheels are available for most platforms and Python versions. For standard installations using pip with wheel support, only python3 and python3-pip are required. If building from source is needed (rare cases like unsupported architectures), additional dependencies would include: build-essential, rustc, cargo (Rust toolchain) - same as pydantic. However, for typical installations, no additional system packages are required beyond the Python runtime."
  },
  "openai-whisper": {
    "package": "openai-whisper",
    "version": "latest",
    "apt_dependencies": [
      "ffmpeg"
    ],
    "description": "OpenAI Whisper is a speech recognition model that requires FFmpeg for audio file processing and format conversion"
  },
  "soundfile": {
    "package": "soundfile",
    "apt_dependencies": [
      "libsndfile1",
      "libsndfile1-dev"
    ],
    "description": "System dependencies required for soundfile pip package on Ubuntu/Debian systems",
    "notes": "soundfile is a Python wrapper for libsndfile, a C library for reading and writing audio files in various formats (WAV, FLAC, OGG, etc.). libsndfile1 provides the runtime library required for the package to function. libsndfile1-dev contains the development headers needed for building the Python bindings during pip installation."
  },
  "opencv-python": {
    "package": "opencv-python",
    "apt_dependencies": [
      "python3-opencv",
      "libopencv-dev",
      "libgl1-mesa-glx",
      "libglib2.0-0",
      "libsm6",
      "libxext6",
      "libxrender-dev",
      "libgomp1",
      "libgstreamer1.0-0",
      "libgstreamer-plugins-base1.0-0",
      "libavcodec58",
      "libavformat58",
      "libswscale5"
    ],
    "description": "System dependencies required for opencv-python pip package on Ubuntu/Debian systems",
    "notes": "These are the core system libraries that opencv-python needs to function properly. The exact versions may vary by Ubuntu/Debian release."
  },
  "pytest-asyncio": {
    "package": "pytest-asyncio==0.26.0",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for pytest-asyncio==0.26.0 pip package on Ubuntu/Debian systems",
    "notes": "pytest-asyncio is a pure Python plugin for pytest that provides asyncio support for testing async Python code. It has no C extensions or special system-level dependencies beyond Python itself and pip. The package requires pytest>=8.2 and Python>=3.8 as Python dependencies, but these don't introduce additional apt-get requirements. No build tools are needed as it installs from pure Python wheels."
  },
  "ollama": {
    "package": "ollama",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "curl"
    ],
    "description": "System dependencies required for ollama>=0.6.0 pip package on Ubuntu/Debian systems",
    "notes": "The ollama Python package is a pure Python client library with minimal system dependencies. It only requires Python and pip to install. The 'curl' package is included as it's commonly used to install and interact with the Ollama server itself (which is separate from the Python package). Note: The ollama pip package is just a client - to actually use it, you need the Ollama server running separately, which can be installed via: curl -fsSL https://ollama.com/install.sh | sh"
  },
  "trimesh": {
    "package": "trimesh",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "libspatialindex-dev",
      "libgeos-dev",
      "libgeos++-dev"
    ],
    "description": "System dependencies required for trimesh>=3.22.0 pip package on Ubuntu/Debian systems",
    "notes": "Trimesh is a pure Python library for loading and using triangular meshes. While trimesh itself is pure Python and has minimal system dependencies, some of its optional features require system libraries. libspatialindex-dev is needed for rtree (spatial indexing), and libgeos-dev/libgeos++-dev are needed for shapely (2D geometry operations). The core trimesh functionality works with just Python, but these dependencies enable the full feature set. For minimal installations, only python3 and python3-pip are strictly required."
  },
  "types-jmespath": {
    "package": "types-jmespath",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for types-jmespath>=1.0.2.20250809,<2 pip package on Ubuntu/Debian systems",
    "notes": "The types-jmespath package is a pure Python type stub package from the typeshed project. It contains only type hint files (.pyi) with no compiled extensions or runtime code. Only python3 and python3-pip are required for installation."
  },
  "typeguard": {
    "package": "typeguard",
    "apt_dependencies": [],
    "description": "System dependencies required for typeguard pip package on Ubuntu/Debian systems",
    "notes": "typeguard is a pure Python package that provides runtime type checking for Python type hints. It has no native code or system library dependencies. Its only dependencies are typing_extensions (pure Python) and importlib_metadata for Python < 3.10 (also pure Python). Only Python itself needs to be installed."
  },
  "pandas": {
    "package": "pandas",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "gfortran",
      "libopenblas-dev",
      "liblapack-dev",
      "pkg-config"
    ],
    "description": "System dependencies required for pandas>=1.5.2 pip package on Ubuntu/Debian systems",
    "notes": "Pandas requires NumPy as a core dependency, which needs BLAS/LAPACK libraries for numerical operations. libopenblas-dev provides optimized BLAS implementation. gfortran is needed for building from source. For pre-built wheels (most common installation method), only python3 and python3-pip are strictly required at runtime, but the build dependencies are included for systems that need to compile from source or for older architectures without wheel support. Pandas 1.5.2+ also benefits from optional dependencies like pyarrow for parquet file support and openpyxl for Excel file support, though these are handled at the pip level."
  },
  "tensorboard": {
    "package": "tensorboard==2.20.0",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for tensorboard==2.20.0 pip package on Ubuntu/Debian systems",
    "notes": "TensorBoard 2.20.0 is a pure Python web application for visualizing TensorFlow/PyTorch training metrics. It does not require additional system-level dependencies beyond Python 3.9+ and pip. All of TensorBoard's dependencies (including werkzeug, protobuf, grpcio, markdown, numpy, setuptools, wheel, absl-py, google-auth, google-auth-oauthlib, requests) are available as pip packages. The web server runs on Python's built-in networking capabilities. For the frontend, static assets are bundled with the package. If using TensorBoard with GPU-accelerated frameworks, those frameworks (TensorFlow, PyTorch) will have their own GPU dependencies (CUDA, cuDNN), but TensorBoard itself only needs Python runtime."
  },
  "llvmlite": {
    "package": "llvmlite>=0.42.0",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "llvm-14-dev",
      "libedit-dev",
      "libxml2-dev",
      "zlib1g-dev"
    ],
    "description": "System dependencies required for llvmlite>=0.42.0 pip package on Ubuntu/Debian systems",
    "notes": "llvmlite requires LLVM development libraries to build and run. Version 0.42.0 typically supports LLVM 11-14. llvm-14-dev is recommended for best compatibility. libedit-dev and libxml2-dev are required by LLVM. build-essential provides gcc/g++ needed for compilation. For pre-built wheels, some of these may not be strictly required at runtime, but they are needed for systems that compile from source or lack wheel support."
  },
  "lap": {
    "package": "lap",
    "apt_dependencies": [
      "python3-dev",
      "python3-pip",
      "build-essential",
      "gcc",
      "g++",
      "cython3"
    ],
    "description": "System dependencies required for lap>=0.5.12 pip package on Ubuntu/Debian systems",
    "notes": "LAP (Linear Assignment Problem) is a library for solving assignment problems. It requires Cython and C++ compilation tools to build the native extensions. python3-dev provides Python header files needed for building C extensions. build-essential, gcc, and g++ provide the C/C++ compiler toolchain. cython3 is required to compile the Cython source files. NumPy is a Python dependency that will be installed via pip. For pre-built wheels on common platforms (x86_64 Linux with glibc), only python3 and python3-pip may be needed at runtime, but build dependencies are included for source installation or platforms without wheel support."
  },
  "plum-dispatch": {
    "package": "plum-dispatch",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for plum-dispatch==2.5.7 pip package on Ubuntu/Debian systems",
    "notes": "plum-dispatch is a pure Python package that implements multiple dispatch (function overloading). It has no system-level dependencies beyond Python itself. The package only requires Python 3.8 or higher and has minimal Python dependencies (beartype>=0.11 for runtime type checking). No C extensions or external libraries are needed."
  },
  "types-defusedxml": {
    "package": "types-defusedxml",
    "version_spec": ">=0.7.0.20250822,<1",
    "apt_dependencies": []
  },
  "pytest-mock": {
    "package": "pytest-mock==3.15.0",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for pytest-mock==3.15.0 pip package on Ubuntu/Debian systems",
    "notes": "pytest-mock is a pure Python plugin that provides a mocker fixture for pytest. It wraps the mock package from the standard library (unittest.mock). As a pure Python package with only Python dependencies (pytest>=6.2.5), it does not require any additional system-level apt-get packages beyond Python itself and pip. The package is compatible with Python 3.8+."
  },
  "plotext": {
    "package": "plotext",
    "version": "5.3.2",
    "apt_dependencies": [],
    "description": "System dependencies required for plotext==5.3.2 pip package on Ubuntu/Debian systems",
    "notes": "plotext is a pure Python terminal plotting library with no native code or system library dependencies. It works entirely through terminal text output and ANSI escape codes. The package only requires Python itself and standard library modules to function. No additional apt packages are needed beyond python3 and python3-pip for the base Python environment."
  },
  "langchain-text-splitters": {
    "package": "langchain-text-splitters>=1,<2",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential"
    ],
    "description": "System dependencies required for langchain-text-splitters>=1,<2 pip package on Ubuntu/Debian systems",
    "notes": "langchain-text-splitters is a pure Python package for text splitting and chunking utilities. It has minimal system dependencies. The package primarily depends on langchain-core and standard Python libraries. build-essential is included for potential compilation of dependencies like pydantic-core and other compiled extensions that may be required by its dependencies. For most installations using pre-built wheels, only python3 and python3-pip are strictly required."
  },
  "langchain-chroma": {
    "package": "langchain-chroma",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "gcc",
      "g++",
      "cmake",
      "python3-dev",
      "libopenblas-dev",
      "liblapack-dev",
      "pkg-config",
      "curl"
    ],
    "description": "System dependencies required for langchain-chroma>=1,<2 pip package on Ubuntu/Debian systems",
    "notes": "langchain-chroma depends on chromadb which requires several native dependencies. The main requirements are: (1) build-essential, gcc, g++, and cmake for compiling native extensions including chroma-hnswlib (HNSW library for vector similarity search) and potentially onnxruntime components; (2) python3-dev for Python C extension headers; (3) libopenblas-dev and liblapack-dev for numpy numerical operations (inherited from numpy>=1.26.0 dependency); (4) pkg-config for build configuration; (5) curl for potential runtime operations. For pre-built wheels on common architectures (x86_64, aarch64), only python3, python3-pip, and runtime libraries may be strictly required, but build dependencies are included to support source compilation on systems without wheel support or for custom builds."
  },
  "typer": {
    "package": "typer",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for typer>=0.19.2,<1 pip package on Ubuntu/Debian systems",
    "notes": "Typer is a pure Python CLI library built on top of Click and Pydantic. It has no native code or system library dependencies beyond Python itself. The package only requires Python 3.6+ and pip for installation. All its dependencies (click, typing-extensions, etc.) are also pure Python packages available as wheels, so no compilation or additional system libraries are needed."
  },
  "Flask": {
    "package": "Flask>=2.2",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for Flask>=2.2 pip package on Ubuntu/Debian systems",
    "notes": "Flask is a pure Python web framework and does not require additional system-level dependencies beyond Python itself. All of Flask's dependencies (Werkzeug, Jinja2, MarkupSafe, ItsDangerous, Click) are also pure Python packages available via pip. For production deployments with WSGI servers like gunicorn or uWSGI, those packages may have their own system dependencies, but Flask itself only requires Python 3.7+ and pip."
  },
  "mypy": {
    "package": "mypy==1.19.0",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential"
    ],
    "description": "System dependencies required for mypy==1.19.0 pip package on Ubuntu/Debian systems",
    "notes": "Mypy is a static type checker for Python that is primarily pure Python code. The package includes librt (mypyc runtime library) which is distributed as compiled wheels for common platforms. The build-essential package is included to handle potential compilation scenarios on architectures without pre-built wheels, though this is rarely needed. For typical installations using pre-built wheels (most common), only python3 and python3-pip are strictly required. Mypy requires Python >=3.9."
  },
  "detectron2": {
    "package": "detectron2",
    "apt_dependencies": [
      "build-essential",
      "g++",
      "gcc",
      "python3-dev",
      "python3-pip",
      "git",
      "ninja-build",
      "python3-opencv",
      "libopencv-dev",
      "ca-certificates",
      "wget",
      "cmake"
    ],
    "description": "System dependencies required for detectron2 (Facebook AI Research's object detection library) on Ubuntu/Debian systems",
    "notes": "detectron2 requires gcc & g++ ≥ 5.4 for compilation. The package must be built from source as it uses C++ extensions with CUDA/PyTorch. build-essential provides essential compilation tools. python3-dev provides Python headers needed for building C++ extensions. ninja-build is optional but recommended for faster builds. python3-opencv and libopencv-dev provide OpenCV which is optional but needed for demos and visualization. ca-certificates is needed for secure downloads. cmake is required as a build dependency (version from pip is preferred over apt as the apt version may be too old). PyTorch and torchvision must be installed separately and match versions. For GPU support, CUDA toolkit and appropriate NVIDIA drivers are required but are typically installed separately."
  },
  "unitree-webrtc-connect": {
    "package": "unitree-webrtc-connect",
    "apt_dependencies": [
      "portaudio19-dev",
      "libavcodec-dev",
      "libavformat-dev",
      "libavutil-dev",
      "libavdevice-dev",
      "libavfilter-dev",
      "libswscale-dev",
      "libswresample-dev",
      "libvpx-dev",
      "libopus-dev",
      "libsrtp2-dev",
      "liblz4-dev",
      "ffmpeg",
      "python3-dev",
      "build-essential",
      "pkg-config"
    ],
    "description": "System dependencies required for unitree-webrtc-connect pip package on Ubuntu/Debian systems",
    "notes": "This package is used to connect to Unitree Go2 and G1 robots via WebRTC. The main dependencies are: portaudio19-dev (for pyaudio and sounddevice audio I/O), various libav* packages (for aiortc WebRTC library), libvpx-dev and libopus-dev (for video/audio codecs), libsrtp2-dev (for secure RTP), liblz4-dev (for LZ4 compression), and ffmpeg (for pydub audio processing). The build-essential and python3-dev packages are needed for building native extensions during pip installation."
  },
  "tqdm": {
    "package": "tqdm",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for tqdm>=4.65.0 pip package on Ubuntu/Debian systems",
    "notes": "tqdm is a pure Python package for creating progress bars. It has no compiled extensions and requires no additional system dependencies beyond Python itself. The package works with the Python standard library and has optional dependencies for specific features (like colorama for Windows), but these are also pure Python packages. No C libraries or build tools are needed."
  },
  "kaleido": {
    "package": "kaleido",
    "apt_dependencies": [
      "python3-dev",
      "python3-pip",
      "libglib2.0-0",
      "libnss3",
      "libnspr4",
      "libatk1.0-0",
      "libatk-bridge2.0-0",
      "libcups2",
      "libdrm2",
      "libdbus-1-3",
      "libxkbcommon0",
      "libxcomposite1",
      "libxdamage1",
      "libxfixes3",
      "libxrandr2",
      "libgbm1",
      "libpango-1.0-0",
      "libcairo2",
      "libasound2",
      "libatspi2.0-0"
    ],
    "description": "System dependencies required for kaleido>=0.2.1 pip package on Ubuntu/Debian systems",
    "notes": "Kaleido is a cross-platform library for generating static images of plotly charts. It bundles a Chromium-based rendering engine and has minimal external dependencies. The listed dependencies are primarily for the bundled Chromium runtime: libglib2.0-0 for GLib support, libnss3/libnspr4 for network security, libatk*/libatspi2.0-0 for accessibility, libcups2 for printing support, libdrm2/libgbm1 for GPU access, X11 libraries (libxkbcommon0, libxcomposite1, libxdamage1, libxfixes3, libxrandr2) for display management, libpango-1.0-0/libcairo2 for text and graphics rendering, libdbus-1-3 for inter-process communication, and libasound2 for audio support. Pre-built wheels include the rendering engine, so these are runtime dependencies for the embedded Chromium to function properly. Works with plotly>=4.0"
  },
  "transformers[torch]": {
    "package": "transformers[torch]==4.49.0",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "git",
      "libgomp1",
      "libopenblas-dev",
      "liblapack-dev",
      "gfortran",
      "pkg-config",
      "python3-dev",
      "libffi-dev",
      "libssl-dev"
    ],
    "description": "System dependencies required for transformers[torch]==4.49.0 pip package on Ubuntu/Debian systems",
    "notes": "The transformers library with torch backend requires PyTorch and its dependencies. Core dependencies include: build-essential (gcc, g++, make for compiling native extensions), git (for downloading models from HuggingFace Hub), libgomp1 (OpenMP runtime for parallel processing), libopenblas-dev and liblapack-dev (BLAS/LAPACK for numerical operations in PyTorch), gfortran (Fortran compiler for scientific libraries), pkg-config (for library configuration), python3-dev (Python headers for building C extensions), libffi-dev (foreign function interface library), and libssl-dev (SSL/TLS support for secure downloads). For GPU support, additional CUDA dependencies would be needed (nvidia-cuda-toolkit, libcudnn8, etc.). Most users installing from pre-built wheels only need python3, python3-pip, git, and libgomp1 at runtime, but build dependencies are included for systems requiring source compilation."
  },
  "mss": {
    "package": "mss",
    "apt_dependencies": [
      "python3-dev",
      "python3-pip",
      "libx11-6",
      "libxext6",
      "libxrandr2",
      "libxinerama1",
      "libxfixes3"
    ],
    "description": "System dependencies required for mss (Multiple Screen Shots) pip package on Ubuntu/Debian systems",
    "notes": "mss is a pure Python library for cross-platform screenshot capture. On Linux, it uses the X11 protocol to capture screens. The X11 libraries (libx11-6, libxext6, libxrandr2, libxinerama1, libxfixes3) provide the necessary system-level screen capture capabilities. For Wayland systems, additional dependencies may be required. The package works with pre-built wheels on most systems, but these dependencies ensure proper runtime functionality for screen capture operations."
  },
  "types-protobuf": {
    "package": "types-protobuf>=6.32.1.20250918,<7",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for types-protobuf>=6.32.1.20250918,<7 pip package on Ubuntu/Debian systems",
    "notes": "types-protobuf is a type stubs package that provides type hints for the protobuf library. It is used for static type checking with mypy, pyright, and other type checkers. As a pure Python package containing only type stub files (.pyi), it has no compiled components or system library dependencies beyond Python itself. The package is installed at development/type-checking time and has no runtime dependencies. Note that this package only provides type information and does not include the actual protobuf implementation - you must install protobuf separately for runtime Protocol Buffers functionality."
  },
  "requests-mock": {
    "package": "requests-mock==1.12.1",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for requests-mock==1.12.1 pip package on Ubuntu/Debian systems",
    "notes": "requests-mock is a pure Python library that provides a mock/stub implementation of the requests library for testing purposes. It has no compiled extensions or system-level dependencies beyond Python itself. The package depends on the 'requests' library (also pure Python) and 'six' for Python 2/3 compatibility. No additional apt packages are required beyond python3 and python3-pip."
  },
  "googlemaps": {
    "package": "googlemaps",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "ca-certificates",
      "libssl3",
      "openssl"
    ],
    "description": "System dependencies required for googlemaps>=4.10.0 pip package on Ubuntu/Debian systems",
    "notes": "The googlemaps package is a pure Python library that depends on requests>=2.20.0,<3.0 for making HTTP/HTTPS API calls to Google Maps services. It requires SSL/TLS support for secure HTTPS connections. ca-certificates provides trusted certificate authorities for SSL verification. libssl3 and openssl provide the SSL/TLS implementation required by the underlying requests library. For most modern Python installations with pip, googlemaps can be installed with just python3 and python3-pip, but the SSL libraries are essential for secure HTTPS communications with Google Maps APIs."
  },
  "ultralytics": {
    "package": "ultralytics",
    "version": ">=8.3.70",
    "apt_dependencies": [
      "libgl1-mesa-glx",
      "libglib2.0-0",
      "libsm6",
      "libxext6",
      "libxrender-dev",
      "libgomp1",
      "libgstreamer1.0-0",
      "libgstreamer-plugins-base1.0-0",
      "libavcodec-dev",
      "libavformat-dev",
      "libswscale-dev",
      "python3-opencv",
      "libopencv-dev",
      "ffmpeg",
      "libfreetype6-dev",
      "libpng-dev",
      "libjpeg-dev",
      "libffi-dev",
      "git"
    ],
    "description": "System dependencies required for ultralytics>=8.3.70 pip package on Ubuntu/Debian systems",
    "notes": "Ultralytics (YOLOv8) requires OpenCV for image/video processing, FFmpeg for video handling, and various system libraries for GUI rendering and media codec support. The package also depends on PyTorch, which may require additional CUDA libraries for GPU support (those are separate from these apt dependencies). Git is required for downloading model weights and some dependencies."
  },
  "types-PyYAML": {
    "package": "types-PyYAML>=6.0.12.20250915,<7",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for types-PyYAML>=6.0.12.20250915,<7 pip package on Ubuntu/Debian systems",
    "notes": "types-PyYAML is a type stubs package that provides type hints for the PyYAML library. It is used for static type checking with mypy, pyright, and other type checkers. As a pure Python package containing only type stub files (.pyi), it has no compiled components or system library dependencies beyond Python itself. The package is installed at development/type-checking time and has no runtime dependencies. Note that this package only provides type information and does not include the actual PyYAML implementation - you must install pyyaml separately for runtime YAML parsing functionality."
  },
  "lxml-stubs": {
    "package": "lxml-stubs>=0.5.1,<1",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for lxml-stubs>=0.5.1,<1 pip package on Ubuntu/Debian systems",
    "notes": "lxml-stubs is a pure Python package containing only PEP 484 type stubs (.pyi files) for the lxml library. It has no compiled code and requires no system-level dependencies beyond Python itself. The package provides type annotations for static type checkers like mypy but does not include or require the actual lxml library. If you need the actual lxml library (which lxml-stubs provides types for), that package has additional system dependencies including libxml2-dev, libxslt1-dev, and their associated libraries."
  },
  "textual": {
    "package": "textual==3.7.1",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for textual==3.7.1 pip package on Ubuntu/Debian systems",
    "notes": "Textual is a modern Text User Interface (TUI) framework for Python. It is distributed as a pure Python wheel (py3-none-any) and has no compiled extensions. The package only requires Python 3.8+ and pip for installation. All of its core dependencies (markdown-it-py, rich, typing-extensions, platformdirs) are also pure Python packages. The optional syntax highlighting features (textual[syntax]) use tree-sitter, which provides pre-compiled binary wheels for all major platforms with no additional system library dependencies. For basic installation, only python3 and python3-pip are required."
  },
  "requests": {
    "package": "requests",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "ca-certificates",
      "libssl3",
      "openssl"
    ],
    "description": "System dependencies required for requests pip package on Ubuntu/Debian systems",
    "notes": "The requests library is primarily pure Python, but requires SSL/TLS support for HTTPS connections. ca-certificates provides trusted certificate authorities for SSL verification. libssl3 and openssl provide the SSL/TLS implementation. For most modern Python installations with pip, requests can be installed with just python3 and python3-pip, but the SSL libraries are essential for secure HTTPS communications which is the primary use case for requests."
  },
  "open3d": {
    "package": "open3d",
    "apt_dependencies": [
      "libgl1-mesa-dev",
      "libgl1-mesa-glx",
      "libglew-dev",
      "libglfw3-dev",
      "libglu1-mesa-dev",
      "libc++-dev",
      "libc++abi-dev",
      "libsdl2-dev",
      "libegl1-mesa-dev",
      "libgles2-mesa-dev",
      "libosmesa6-dev",
      "xorg-dev",
      "libx11-dev",
      "libxrandr-dev",
      "libxi-dev",
      "libxinerama-dev",
      "libxcursor-dev",
      "libxxf86vm-dev",
      "libgomp1",
      "libomp-dev",
      "python3-dev",
      "libpng-dev",
      "libjpeg-dev",
      "libtiff-dev",
      "libavcodec-dev",
      "libavformat-dev",
      "libavutil-dev",
      "libswscale-dev"
    ],
    "description": "System dependencies required for open3d pip package on Ubuntu/Debian systems",
    "notes": "Open3D is a 3D data processing library that requires OpenGL, GLFW, and various graphics libraries for visualization. These dependencies are needed for the pre-built pip wheels to function properly. The exact package names may vary slightly between Ubuntu/Debian versions."
  },
  "plotly": {
    "package": "plotly",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for plotly>=5.9.0 pip package on Ubuntu/Debian systems",
    "notes": "Plotly is a pure Python graphing library that creates interactive HTML/JavaScript visualizations. It does not require compilation or system libraries beyond the Python interpreter. Pre-built wheels are available for all platforms. The package works entirely at the Python level and generates JSON data structures that are rendered using the Plotly.js JavaScript library in the browser. Optional features like static image export (orca, kaleido) have their own separate dependencies, but core plotly functionality only requires Python 3.8+."
  },
  "colorlog": {
    "package": "colorlog",
    "apt_dependencies": [],
    "description": "System dependencies required for colorlog==6.9.0 pip package on Ubuntu/Debian systems",
    "notes": "colorlog is a pure Python package with no native code or system library dependencies. It only requires Python >= 3.6 to be installed. The package provides colored terminal output for Python's logging module and has no apt-get dependencies. On Windows, it optionally uses colorama for color support, but this is not needed on Linux systems."
  },
  "scipy": {
    "package": "scipy",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "gfortran",
      "libopenblas-dev",
      "liblapack-dev",
      "pkg-config",
      "libatlas-base-dev",
      "libblas-dev"
    ],
    "description": "System dependencies required for scipy>=1.15.1 pip package on Ubuntu/Debian systems",
    "notes": "SciPy requires BLAS/LAPACK libraries for linear algebra operations. libopenblas-dev and libatlas-base-dev provide optimized BLAS implementations. gfortran is needed for building Fortran extensions from source. For pre-built wheels (most common installation method), only python3 and python3-pip are strictly required at runtime, but the build dependencies are included for systems that need to compile from source or for older architectures without wheel support. SciPy 1.15.1 requires NumPy as a prerequisite."
  },
  "numba": {
    "package": "numba",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "llvm-14",
      "llvm-14-dev",
      "libedit-dev",
      "libxml2-dev",
      "zlib1g-dev"
    ],
    "description": "System dependencies required for numba>=0.60.0 pip package on Ubuntu/Debian systems",
    "notes": "Numba is a JIT compiler that translates Python and NumPy code to LLVM-compiled machine code. It requires llvmlite (which itself depends on LLVM) as a critical dependency. LLVM 14 is recommended for numba 0.60.0. The llvm-14-dev package provides LLVM headers and libraries needed for llvmlite. libedit-dev, libxml2-dev, and zlib1g-dev are additional LLVM build dependencies. build-essential provides gcc/g++ compilers. For pre-built wheels (most common installation method), the LLVM runtime libraries are typically bundled, but having LLVM installed system-wide ensures compatibility. When building from source or when wheels are unavailable, all build dependencies are required. Numba also requires NumPy as a prerequisite at runtime."
  },
  "mmcv": {
    "package": "mmcv",
    "version": ">=2.1.0",
    "apt_dependencies": [
      "python3-dev",
      "build-essential",
      "ninja-build",
      "libopencv-dev",
      "libgl1-mesa-glx",
      "libglib2.0-0",
      "libsm6",
      "libxext6",
      "libxrender-dev",
      "libgomp1",
      "libgstreamer1.0-0",
      "libgstreamer-plugins-base1.0-0",
      "libjpeg-dev",
      "libjpeg8-dev",
      "libpng-dev",
      "libtiff-dev",
      "libavcodec-dev",
      "libavformat-dev",
      "libswscale-dev",
      "libv4l-dev",
      "libatlas-base-dev",
      "gfortran",
      "libhdf5-dev",
      "pkg-config"
    ],
    "description": "System dependencies required for mmcv>=2.1.0 pip package on Ubuntu/Debian systems",
    "notes": "MMCV (MMDetection Computer Vision) is a foundational library for OpenMMLab's computer vision projects. Building from source requires: C++ compiler and build tools (build-essential, ninja-build, pkg-config), Python development headers (python3-dev), OpenCV development libraries (libopencv-dev) for core CV operations, GUI and windowing support (libgl1-mesa-glx, libglib2.0-0, libsm6, libxext6, libxrender-dev), parallel processing (libgomp1), GStreamer for video I/O (libgstreamer1.0-0, libgstreamer-plugins-base1.0-0), image format codecs (libjpeg-dev, libjpeg8-dev, libpng-dev, libtiff-dev), video codec support (libavcodec-dev, libavformat-dev, libswscale-dev), camera access (libv4l-dev), optimized linear algebra (libatlas-base-dev, gfortran), and HDF5 for data storage (libhdf5-dev). MMCV 2.x requires PyTorch and CUDA for GPU support. Pre-built wheels are available for common configurations via 'pip install mmcv -f https://download.openmmlab.com/mmcv/dist/{cu_version}/{torch_version}/index.html', but building from source provides optimal performance and compatibility."
  },
  "empy": {
    "package": "empy",
    "apt_dependencies": [],
    "description": "System dependencies required for empy==3.3.4 pip package on Ubuntu/Debian systems",
    "notes": "empy is a pure Python templating system with no native code or system library dependencies. It only requires Python itself to be installed. The package is entirely self-contained and works with any Python 3 installation."
  },
  "rxpy-backpressure": {
    "package": "rxpy-backpressure",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for rxpy-backpressure pip package on Ubuntu/Debian systems",
    "notes": "rxpy-backpressure is a pure Python package that provides reactive extensions with backpressure support. It does not require additional system-level dependencies beyond Python itself. All of its dependencies are Python packages available via pip. The package only requires Python 3.6+ and pip."
  },
  "scikit-learn": {
    "package": "scikit-learn",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "gfortran",
      "libopenblas-dev",
      "liblapack-dev",
      "pkg-config",
      "libomp-dev",
      "cython3"
    ],
    "description": "System dependencies required for scikit-learn pip package on Ubuntu/Debian systems",
    "notes": "Scikit-learn depends on NumPy, SciPy, and joblib, which require BLAS/LAPACK libraries for numerical operations. libopenblas-dev provides optimized BLAS implementation. gfortran is needed for building SciPy from source. libomp-dev provides OpenMP support for parallel operations. cython3 is used for building optimized extensions. For pre-built wheels (most common installation method), only python3 and python3-pip are strictly required at runtime, but the build dependencies are included for systems that need to compile from source or for older architectures without wheel support."
  },
  "PyTurboJPEG": {
    "package": "PyTurboJPEG",
    "version": "1.8.2",
    "apt_dependencies": [
      "libturbojpeg0-dev",
      "libturbojpeg"
    ],
    "description": "PyTurboJPEG is a Python wrapper for libjpeg-turbo, which requires the TurboJPEG library to be installed on the system"
  },
  "langchain": {
    "package": "langchain>=1,<2",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "libssl-dev",
      "libffi-dev"
    ],
    "description": "System dependencies required for langchain>=1,<2 pip package on Ubuntu/Debian systems",
    "notes": "LangChain is primarily a pure Python package for building LLM applications. Core dependencies are minimal - mainly python3 and pip. build-essential, libssl-dev, and libffi-dev are included for building cryptography-related dependencies (like pydantic-core and other compiled extensions) that langchain and its dependencies may require. Many of langchain's features depend on optional integrations (vector stores, specific LLM providers, etc.) which may have their own system requirements, but these are not included as they are optional."
  },
  "uvicorn": {
    "package": "uvicorn>=0.34.0",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "python3-dev",
      "build-essential"
    ],
    "description": "System dependencies required for uvicorn>=0.34.0 pip package on Ubuntu/Debian systems",
    "notes": "Uvicorn is primarily a pure Python ASGI server. The core package only requires Python 3.8+. However, python3-dev and build-essential are included because uvicorn's optional performance dependencies (uvloop and httptools) are compiled extensions that require C headers and build tools during pip installation. For standard installations with 'uvicorn[standard]', these build dependencies ensure optimal performance. For minimal installations without compiled extensions, only python3 and python3-pip are strictly necessary."
  },
  "sentence_transformers": {
    "package": "sentence_transformers",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "cargo",
      "rustc",
      "libssl-dev",
      "libffi-dev",
      "libyaml-dev",
      "pkg-config",
      "gfortran",
      "libopenblas-dev",
      "liblapack-dev"
    ],
    "description": "System dependencies required for sentence_transformers pip package on Ubuntu/Debian systems",
    "notes": "sentence_transformers is built on PyTorch and Transformers libraries for computing sentence/text embeddings. Key system dependencies: (1) cargo/rustc are required for building the 'tokenizers' package (Rust-based fast tokenization library used by transformers), (2) build-essential provides gcc/g++ for compiling C/C++ extensions in PyTorch, NumPy, and other numerical packages, (3) gfortran, libopenblas-dev, and liblapack-dev are needed for BLAS/LAPACK linear algebra operations (critical for tensor operations in PyTorch and NumPy), (4) libssl-dev and libffi-dev for cryptographic operations in huggingface-hub and other dependencies, (5) libyaml-dev for PyYAML C extensions, (6) pkg-config for library detection during builds. The package depends on transformers, torch, torchvision, numpy, scikit-learn, scipy, nltk, and sentencepiece. When installing from pre-built wheels on common platforms (x86_64/aarch64 Linux with recent pip and glibc), only python3 and python3-pip are strictly required at runtime, but the build dependencies ensure compatibility across all architectures and when building from source or working with older systems."
  },
  "langchain-ollama": {
    "package": "langchain-ollama>=1,<2",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "libssl-dev",
      "libffi-dev"
    ],
    "description": "System dependencies required for langchain-ollama>=1,<2 pip package on Ubuntu/Debian systems",
    "notes": "langchain-ollama is a LangChain integration for Ollama that provides a Python client interface. It depends on langchain-core and the ollama Python package (which uses httpx for HTTP requests and pydantic>=2.9 for data validation). The system dependencies are minimal for pre-built wheels: python3 and pip are required for installation. build-essential, libssl-dev, and libffi-dev are included for compiling pydantic-core (Rust-based) and other compiled extensions in the dependency tree, particularly when pre-built wheels aren't available or when building from source. The package itself is pure Python but its dependencies (especially pydantic v2) include compiled components."
  },
  "ctransformers[cuda]": {
    "package": "ctransformers[cuda]",
    "version": "0.2.27",
    "apt_dependencies": [
      "python3-dev",
      "python3-pip",
      "build-essential",
      "gcc",
      "g++",
      "make",
      "cmake",
      "libstdc++6",
      "nvidia-cuda-toolkit",
      "libcublas-dev",
      "libcudnn8",
      "libcudnn8-dev"
    ],
    "description": "System dependencies required for ctransformers[cuda]==0.2.27 pip package on Ubuntu/Debian systems",
    "notes": "ctransformers[cuda] is a Python library providing bindings for transformer models using GGML/GGUF format with CUDA GPU acceleration. It requires both C/C++ compilation toolchain and CUDA development libraries. build-essential provides essential compilation tools (gcc, g++, make). cmake is needed for the build process. python3-dev provides Python header files needed to compile C extensions. libstdc++6 provides the standard C++ library runtime. nvidia-cuda-toolkit provides CUDA compiler and libraries. libcublas-dev provides CUDA linear algebra library headers. libcudnn8 and libcudnn8-dev provide deep learning primitives for neural network operations. The CUDA dependencies enable GPU-accelerated inference for transformer models."
  },
  "Pillow": {
    "package": "Pillow",
    "apt_dependencies": [
      "python3-dev",
      "python3-pip",
      "libjpeg-dev",
      "libjpeg8-dev",
      "zlib1g-dev",
      "libtiff-dev",
      "libfreetype6-dev",
      "liblcms2-dev",
      "libwebp-dev",
      "libharfbuzz-dev",
      "libfribidi-dev",
      "libopenjp2-7-dev",
      "libimagequant-dev",
      "libxcb1-dev",
      "tk-dev",
      "tcl-dev"
    ],
    "description": "System dependencies required for Pillow (PIL Fork) pip package on Ubuntu/Debian systems",
    "notes": "Pillow is the Python Imaging Library fork. These dependencies enable full image format support including JPEG (libjpeg-dev), PNG (zlib1g-dev), TIFF (libtiff-dev), WebP (libwebp-dev), JPEG 2000 (libopenjp2-7-dev), and text rendering with TrueType/OpenType fonts (libfreetype6-dev, libharfbuzz-dev, libfribidi-dev). liblcms2-dev provides color management support. tk-dev and tcl-dev enable ImageTk module for Tkinter integration. While pre-built wheels are available for most platforms, these dependencies are required for building from source or for optimal feature support."
  },
  "coverage": {
    "package": "coverage>=7.0",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential"
    ],
    "description": "System dependencies required for coverage>=7.0 pip package on Ubuntu/Debian systems",
    "notes": "Coverage.py is a code coverage measurement tool for Python. It includes optional C extensions for performance optimization. build-essential (which provides gcc and build tools) is needed if building from source or compiling the C extensions. For most installations using pre-built wheels, only python3 and python3-pip are strictly required. The C extension significantly improves performance but is not mandatory for the package to function."
  },
  "python-multipart": {
    "package": "python-multipart",
    "apt_dependencies": [],
    "description": "",
    "notes": "Pure Python package with no system-level dependencies. Only requires Python runtime."
  },
  "pre_commit": {
    "package": "pre_commit",
    "apt_dependencies": [
      "git",
      "python3",
      "python3-pip",
      "build-essential",
      "libyaml-dev"
    ],
    "description": "System dependencies required for pre_commit==4.2.0 pip package on Ubuntu/Debian systems",
    "notes": "Pre-commit is a framework for managing git hooks. It requires git for hook functionality. The build-essential and libyaml-dev packages are needed for compiling PyYAML (a dependency) with C extensions for better performance. If installing from pre-built wheels, only git, python3, and python3-pip are strictly required at runtime. Pre-commit also uses virtualenv and nodeenv to create isolated environments for hooks written in different languages."
  },
  "xarm-python-sdk": {
    "package": "xarm-python-sdk",
    "apt_dependencies": [],
    "description": "System dependencies required for xarm-python-sdk pip package on Ubuntu/Debian systems",
    "notes": "xarm-python-sdk is a pure Python package (py3-none-any wheel) that uses only Python standard library modules. It has no Python package dependencies (requirements.txt is empty) and requires no system-level apt-get packages. The SDK provides a Python interface for controlling UFACTORY robotic arms (850, xArm 5/6/7, and Lite6) over network socket connections."
  },
  "matplotlib": {
    "package": "matplotlib",
    "apt_dependencies": [
      "python3-dev",
      "python3-pip",
      "pkg-config",
      "libfreetype6-dev",
      "libpng-dev",
      "libjpeg-dev",
      "libqhull-dev",
      "libfontconfig1-dev",
      "libxft-dev",
      "libxcb-render0-dev",
      "libxcb-shm0-dev",
      "libxrender-dev",
      "libxext-dev",
      "libx11-dev",
      "tk-dev",
      "tcl-dev",
      "libgtk-3-dev",
      "libcairo2-dev",
      "libgirepository1.0-dev",
      "gir1.2-gtk-3.0"
    ],
    "description": "System dependencies required for matplotlib>=3.7.1 pip package on Ubuntu/Debian systems",
    "notes": "Matplotlib is a comprehensive plotting library for Python. Core dependencies include FreeType (libfreetype6-dev) for font rendering, libpng-dev for PNG image support, and libqhull-dev for computational geometry. X11 libraries (libxcb-*, libxrender-dev, libxext-dev, libx11-dev) are required for interactive backends. GTK3 support (libgtk-3-dev, libcairo2-dev, libgirepository1.0-dev, gir1.2-gtk-3.0) enables the GTK3Agg/GTK3Cairo backends. Tk/Tcl (tk-dev, tcl-dev) enables the TkAgg backend. While pre-built wheels are available for most platforms and only require python3-pip at runtime, these dependencies are essential for building from source, enabling all backends, or for systems without wheel support. matplotlib>=3.7.1 requires NumPy as a prerequisite."
  },
  "langchain-huggingface": {
    "package": "langchain-huggingface>=1,<2",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "cargo",
      "rustc",
      "libssl-dev",
      "libffi-dev",
      "libyaml-dev",
      "pkg-config"
    ],
    "description": "System dependencies required for langchain-huggingface>=1,<2 pip package on Ubuntu/Debian systems",
    "notes": "langchain-huggingface provides HuggingFace integrations for LangChain, including embeddings and LLM wrappers. Key system dependencies: (1) cargo/rustc are required for building the 'tokenizers' package (Rust-based fast tokenization library) and pydantic-core (via langchain-core dependency), (2) build-essential for compiling any C/C++ extensions, (3) libssl-dev and libffi-dev for cryptographic operations in huggingface-hub and other dependencies, (4) libyaml-dev for PyYAML C extensions, (5) pkg-config for library detection during builds. Core dependencies include huggingface-hub (for model downloading), tokenizers (Rust-based tokenization), and langchain-core. When installing from pre-built wheels on common platforms (x86_64/aarch64 Linux with recent pip), only python3 and python3-pip are strictly required at runtime, but the build dependencies ensure compatibility across all architectures and when building from source."
  },
  "sse-starlette": {
    "package": "sse-starlette",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for sse-starlette>=2.2.1 pip package on Ubuntu/Debian systems",
    "notes": "sse-starlette is a pure Python package for Server-Sent Events (SSE) support in Starlette/FastAPI applications. It has no native extensions or system library requirements. The package depends on starlette>=0.49.1 and anyio>=4.7.0, both of which are also pure Python. Therefore, only the basic Python runtime (python3) and pip (python3-pip) are required. No build tools or additional system libraries are needed."
  },
  "wasmtime": {
    "package": "wasmtime",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for wasmtime>=40.0.0 pip package on Ubuntu/Debian systems",
    "notes": "Wasmtime is a WebAssembly runtime that ships with pre-built binary wheels for all major platforms (Linux x86_64, Linux aarch64, macOS, Windows). The wheels contain the compiled Wasmtime runtime, so no compilation or additional system libraries are required. Only Python 3 and pip are needed to install the pre-built wheel. If building from source (rare), standard build tools (build-essential, cargo/rust) would be required, but this is uncommon as PyPI provides wheels for all standard platforms."
  },
  "lark": {
    "package": "lark",
    "apt_dependencies": [],
    "description": "System dependencies required for lark pip package on Ubuntu/Debian systems",
    "notes": "Lark (lark-parser) is a pure Python parsing library with no native code or system library dependencies. It only requires Python >= 3.6 to be installed. The package provides a modern parsing library capable of parsing any context-free grammar and has no apt-get dependencies. It works entirely with Python standard library and has no external system requirements."
  },
  "ftfy": {
    "package": "ftfy",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for ftfy>=6.3.1 pip package on Ubuntu/Debian systems",
    "notes": "ftfy is a pure Python package for fixing Unicode text encoding issues (mojibake). It only requires wcwidth as a Python dependency, which is also pure Python. No C extensions or system libraries are needed beyond the basic Python runtime environment. Pre-built wheels are available for all platforms."
  },
  "onnxruntime": {
    "package": "onnxruntime",
    "apt_dependencies": [
      "libgomp1",
      "libstdc++6",
      "libc6"
    ],
    "description": "System dependencies required for onnxruntime pip package on Ubuntu/Debian systems",
    "notes": "ONNX Runtime is distributed as pre-built wheels for most platforms and has minimal system dependencies. libgomp1 is needed for OpenMP support (parallel execution). libstdc++6 and libc6 are standard C++ and C libraries typically already present on most systems. For GPU support (onnxruntime-gpu), additional CUDA/cuDNN libraries are required but those are handled separately. The Python wheel includes most dependencies statically linked, making it relatively self-contained compared to building from source."
  },
  "pytest-timeout": {
    "package": "pytest-timeout",
    "apt_dependencies": [],
    "description": "System dependencies required for pytest-timeout==2.4.0 pip package on Ubuntu/Debian systems",
    "notes": "pytest-timeout is a pure Python package with no native code or system library dependencies. It is a pytest plugin that adds timeout functionality to abort hanging tests. The package only requires Python >= 3.7 and pytest >= 7.0.0 to be installed. It has no apt-get dependencies as it's implemented entirely in Python without any C extensions or system library bindings."
  },
  "cupy-cuda12x": {
    "package": "cupy-cuda12x==13.6.0",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "libgomp1",
      "gcc",
      "g++"
    ],
    "description": "System dependencies required for cupy-cuda12x==13.6.0 pip package on Ubuntu/Debian systems",
    "notes": "CuPy CUDA 12.x is distributed as a binary wheel package that includes CUDA Toolkit libraries (cuDNN, cuTENSOR, NCCL, cutensor) bundled within the package, so CUDA installation is not required as a system dependency. The apt dependencies listed are minimal runtime requirements. libgomp1 is required for OpenMP support used by CuPy. build-essential (gcc, g++) may be needed for some operations that require compilation of custom CUDA kernels at runtime. For most use cases with pre-built wheels, only python3, python3-pip, and libgomp1 are strictly necessary."
  },
  "types-colorama": {
    "package": "types-colorama",
    "version_spec": ">=0.4.15.20250801,<1",
    "apt_dependencies": []
  },
  "onnxruntime-gpu": {
    "package": "onnxruntime-gpu",
    "apt_dependencies": [
      "libgomp1",
      "libstdc++6",
      "libc6",
      "cuda-toolkit-12-0",
      "libcudnn8",
      "libcublas-12-0",
      "libcufft-12-0",
      "libcurand-12-0",
      "libcusolver-12-0",
      "libcusparse-12-0",
      "libnvinfer8",
      "libnvinfer-plugin8",
      "libnvonnxparsers8",
      "nvidia-cuda-toolkit"
    ],
    "description": "System dependencies required for onnxruntime-gpu>=1.17.1 pip package on Ubuntu/Debian systems",
    "notes": "ONNX Runtime GPU version requires CUDA 12.x runtime libraries and cuDNN 8.x for GPU acceleration. The package is distributed as pre-built wheels with some CUDA components bundled, but system CUDA libraries are still required. libgomp1 is needed for OpenMP support. CUDA toolkit includes the core GPU compute libraries (cuBLAS, cuFFT, cuRAND, cuSOLVER, cuSPARSE). cuDNN provides optimized primitives for deep learning. TensorRT libraries (libnvinfer*) are optional but recommended for additional optimization. For ONNX Runtime 1.17.1+, CUDA 12.0+ and cuDNN 8.9+ are the recommended versions. Note that the exact CUDA version may need to match what the wheel was built against - check the official ONNX Runtime release notes for the specific version compatibility."
  },
  "filterpy": {
    "package": "filterpy",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "gfortran",
      "libopenblas-dev",
      "liblapack-dev",
      "pkg-config"
    ],
    "description": "System dependencies required for filterpy>=1.4.5 pip package on Ubuntu/Debian systems",
    "notes": "FilterPy is a Kalman filtering library that depends on NumPy and SciPy. The system dependencies listed here are primarily for building and running NumPy and SciPy, which are the core dependencies of filterpy. gfortran is needed for SciPy's Fortran code, and libopenblas-dev/liblapack-dev provide optimized linear algebra operations. For pre-built wheels (most common installation method), only python3 and python3-pip are strictly required at runtime, but the build dependencies are included for systems that need to compile from source."
  },
  "types-jsonschema": {
    "package": "types-jsonschema",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for types-jsonschema>=4.25.1.20251009,<5 pip package on Ubuntu/Debian systems",
    "notes": "The types-jsonschema package is a pure Python type stub package from the typeshed project. It contains only type hint files (.pyi) with no compiled extensions or runtime code. Only python3 and python3-pip are required for installation."
  },
  "pygame": {
    "package": "pygame",
    "apt_dependencies": [
      "libsdl2-dev",
      "libsdl2-image-dev",
      "libsdl2-mixer-dev",
      "libsdl2-ttf-dev",
      "libfreetype6-dev",
      "libportmidi-dev",
      "libjpeg-dev",
      "python3-dev",
      "libsdl2-2.0-0",
      "libsdl2-image-2.0-0",
      "libsdl2-mixer-2.0-0",
      "libsdl2-ttf-2.0-0"
    ],
    "description": "System dependencies required for pygame>=2.6.1 pip package on Ubuntu/Debian systems",
    "notes": "Pygame 2.6.1+ is built on SDL2 (Simple DirectMedia Layer 2). The -dev packages (libsdl2-dev, libsdl2-image-dev, libsdl2-mixer-dev, libsdl2-ttf-dev) provide headers and development files needed for building pygame from source via pip. The runtime libraries (libsdl2-2.0-0, etc.) are required for pygame to function. libfreetype6-dev provides font rendering support, libportmidi-dev enables MIDI support, libjpeg-dev adds JPEG image format support, and python3-dev provides Python development headers. While pygame provides pre-built wheels for many platforms, these dependencies ensure full feature support and are required when building from source."
  },
  "mmengine": {
    "package": "mmengine>=0.10.3",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "python3-dev",
      "build-essential",
      "libyaml-dev"
    ],
    "description": "System dependencies required for mmengine>=0.10.3 pip package on Ubuntu/Debian systems",
    "notes": "MMEngine is OpenMMLab's foundational library for training deep learning models. It's primarily a pure Python package with minimal system dependencies. python3-dev and build-essential may be needed for compiling C extensions from dependencies like PyYAML. libyaml-dev provides faster YAML parsing. For most use cases with pre-built wheels, only python3 and python3-pip are strictly required at runtime."
  },
  "dimos-lcm": {
    "package": "dimos-lcm",
    "version": "0.1.0",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "python3-dev",
      "build-essential",
      "liblcm-dev",
      "lcm-tools",
      "default-jdk",
      "default-jre",
      "git"
    ],
    "description": "System dependencies required for dimos-lcm pip package on Ubuntu/Debian systems",
    "notes": "dimos-lcm is a LCM-Foxglove bridge and message utilities package that requires the LCM (Lightweight Communications and Marshalling) library. Key dependencies include: LCM development libraries (liblcm-dev) for the core C/C++ libraries, LCM tools (lcm-tools) which provide lcm-gen for code generation and lcm-spy for message inspection, Java Development Kit (default-jdk) and Java Runtime Environment (default-jre) for building and running Java bindings including lcm-spy, Python development headers (python3-dev) for building Python bindings, and build-essential for general compilation tools. The package also depends on numpy, foxglove-websocket, and lcm Python packages which are installed via pip. Git is required as the package is typically installed from a GitHub repository."
  },
  "einops": {
    "package": "einops==0.8.1",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for einops==0.8.1 pip package on Ubuntu/Debian systems",
    "notes": "einops (Einstein Operations) is a pure Python package for tensor operations with no external dependencies beyond Python itself. It works with various deep learning frameworks (PyTorch, TensorFlow, JAX, etc.) but does not require them directly. The package is distributed as a universal wheel, so no build tools are needed for installation."
  },
  "python-dotenv": {
    "package": "python-dotenv",
    "apt_dependencies": [],
    "description": "System dependencies required for python-dotenv pip package on Ubuntu/Debian systems",
    "notes": "python-dotenv is a pure Python package with no native code or system library dependencies. It only requires Python itself to be installed."
  },
  "tiktoken": {
    "package": "tiktoken",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "cargo",
      "rustc"
    ],
    "description": "System dependencies required for tiktoken>=0.8.0 pip package on Ubuntu/Debian systems",
    "notes": "tiktoken is a BPE tokenizer library with Rust extensions. Pre-built wheels are available for common platforms (x86_64, aarch64 on Linux/macOS/Windows), which only require python3 and python3-pip at runtime. However, build-essential, cargo, and rustc are required for platforms without pre-built wheels or when building from source. The Rust toolchain (cargo and rustc) is essential for compiling the tiktoken-rust core library."
  },
  "pytest-env": {
    "package": "pytest-env==1.1.5",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for pytest-env==1.1.5 pip package on Ubuntu/Debian systems",
    "notes": "pytest-env is a pytest plugin for setting environment variables from pytest configuration files. It is a pure Python package that depends only on pytest. The package has no native extensions or system-level library requirements beyond the base Python installation and pip package manager. All dependencies (pytest and its dependencies) are available via pip."
  },
  "fastapi": {
    "package": "fastapi>=0.115.6",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for fastapi>=0.115.6 pip package on Ubuntu/Debian systems",
    "notes": "FastAPI is a pure Python web framework built on Starlette and Pydantic. It does not require additional system-level dependencies beyond Python itself when installing pre-built wheels. All core dependencies (starlette, pydantic, pydantic-core, typing-extensions) are available as wheels. For production deployments, you may want uvicorn[standard] which includes optional C-extension dependencies (uvloop, httptools) for better performance, but these also typically install from wheels. Building from source or on architectures without wheel support may require build-essential, but this is uncommon for typical x86_64/ARM64 systems."
  },
  "regex": {
    "package": "regex",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "python3-dev",
      "build-essential",
      "gcc"
    ],
    "description": "System dependencies required for regex pip package on Ubuntu/Debian systems",
    "notes": "The regex module is an alternative regular expression module for Python that is backwards-compatible with the standard 're' module but offers additional functionality. It contains C extensions that need to be compiled. python3-dev provides Python header files, build-essential includes make and other build tools, and gcc is the C compiler. For most modern platforms, pre-built wheels are available on PyPI, making these build dependencies only necessary when building from source on unsupported architectures or when wheels are not available. At runtime, only python3 is strictly required when using pre-built wheels."
  },
  "piper-sdk": {
    "package": "piper-sdk",
    "apt_dependencies": [
      "can-utils",
      "ethtool",
      "iproute2"
    ],
    "description": "System dependencies required for piper-sdk pip package on Ubuntu/Debian systems",
    "notes": "piper-sdk is a Python SDK for controlling Agilex Piper robotic arms via CAN bus communication. The package requires can-utils for CAN interface configuration tools, ethtool for network interface diagnostics, and iproute2 for the 'ip' command used in the CAN module activation scripts. These dependencies are essential for detecting, configuring, and activating USB-to-CAN modules at the required 1000000 baud rate."
  },
  "pandas-stubs": {
    "package": "pandas-stubs>=2.3.2.250926,<3",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for pandas-stubs>=2.3.2.250926,<3 pip package on Ubuntu/Debian systems",
    "notes": "pandas-stubs is a type stubs package that provides type hints for the pandas library. It is used for static type checking with mypy and other type checkers. As a pure Python package containing only type stub files (.pyi), it has no compiled components or system library dependencies beyond Python itself. The package is installed at development/type-checking time and has no runtime dependencies."
  },
  "pytest": {
    "package": "pytest==8.3.5",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for pytest==8.3.5 pip package on Ubuntu/Debian systems",
    "notes": "pytest is a pure Python testing framework and does not require additional system-level dependencies beyond Python itself. All of pytest's dependencies (iniconfig, packaging, pluggy) are also pure Python packages available via pip. The framework works on Python 3.8+ and only requires the base Python installation and pip package manager."
  },
  "moondream": {
    "package": "moondream",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "libjpeg-dev",
      "zlib1g-dev",
      "libtiff-dev",
      "libfreetype6-dev",
      "liblcms2-dev",
      "libwebp-dev",
      "libharfbuzz-dev",
      "libfribidi-dev",
      "libxcb1-dev",
      "libopenjp2-7-dev"
    ],
    "description": "System dependencies required for moondream pip package on Ubuntu/Debian systems",
    "notes": "Moondream is a vision language model that depends on Pillow (>=10.4.0,<11.0.0) for image processing. The listed dependencies are primarily for building and running Pillow with full image format support. Requires Python >=3.10,<4.0. The actual moondream model inference may have additional runtime dependencies (like PyTorch) that are bundled with the package or installed as hidden dependencies."
  },
  "gdown": {
    "package": "gdown",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "ca-certificates",
      "libssl3",
      "openssl"
    ],
    "description": "System dependencies required for gdown==5.2.0 pip package on Ubuntu/Debian systems",
    "notes": "gdown is a pure Python package for downloading files from Google Drive. It has no direct system library dependencies. However, it depends on requests[socks] which requires SSL/TLS support for HTTPS connections to Google Drive. ca-certificates provides trusted certificate authorities for SSL verification, while libssl3 and openssl provide the SSL/TLS implementation. For most modern Python installations with pip, gdown can be installed with just python3 and python3-pip, but the SSL libraries are essential for secure HTTPS communications with Google Drive."
  },
  "reactivex": {
    "package": "reactivex",
    "apt_dependencies": [],
    "description": "System dependencies required for reactivex pip package on Ubuntu/Debian systems",
    "notes": "reactivex (ReactiveX for Python, also known as RxPY) is a pure Python package with no native code or system library dependencies. It implements reactive programming patterns using observables and requires only Python >= 3.8 to be installed. The package has no apt-get dependencies as it's entirely implemented in Python without any C extensions or system library bindings."
  },
  "bitsandbytes": {
    "package": "bitsandbytes",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "build-essential",
      "nvidia-cuda-toolkit",
      "libcudnn8",
      "libcudnn8-dev",
      "gcc",
      "g++",
      "make"
    ],
    "description": "System dependencies required for bitsandbytes>=0.48.2,<1.0 pip package on Ubuntu/Debian systems (Linux only)",
    "notes": "bitsandbytes requires CUDA toolkit for GPU-accelerated 8-bit optimization and quantization. The package is Linux-only (sys_platform == 'linux'). For CUDA 11.x, use nvidia-cuda-toolkit. For CUDA 12.x, the package name may vary (nvidia-cuda-toolkit-12-x). Build tools (gcc, g++, make, build-essential) are needed for compiling CUDA kernels. libcudnn8 provides deep learning primitives. Note: The exact CUDA version required depends on your PyTorch installation - ensure CUDA toolkit version matches PyTorch's CUDA version."
  },
  "yapf": {
    "package": "yapf==0.40.2",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for yapf==0.40.2 pip package on Ubuntu/Debian systems",
    "notes": "YAPF (Yet Another Python Formatter) is a pure Python package with no native code or system library dependencies beyond Python itself. Version 0.40.2 requires Python 3.7 or later. It has minimal pip dependencies (importlib-metadata for Python <3.8, platformdirs, and tomli for Python <3.11) which are all pure Python packages. No additional apt packages are required beyond the Python interpreter and pip."
  },
  "langchain-openai": {
    "package": "langchain-openai>=1,<2",
    "apt_dependencies": [
      "python3",
      "python3-pip",
      "python3-dev",
      "build-essential",
      "libssl-dev",
      "libffi-dev",
      "ca-certificates"
    ],
    "description": "System dependencies required for langchain-openai>=1,<2 pip package on Ubuntu/Debian systems",
    "notes": "langchain-openai is a pure Python integration package that connects LangChain with OpenAI's API. It requires python3 runtime, development headers (python3-dev) for building compiled dependencies, build tools (build-essential) for native extensions in dependencies like pydantic-core and tiktoken, SSL/TLS libraries (libssl-dev, libffi-dev, ca-certificates) for secure HTTPS communication with OpenAI's API endpoints, and standard pip for installation. The package inherits dependencies from both langchain-core and the openai SDK."
  },
  "structlog": {
    "package": "structlog",
    "apt_dependencies": [],
    "description": "System dependencies required for structlog>=25.5.0,<26 pip package on Ubuntu/Debian systems",
    "notes": "structlog is a pure Python package with no native code or system library dependencies. It only requires Python itself to be installed. The package provides structured logging capabilities using only Python's standard library and has no C extensions or binary dependencies."
  },
  "terminaltexteffects": {
    "package": "terminaltexteffects",
    "apt_dependencies": [
      "python3",
      "python3-pip"
    ],
    "description": "System dependencies required for terminaltexteffects==0.12.2 pip package on Ubuntu/Debian systems",
    "notes": "TerminalTextEffects is a pure Python package with no third-party dependencies. It requires only Python 3.8+ and uses standard ANSI terminal sequences. No additional system libraries or build tools are needed. The package works out of the box with just python3 and python3-pip for installation."
  },
  "dataclasses": {
    "package": "dataclasses",
    "type": "pip",
    "apt_dependencies": [],
    "notes": "Pure Python module with no system dependencies. Part of Python standard library since 3.7, only needed as backport for Python < 3.7"
  }
}
