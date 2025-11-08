Mac / ARM Support

Check out docker for context on x86/ubuntu system libraries that would need to be solved for. This bounty requires support for both CPU only ARM build and GPU. Requires handling of system libraries including:

BLAS / LAPACK for SciPy & friends
libopenblas-dev  gfortran  liblapacke-dev
Media I/O – used by ffmpeg-python, aiortc, whisper, etc.
ffmpeg  libvpx-dev  libopus-dev  libsndfile1-dev
Build chain for all C/C++ wheels that may have to compile from source on ARM
build-essential  cmake  pkg-config
Misc. libraries required by Python packages in requirements.txt
• libportaudio2 (runtime lib for PortAudio) – header is already installed via portaudio19-dev
• libjpeg-dev  zlib1g-dev (Pillow falls back to them if no wheel)
• libfreetype6-dev (also for Pillow)
• libfftw3-dev (scipy optional but speeds up filters)
CUDA-specific (only when targeting Jetson)
• cuda-toolkit-12-3 (for pycuda, ctransformers[cuda], etc.)
Optional GUI helpers (needed if you plan to run RViz or Qt5 apps under X11/Wayland)
libxcb-xinput-dev  libxrender-dev  libfontconfig1-dev  libxkbcommon-dev



Further context

The first step if for the CPU only build uv pip install -e .[cpu] is currently has some system level issues with some of those deps not running on ARM

Also we do have a build for Jetson jetpack 6.2. Jetson runs ARM as well. For that one you can see we are using many of the pre complied wheels that nvidia provides to build properly: 
jetson-jp6-cuda126 = [
    # Jetson Jetpack 6.2 with CUDA 12.6 specific wheels
    # Note: Alternative torch wheel from docs: https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
    "torch @ https://pypi.jetson-ai-lab.io/jp6/cu126/+f/564/4d4458f1ba159/torch-2.8.0-cp310-cp310-linux_aarch64.whl",
    "torchvision @ https://pypi.jetson-ai-lab.io/jp6/cu126/+f/1c0/3de08a69e9554/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl",
    "onnxruntime-gpu @ https://pypi.jetson-ai-lab.io/jp6/cu126/+f/4eb/e6a8902dc7708/onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl",
    "xformers @ https://pypi.jetson-ai-lab.io/jp6/cu126/+f/731/15133b0ebb2b3/xformers-0.0.33+ac00641.d20250830-cp39-abi3-linux_aarch64.whl",
]



