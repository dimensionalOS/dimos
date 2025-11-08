# Mac ARM (Apple Silicon) Support Setup Notes

## Prerequisites
- macOS with Apple Silicon (M1/M2/M3 chips)
- Homebrew installed (`/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"`)
- uv installed (`curl -LsSf https://astral.sh/uv/install.sh | sh`)

## System Dependencies Installed via Homebrew
The following system libraries are required for compilation of Python packages that don't have pre-built ARM wheels:

- `openblas` - BLAS library for linear algebra
- `lapack` - LAPACK library for linear algebra
- `gcc` - GNU Compiler Collection (includes gfortran)
- `ffmpeg` - Multimedia framework
- `libvpx` - VP8/VP9 video codec
- `opus` - Audio codec
- `libsndfile` - Audio file I/O
- `portaudio` - Audio I/O library
- `jpeg` - JPEG image library
- `zlib` - Compression library
- `freetype` - Font rendering library
- `fftw` - Fast Fourier Transform library
- `cmake` - Build system
- `pkg-config` - Package configuration tool

## Python Requirements
- Python 3.12 (installed via `uv python install 3.12`)
- Virtual environment created with `uv venv --python 3.12`

## Environment Variables
For keg-only Homebrew packages (not symlinked into /opt/homebrew), the following environment variables must be set during installation:

```bash
export LDFLAGS="-L/opt/homebrew/opt/openblas/lib -L/opt/homebrew/opt/lapack/lib -L/opt/homebrew/opt/jpeg/lib -L/opt/homebrew/opt/zlib/lib $LDFLAGS"
export CPPFLAGS="-I/opt/homebrew/opt/openblas/include -I/opt/homebrew/opt/lapack/include -I/opt/homebrew/opt/jpeg/include -I/opt/homebrew/opt/zlib/include $CPPFLAGS"
export PKG_CONFIG_PATH="/opt/homebrew/opt/openblas/lib/pkgconfig:/opt/homebrew/opt/lapack/lib/pkgconfig:/opt/homebrew/opt/jpeg/lib/pkgconfig:/opt/homebrew/opt/zlib/lib/pkgconfig:$PKG_CONFIG_PATH"
export CMAKE_PREFIX_PATH="/opt/homebrew/opt/openblas:/opt/homebrew/opt/lapack:$CMAKE_PREFIX_PATH"
```

These allow compilers and build tools to find the required libraries.

## Installation Process
1. Install system dependencies via Homebrew
2. Install Python 3.12 via uv
3. Create virtual environment
4. Set environment variables for keg-only packages
5. Run `uv pip install -e .[cpu]` to install DimOS with CPU dependencies

## Notes
- CUDA is not available on Apple Silicon Macs, so GPU builds are not supported
- The `set-macos.sh` script automates all these steps
- All packages in `. [cpu]` (onnxruntime, ctransformers, etc.) now compile successfully on ARM
- Tested on macOS 15.7.1 with Apple Silicon

## Testing Results

All README examples tested successfully on Mac ARM:

✅ **pytest -s dimos/** - 706 tests collected, 0 errors (multicast fixed)  
✅ **CONNECTION_TYPE=replay python dimos/robot/unitree_webrtc/unitree_go2.py** - Starts successfully  
✅ **pip install -e .[sim]** and **CONNECTION_TYPE=mujoco python dimos/robot/unitree_webrtc/unitree_go2.py** - Full simulation initializes with Dask cluster, navigation modules, and WebSocket visualization  

## How to Run

### Initial Setup
1. Clone the repository:
   ```bash
   git clone --branch dev --single-branch https://github.com/dimensionalOS/dimos.git
   cd dimos
   ```

2. Run the setup script:
   ```bash
   ./set-macos.sh
   ```

### Running DimOS
1. Activate the virtual environment:
   ```bash
   source .venv/bin/activate
   ```

2. Copy and configure environment variables:
   ```bash
   cp default.env .env
   # Edit .env with your API keys (OpenAI, Claude, Alibaba, etc.)
   ```

3. Test the installation:
   ```bash
   pytest -s dimos/
   ```

4. Run example applications:
   - Test with replay stream: `CONNECTION_TYPE=replay python dimos/robot/unitree_webrtc/unitree_go2.py`
   - Test with simulated robot: `export DISPLAY=:1 && CONNECTION_TYPE=mujoco python dimos/robot/unitree_webrtc/unitree_go2.py`
   - For real robot: Set `ROBOT_IP` and run without CONNECTION_TYPE

5. Run agents (requires API keys):
   ```bash
   export ROBOT_IP=192.168.X.XXX
   python dimos/robot/unitree_webrtc/run_agents2.py
   ```

6. Run the web interface:
   ```bash
   cd dimos/web/dimos_interface
   yarn install
   yarn dev
   ```

### Deactivating Environment
When done, deactivate the virtual environment:
```bash
deactivate
```

## Troubleshooting
- If compilation fails, ensure all Homebrew packages are installed and environment variables are set
- The script checks for Homebrew and uv presence
- Virtual environment can be recreated with `uv venv --python 3.12 --clear` if needed
- For GUI applications, you may need to set `export DISPLAY=:0` or `:1`
