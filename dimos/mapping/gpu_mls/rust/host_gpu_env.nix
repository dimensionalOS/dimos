# nix's glibc does not read ld.so.cache, so the Vulkan loader cannot find the host's NVIDIA driver.
# Hand it just the driver's own libraries (a whole driver directory would shadow nix's wayland and
# friends) plus the two things the Tegra ICD loads behind the loader's back: glvnd's libEGL, which
# dispatches to libEGL_nvidia (without it vk_icdGetInstanceProcAddr returns NULL), and libX11 and
# libXext, which libGLX_nvidia links but nix's search path lacks.
{ pkgs, lib }:
''
          driver=''${XDG_CACHE_HOME:-$HOME/.cache}/dimos-gpu-mls/host-driver
          mkdir -p "$driver"
          for lib in \
            /usr/lib/aarch64-linux-gnu/nvidia/libGLX_nvidia.so* /usr/lib/aarch64-linux-gnu/nvidia/libnv*.so* \
            /usr/lib/aarch64-linux-gnu/tegra/libnv*.so* /usr/lib/aarch64-linux-gnu/tegra-egl/libEGL_nvidia.so* \
            /usr/lib/x86_64-linux-gnu/libGLX_nvidia.so* /usr/lib/x86_64-linux-gnu/libEGL_nvidia.so* \
            /usr/lib/x86_64-linux-gnu/libnvidia-*.so*; do
            [ -e "$lib" ] && ln -sf "$lib" "$driver/"
          done
          for icd in /etc/vulkan/icd.d/nvidia_icd.json /usr/share/vulkan/icd.d/nvidia_icd.json; do
            [ -z "''${VK_ICD_FILENAMES:-}" ] && [ -e "$icd" ] && export VK_ICD_FILENAMES="$icd"
          done
          egl_vendor=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
          [ -z "''${__EGL_VENDOR_LIBRARY_FILENAMES:-}" ] && [ -e "$egl_vendor" ] && export __EGL_VENDOR_LIBRARY_FILENAMES="$egl_vendor"
          export LD_LIBRARY_PATH="$driver:${lib.makeLibraryPath [ pkgs.libglvnd pkgs.libx11 pkgs.libxext pkgs.vulkan-loader ]}''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
''
