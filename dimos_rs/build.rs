fn main() {
    // Use pkg-config to find LCM library and configure linker paths
    match pkg_config::Config::new().probe("lcm") {
        Ok(lcm) => {
            // Output library search paths for the linker
            for path in lcm.link_paths.iter() {
                println!("cargo:rustc-link-search=native={}", path.display());
            }
            // Also output the library name (though lcm crate should handle this)
            println!("cargo:rustc-link-lib=dylib=lcm");
        }
        Err(e) => {
            // If pkg-config fails, try to manually set the path from environment
            // This is common in Nix environments
            if let Ok(pkg_config_path) = std::env::var("PKG_CONFIG_PATH") {
                eprintln!(
                    "Warning: pkg-config failed: {}. PKG_CONFIG_PATH={}",
                    e, pkg_config_path
                );
            }
            // Try to find LCM library in common Nix store locations
            if let Ok(ld_library_path) = std::env::var("LD_LIBRARY_PATH") {
                for path in ld_library_path.split(':') {
                    if path.contains("lcm") {
                        println!("cargo:rustc-link-search=native={}", path);
                        println!("cargo:rustc-link-lib=dylib=lcm");
                        break;
                    }
                }
            }
            // Fallback: try to extract from pkg-config output directly
            if let Ok(output) = std::process::Command::new("pkg-config")
                .args(&["--libs-only-L", "lcm"])
                .output()
            {
                if output.status.success() {
                    let output_str = String::from_utf8_lossy(&output.stdout);
                    for part in output_str.split_whitespace() {
                        if part.starts_with("-L") {
                            let path = &part[2..];
                            println!("cargo:rustc-link-search=native={}", path);
                        }
                    }
                    println!("cargo:rustc-link-lib=dylib=lcm");
                }
            }
        }
    }
}
