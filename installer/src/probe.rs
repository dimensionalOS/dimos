//! Read-only detection: the machine as `Probes`, read once through `capture` and `probe_parse`.

use std::collections::BTreeMap;
use std::fs;
use std::io::Read;
use std::net::Ipv4Addr;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::Duration;

use anyhow::{bail, Context, Result};
use serde::Serialize;

use crate::install_record::{self, Installed, PlatformSummary};
use crate::plan;
use crate::probe_parse::{
    detect_gpu, is_musl_loader, jetpack_for_l4t, memlock_conf_bytes, multicast_ok,
    nvpmodel_is_maxn, parse_device_tree_model, parse_glibc, parse_iface_ipv4,
    parse_nv_tegra_release, parse_os_release, parse_sysctl_value, parse_unit_files, passwd_shell,
    path_lists_dir,
};
use crate::spawn;

/// torch's TLS block needs glibc's static-TLS surplus, raised to 4 slots only in 2.34.
const STATIC_TLS_GLIBC: (u32, u32) = (2, 34);

/// A machine probe answers in milliseconds; a hung one (dead NFS home, wedged dpkg lock) must not hang the run.
const PROBE_TIMEOUT_S: u64 = 20;
/// A login shell sources every rc file the user has (nvm, conda), which can take a while.
const LOGIN_SHELL_TIMEOUT_S: u64 = 60;

#[derive(Debug, Clone, PartialEq, Serialize)]
pub enum Os {
    Linux { id: String, version: String },
    MacOs { version: String },
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub enum Arch {
    X86_64,
    Aarch64,
}

impl Arch {
    pub fn name(self) -> &'static str {
        match self {
            Arch::X86_64 => "x86_64",
            Arch::Aarch64 => "aarch64",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub enum Gpu {
    Nvidia,
    Tegra,
    AppleSilicon,
    None,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub enum PkgManager {
    Apt,
    Brew,
    None,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Jetson {
    pub l4t: String,
    pub jetpack: Option<&'static str>,
    pub model: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct Platform {
    pub os: Os,
    pub arch: Arch,
    /// None on macOS and on musl, where there is no glibc version to report.
    pub glibc: Option<(u32, u32)>,
    pub jetson: Option<Jetson>,
    pub gpu: Gpu,
    pub pkg: PkgManager,
    /// /run/systemd/system exists; false in a container, which has no units to install.
    pub systemd: bool,
    pub home: PathBuf,
    pub user: String,
    pub shell: PathBuf,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct Kernel {
    pub sysctl: BTreeMap<String, u64>,
    pub lo_multicast: bool,
    pub multicast_route: bool,
    pub memlock_conf_bytes: Option<u64>,
    pub nvpmodel_maxn: Option<bool>,
    /// /etc/sysctl.d/99-dimos.conf as it stands, so a re-run plans nothing.
    pub sysctl_conf: Option<String>,
    pub enabled_units: Vec<String>,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct Tools {
    pub uv: Option<PathBuf>,
    pub git: bool,
    pub curl: bool,
    pub nix: bool,
    pub login_path_has_local_bin: bool,
    pub dpkg_status: String,
    pub brew_list: String,
}

#[derive(Debug, Clone)]
pub struct RcFile {
    pub path: PathBuf,
    pub text: String,
}

/// Everything a plan builder may look at; building a stage from anything else is a re-probe.
pub struct Probes {
    pub platform: Platform,
    pub kernel: Kernel,
    pub tools: Tools,
    pub installed: Option<Installed>,
    /// The rc files the user's login shell reads, as they stand, so a block is planned once.
    pub rc: Vec<RcFile>,
    pub ifaces: Vec<(String, Ipv4Addr)>,
    pub current_exe: PathBuf,
}

/// The one bounded read-only spawn: trimmed stdout on exit 0, None on failure or at the deadline.
pub fn capture(
    program: &str,
    args: &[&str],
    env: &[(&str, &str)],
    timeout_s: u64,
) -> Option<String> {
    let mut cmd = Command::new(program);
    cmd.args(args)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null());
    for (key, value) in env {
        cmd.env(key, value);
    }
    let mut child = cmd.spawn().ok()?;
    let mut pipe = child.stdout.take()?;
    // Drained on its own thread so a child writing more than the pipe holds never blocks the poll.
    let reader = std::thread::spawn(move || {
        let mut out = String::new();
        let _ = pipe.read_to_string(&mut out);
        out
    });
    let status = spawn::wait_until(&mut child, Duration::from_secs(timeout_s)).ok()??;
    let out = reader.join().ok()?;
    status.success().then(|| out.trim().to_string())
}

pub fn pkg_manager(os: &Os) -> PkgManager {
    match os {
        Os::MacOs { .. } if which::which("brew").is_ok() => PkgManager::Brew,
        Os::Linux { .. } if which::which("apt-get").is_ok() => PkgManager::Apt,
        _ => PkgManager::None,
    }
}

/// A fresh login shell under `home` tests what the rc files actually do, instead of parsing them.
pub fn login_shell_path(shell: &Path, home: &Path) -> Option<String> {
    capture(
        &plan::text(shell),
        &["-l", "-c", "echo $PATH"],
        &[("PATH", "/usr/bin:/bin"), ("HOME", &plan::text(home))],
        LOGIN_SHELL_TIMEOUT_S,
    )
}

pub fn user_shell(user: &str) -> PathBuf {
    passwd_shell(&fs::read_to_string("/etc/passwd").unwrap_or_default(), user)
        .or_else(|| dscl_shell(user))
        .or_else(|| std::env::var("SHELL").ok())
        .map_or(PathBuf::from("/bin/sh"), PathBuf::from)
}

fn dscl_shell(user: &str) -> Option<String> {
    let out = capture(
        "dscl",
        &[".", "-read", &format!("/Users/{user}"), "UserShell"],
        &[],
        PROBE_TIMEOUT_S,
    )?;
    out.split_whitespace().nth(1).map(str::to_string)
}

impl Platform {
    pub fn detect() -> Result<Platform> {
        let os = read_os()?;
        let arch = read_arch()?;
        let jetson = read_jetson();
        let user = read_user();
        Ok(Platform {
            glibc: read_glibc(&os),
            gpu: detect_gpu(arch, &os, jetson.is_some(), read_nvidia_smi().as_deref()),
            pkg: pkg_manager(&os),
            systemd: Path::new("/run/systemd/system").exists(),
            home: dirs::home_dir().context("no home directory: set $HOME and re-run")?,
            shell: user_shell(&user),
            os,
            arch,
            jetson,
            user,
        })
    }

    /// The release-asset suffix: `dimos-<target>` and `dimos-<target>.sha256`.
    pub fn target(&self) -> Result<&'static str> {
        match (&self.os, self.arch) {
            (Os::Linux { .. }, Arch::X86_64) => Ok("x86_64-linux-musl"),
            (Os::Linux { .. }, Arch::Aarch64) => Ok("aarch64-linux-musl"),
            (Os::MacOs { .. }, Arch::Aarch64) => Ok("aarch64-apple-darwin"),
            (Os::MacOs { .. }, Arch::X86_64) => bail!(
                "no installer build for Intel macOS; install DimOS by hand:\n  \
                 curl -LsSf https://astral.sh/uv/install.sh | sh\n  \
                 uv venv --python 3.12 && uv pip install 'dimos[base]'"
            ),
        }
    }

    pub fn is_jetson(&self) -> bool {
        self.jetson.is_some()
    }

    pub fn static_tls_risk(&self) -> bool {
        self.arch == Arch::Aarch64
            && matches!(self.os, Os::Linux { .. })
            && self.glibc.is_some_and(|v| v < STATIC_TLS_GLIBC)
    }

    pub fn summary(&self) -> PlatformSummary {
        let (os, distro, version) = match &self.os {
            Os::Linux { id, version } => ("linux", id.clone(), version.clone()),
            Os::MacOs { version } => ("macos", "macos".to_string(), version.clone()),
        };
        PlatformSummary {
            os: os.to_string(),
            distro,
            version,
            arch: self.arch.name().to_string(),
            glibc: self.glibc.map(|(a, b)| format!("{a}.{b}")),
            jetson: self.jetson.as_ref().map(|j| j.l4t.clone()),
        }
    }
}

impl Kernel {
    pub fn detect(sysctl_keys: &[&str], user: &str) -> Kernel {
        if cfg!(target_os = "macos") {
            return Kernel::default();
        }
        let (lo_multicast, multicast_route) = multicast_ok(
            &run_text("ip", &["link", "show", "lo"]),
            &run_text("ip", &["route", "show", "224.0.0.0/4"]),
        );
        Kernel {
            sysctl: read_sysctl(sysctl_keys),
            lo_multicast,
            multicast_route,
            memlock_conf_bytes: memlock_conf_bytes(
                &fs::read_to_string(install_record::MEMLOCK_CONF).unwrap_or_default(),
                user,
            ),
            nvpmodel_maxn: capture("nvpmodel", &["-q"], &[], PROBE_TIMEOUT_S)
                .map(|t| nvpmodel_is_maxn(&t)),
            sysctl_conf: fs::read_to_string(install_record::SYSCTL_CONF).ok(),
            enabled_units: parse_unit_files(&run_text(
                "systemctl",
                &["list-unit-files", "--state=enabled", "--no-legend"],
            )),
        }
    }
}

impl Tools {
    pub fn detect(home: &Path, shell: &Path, pkg: PkgManager) -> Tools {
        let vendored_uv = home.join(".local/bin/uv");
        Tools {
            uv: which::which("uv")
                .ok()
                .or_else(|| vendored_uv.is_file().then_some(vendored_uv)),
            git: which::which("git").is_ok(),
            curl: which::which("curl").is_ok(),
            nix: which::which("nix").is_ok(),
            login_path_has_local_bin: login_shell_path(shell, home)
                .is_some_and(|path| path_lists_dir(&path, &home.join(".local/bin"))),
            dpkg_status: match pkg {
                PkgManager::Apt => run_text("dpkg-query", &["-W", "-f=${Package} ${Status}\n"]),
                _ => String::new(),
            },
            brew_list: match pkg {
                PkgManager::Brew => run_text("brew", &["list", "--versions"]),
                _ => String::new(),
            },
        }
    }
}

impl Probes {
    pub fn detect(sysctl_keys: &[&str], home: &Path) -> Result<Probes> {
        let platform = Platform::detect()?;
        let installed = install_record::load(home)?;
        Ok(Probes {
            kernel: Kernel::detect(sysctl_keys, &platform.user),
            tools: Tools::detect(home, &platform.shell, platform.pkg),
            rc: read_rc(home, &platform.shell),
            ifaces: parse_iface_ipv4(&read_ifaces(&platform.os), &platform.os),
            current_exe: std::env::current_exe().unwrap_or_default(),
            platform,
            installed,
        })
    }
}

fn read_os() -> Result<Os> {
    if cfg!(target_os = "macos") {
        return Ok(Os::MacOs {
            version: run_text("sw_vers", &["-productVersion"]).trim().to_string(),
        });
    }
    let text = fs::read_to_string("/etc/os-release")
        .context("read /etc/os-release: dimos supports Linux distributions that ship it")?;
    let (id, version) = parse_os_release(&text);
    Ok(Os::Linux { id, version })
}

fn read_arch() -> Result<Arch> {
    match std::env::consts::ARCH {
        "x86_64" => Ok(Arch::X86_64),
        "aarch64" => Ok(Arch::Aarch64),
        other => bail!("unsupported CPU {other}: dimos ships x86_64 and aarch64 only"),
    }
}

fn read_glibc(os: &Os) -> Option<(u32, u32)> {
    if matches!(os, Os::MacOs { .. }) || musl_loader_present() {
        return None;
    }
    let out = capture("ldd", &["--version"], &[], PROBE_TIMEOUT_S)?;
    parse_glibc(out.lines().next()?)
}

fn musl_loader_present() -> bool {
    fs::read_dir("/lib").is_ok_and(|dir| {
        dir.flatten()
            .any(|e| is_musl_loader(&e.file_name().to_string_lossy()))
    })
}

fn read_jetson() -> Option<Jetson> {
    let l4t = parse_nv_tegra_release(&fs::read_to_string("/etc/nv_tegra_release").ok()?)?;
    let model = fs::read_to_string("/proc/device-tree/model")
        .map(|t| parse_device_tree_model(&t))
        .unwrap_or_default();
    Some(Jetson {
        jetpack: jetpack_for_l4t(&l4t),
        l4t,
        model,
    })
}

fn read_nvidia_smi() -> Option<String> {
    capture("nvidia-smi", &[], &[], PROBE_TIMEOUT_S)
}

fn read_user() -> String {
    std::env::var("USER")
        .or_else(|_| std::env::var("LOGNAME"))
        .unwrap_or_else(|_| run_text("id", &["-un"]).trim().to_string())
}

fn read_sysctl(keys: &[&str]) -> BTreeMap<String, u64> {
    keys.iter()
        .filter_map(|key| {
            let text = capture("sysctl", &["-n", key], &[], PROBE_TIMEOUT_S)?;
            Some(((*key).to_string(), parse_sysctl_value(&text)?))
        })
        .collect()
}

/// An absent rc file reads as empty, so the block that creates it is still planned once.
fn read_rc(home: &Path, shell: &Path) -> Vec<RcFile> {
    install_record::rc_files(home, shell)
        .into_iter()
        .map(|path| {
            let text = fs::read_to_string(&path).unwrap_or_default();
            RcFile { path, text }
        })
        .collect()
}

fn read_ifaces(os: &Os) -> String {
    match os {
        Os::Linux { .. } => run_text("ip", &["-o", "-4", "addr", "show"]),
        Os::MacOs { .. } => run_text("ifconfig", &["-a"]),
    }
}

fn run_text(program: &str, args: &[&str]) -> String {
    capture(program, args, &[], PROBE_TIMEOUT_S).unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::probe_parse::path_lists_dir;

    fn ubuntu() -> Os {
        Os::Linux {
            id: "ubuntu".into(),
            version: "20.04".into(),
        }
    }

    #[test]
    fn target_suffix_per_os_arch_and_intel_mac_is_err() {
        let platform = |os, arch| Platform {
            os,
            arch,
            glibc: None,
            jetson: None,
            gpu: Gpu::None,
            pkg: PkgManager::None,
            systemd: false,
            home: PathBuf::from("/home/u"),
            user: "u".into(),
            shell: PathBuf::from("/bin/sh"),
        };
        let mac = || Os::MacOs {
            version: "15.0".into(),
        };
        assert_eq!(
            platform(ubuntu(), Arch::Aarch64).target().unwrap(),
            "aarch64-linux-musl"
        );
        assert_eq!(
            platform(ubuntu(), Arch::X86_64).target().unwrap(),
            "x86_64-linux-musl"
        );
        assert_eq!(
            platform(mac(), Arch::Aarch64).target().unwrap(),
            "aarch64-apple-darwin"
        );
        let err = format!("{:#}", platform(mac(), Arch::X86_64).target().unwrap_err());
        assert!(err.contains("uv pip install"), "{err}");
    }

    #[test]
    fn static_tls_risk_only_on_aarch64_below_2_34() {
        let with = |arch, glibc| Platform {
            os: ubuntu(),
            arch,
            glibc,
            jetson: None,
            gpu: Gpu::None,
            pkg: PkgManager::None,
            systemd: false,
            home: PathBuf::from("/home/u"),
            user: "u".into(),
            shell: PathBuf::from("/bin/sh"),
        };
        assert!(with(Arch::Aarch64, Some((2, 31))).static_tls_risk());
        assert!(!with(Arch::Aarch64, Some((2, 35))).static_tls_risk());
        assert!(!with(Arch::X86_64, Some((2, 31))).static_tls_risk());
        assert!(!with(Arch::Aarch64, None).static_tls_risk());
    }

    #[test]
    fn capture_returns_stdout_on_success_and_none_on_failure_or_at_the_deadline() {
        assert_eq!(capture("echo", &["hi"], &[], 5).as_deref(), Some("hi"));
        assert_eq!(capture("false", &[], &[], 5), None);
        assert_eq!(capture("sleep", &["5"], &[], 1), None);
        assert_eq!(capture("/nonexistent/dimos-probe", &[], &[], 1), None);
    }

    #[test]
    fn capture_drains_more_output_than_a_pipe_holds() {
        let big = capture("yes", &["0123456789abcdef"], &[], 1);
        assert!(
            big.is_none(),
            "yes never exits, so the deadline must kill it"
        );
        let text = capture("sh", &["-c", "yes 0123456789 | head -c 200000"], &[], 10)
            .expect("200 KB of stdout is read, not deadlocked on");
        assert!(text.len() > 190_000, "got {} bytes", text.len());
    }

    /// The rc files `install_record::rc_files` picks are the ones the same shell reads in login mode.
    #[test]
    fn rc_files_and_the_login_shell_probe_agree() {
        for shell in ["/bin/bash", "/bin/zsh"].map(Path::new) {
            if !shell.exists() {
                continue;
            }
            let home = install_record::TmpDir::new("probe-rc");
            let lines = ["export PATH=\"$HOME/.local/bin:$PATH\"".to_string()];
            for file in install_record::rc_files(home.path(), shell) {
                let (text, _) = plan::ensure_block("", "path", &lines);
                fs::write(&file, text).expect("write rc fixture");
            }
            let path = login_shell_path(shell, home.path()).expect("the login shell runs");
            assert!(
                path_lists_dir(&path, &home.path().join(".local/bin")),
                "{}: {path}",
                shell.display()
            );
        }
    }
}
