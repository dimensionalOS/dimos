//! Read-only detection. The I/O lives in a few `read_*` helpers; every parser is pure over
//! a `&str` so a fixture tests it without the machine it came from.

use std::collections::BTreeMap;
use std::fs;
use std::net::Ipv4Addr;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

use anyhow::{bail, Context, Result};
use serde::Serialize;

use crate::state::{self, Installed, PlatformSummary};

/// L4T release prefix -> JetPack, from NVIDIA's L4T archive; a longer prefix must come first.
const JETPACK: [(&str, &str); 4] = [
    ("R35.3.1", "5.1.1"),
    ("R35.4.1", "5.1.2"),
    ("R36.3", "6.0"),
    ("R36.4", "6.2"),
];

/// torch's TLS block needs glibc's static-TLS surplus, raised to 4 slots only in 2.34.
const STATIC_TLS_GLIBC: (u32, u32) = (2, 34);

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
    pub rc: Vec<RcFile>,
    pub ifaces: Vec<(String, Ipv4Addr)>,
    /// `<dir>/.env` when a DimOS install is recorded, so the g1 stage plans nothing twice.
    pub dotenv: Option<String>,
    pub current_exe: PathBuf,
}

pub fn parse_os_release(text: &str) -> (String, String) {
    let field = |key: &str| {
        text.lines()
            .find_map(|l| l.trim().strip_prefix(key))
            .map(|v| v.trim_matches('"').to_string())
            .unwrap_or_default()
    };
    (field("ID="), field("VERSION_ID="))
}

/// `# R35 (release), REVISION: 3.1, GCID: ...` -> `R35.3.1`.
pub fn parse_nv_tegra_release(text: &str) -> Option<String> {
    let major = text.split_whitespace().find(|t| {
        t.len() > 1 && t.starts_with('R') && t[1..].chars().all(|c| c.is_ascii_digit())
    })?;
    let revision = text.split("REVISION:").nth(1)?.split(',').next()?.trim();
    Some(format!("{major}.{revision}"))
}

pub fn jetpack_for_l4t(l4t: &str) -> Option<&'static str> {
    JETPACK
        .iter()
        .find(|(prefix, _)| l4t.starts_with(prefix))
        .map(|(_, jetpack)| *jetpack)
}

/// `ldd (Ubuntu GLIBC 2.31-0ubuntu9.9) 2.31` -> (2, 31).
pub fn parse_glibc(ldd_first_line: &str) -> Option<(u32, u32)> {
    let last = ldd_first_line.split_whitespace().last()?;
    let (major, rest) = last.split_once('.')?;
    let minor: String = rest.chars().take_while(char::is_ascii_digit).collect();
    Some((major.parse().ok()?, minor.parse().ok()?))
}

pub fn is_musl_loader(file_name: &str) -> bool {
    file_name.starts_with("ld-musl-") && file_name.ends_with(".so.1")
}

pub fn parse_sysctl_value(text: &str) -> Option<u64> {
    text.rsplit('=')
        .next()?
        .split_whitespace()
        .next()?
        .parse()
        .ok()
}

/// (`lo` carries MULTICAST, a 224.0.0.0/4 route exists) — both are needed for LCM on one host.
pub fn multicast_ok(ip_link_lo: &str, ip_route_224: &str) -> (bool, bool) {
    (
        ip_link_lo.contains("MULTICAST"),
        ip_route_224.contains("224.0.0.0/4"),
    )
}

/// The largest memlock line for `user` or `*`, in bytes; limits.d counts in KiB.
pub fn memlock_conf_bytes(limits_conf: &str, user: &str) -> Option<u64> {
    limits_conf
        .lines()
        .filter(|l| !l.trim_start().starts_with('#'))
        .filter_map(|l| {
            let f: Vec<&str> = l.split_whitespace().collect();
            (f.len() == 4 && (f[0] == user || f[0] == "*") && f[2] == "memlock").then(|| f[3])
        })
        .filter_map(|value| match value {
            "unlimited" => Some(u64::MAX),
            kib => kib.parse::<u64>().ok().map(|k| k * 1024),
        })
        .max()
}

pub fn nvpmodel_is_maxn(nvpmodel_q: &str) -> bool {
    nvpmodel_q.contains("MAXN") || nvpmodel_q.contains("ID=0") || nvpmodel_q.contains("ID: 0")
}

/// /proc/device-tree strings are NUL-terminated.
pub fn parse_device_tree_model(raw: &str) -> String {
    raw.trim_end_matches('\0').trim().to_string()
}

/// First token of each `systemctl list-unit-files --state=enabled --no-legend` line.
pub fn parse_unit_files(text: &str) -> Vec<String> {
    text.lines()
        .filter_map(|l| l.split_whitespace().next())
        .map(str::to_string)
        .collect()
}

pub fn parse_iface_ipv4(text: &str, os: &Os) -> Vec<(String, Ipv4Addr)> {
    match os {
        Os::Linux { .. } => parse_ip_o_4(text),
        Os::MacOs { .. } => parse_ifconfig(text),
    }
}

fn parse_ip_o_4(text: &str) -> Vec<(String, Ipv4Addr)> {
    text.lines()
        .filter_map(|line| {
            let fields: Vec<&str> = line.split_whitespace().collect();
            let at = fields.iter().position(|f| *f == "inet")?;
            let addr = fields.get(at + 1)?.split('/').next()?;
            Some((fields.get(1)?.to_string(), addr.parse().ok()?))
        })
        .collect()
}

fn parse_ifconfig(text: &str) -> Vec<(String, Ipv4Addr)> {
    let mut found = Vec::new();
    let mut iface = String::new();
    for line in text.lines() {
        if let Some(name) = line
            .split(':')
            .next()
            .filter(|_| !line.starts_with(char::is_whitespace))
        {
            iface = name.to_string();
        }
        if let Some(rest) = line.trim_start().strip_prefix("inet ") {
            if let Ok(addr) = rest.split_whitespace().next().unwrap_or("").parse() {
                found.push((iface.clone(), addr));
            }
        }
    }
    found
}

/// Tegra wins on a Jetson, which ships no nvidia-smi on JetPack 5.
pub fn detect_gpu(arch: Arch, os: &Os, jetson: bool, nvidia_smi: Option<&str>) -> Gpu {
    if jetson {
        return Gpu::Tegra;
    }
    match os {
        Os::MacOs { .. } if arch == Arch::Aarch64 => Gpu::AppleSilicon,
        _ if nvidia_smi.is_some_and(|t| t.contains("NVIDIA-SMI")) => Gpu::Nvidia,
        _ => Gpu::None,
    }
}

pub fn pkg_manager(os: &Os) -> PkgManager {
    match os {
        Os::MacOs { .. } if which::which("brew").is_ok() => PkgManager::Brew,
        Os::Linux { .. } if which::which("apt-get").is_ok() => PkgManager::Apt,
        _ => PkgManager::None,
    }
}

/// `dir` must be a whole PATH entry: `/home/u/.local/bin2` must not satisfy `/home/u/.local/bin`.
fn path_lists_dir(path: &str, dir: &Path) -> bool {
    path.split(':').any(|entry| Path::new(entry) == dir)
}

fn passwd_shell(passwd: &str, user: &str) -> Option<String> {
    passwd
        .lines()
        .find(|l| l.starts_with(&format!("{user}:")))
        .and_then(|l| l.rsplit(':').next())
        .map(str::to_string)
}

/// A fresh login shell tests what the rc files actually do, instead of parsing them.
pub fn login_shell_path(shell: &Path) -> Option<String> {
    let out = Command::new(shell)
        .args(["-l", "-c", "echo $PATH"])
        .env("PATH", "/usr/bin:/bin")
        .stdin(Stdio::null())
        .stderr(Stdio::null())
        .output()
        .ok()?;
    out.status
        .success()
        .then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
}

pub fn user_shell(user: &str) -> PathBuf {
    passwd_shell(&fs::read_to_string("/etc/passwd").unwrap_or_default(), user)
        .or_else(|| dscl_shell(user))
        .or_else(|| std::env::var("SHELL").ok())
        .map_or(PathBuf::from("/bin/sh"), PathBuf::from)
}

fn dscl_shell(user: &str) -> Option<String> {
    let out = run_text_opt(
        "dscl",
        &[".", "-read", &format!("/Users/{user}"), "UserShell"],
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
                &fs::read_to_string(state::MEMLOCK_CONF).unwrap_or_default(),
                user,
            ),
            nvpmodel_maxn: run_text_opt("nvpmodel", &["-q"]).map(|t| nvpmodel_is_maxn(&t)),
            sysctl_conf: fs::read_to_string(state::SYSCTL_CONF).ok(),
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
            login_path_has_local_bin: login_shell_path(shell)
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
        let installed = state::load(home)?;
        Ok(Probes {
            kernel: Kernel::detect(sysctl_keys, &platform.user),
            tools: Tools::detect(home, &platform.shell, platform.pkg),
            rc: read_rc(home),
            ifaces: parse_iface_ipv4(&read_ifaces(&platform.os), &platform.os),
            dotenv: installed
                .as_ref()
                .and_then(|i| fs::read_to_string(i.dir.join(".env")).ok()),
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
    let out = run_text_opt("ldd", &["--version"])?;
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
    run_text_opt("nvidia-smi", &[])
}

fn read_user() -> String {
    std::env::var("USER")
        .or_else(|_| std::env::var("LOGNAME"))
        .unwrap_or_else(|_| run_text("id", &["-un"]).trim().to_string())
}

fn read_sysctl(keys: &[&str]) -> BTreeMap<String, u64> {
    keys.iter()
        .filter_map(|key| {
            let text = run_text_opt("sysctl", &["-n", key])?;
            Some(((*key).to_string(), parse_sysctl_value(&text)?))
        })
        .collect()
}

fn read_rc(home: &Path) -> Vec<RcFile> {
    state::rc_files(home)
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
    run_text_opt(program, args).unwrap_or_default()
}

fn run_text_opt(program: &str, args: &[&str]) -> Option<String> {
    let out = Command::new(program)
        .args(args)
        .stdin(Stdio::null())
        .stderr(Stdio::null())
        .output()
        .ok()?;
    out.status
        .success()
        .then(|| String::from_utf8_lossy(&out.stdout).into_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    const UBUNTU_20_04: &str = "NAME=\"Ubuntu\"\nVERSION_ID=\"20.04\"\nID=ubuntu\nID_LIKE=debian\n";
    const R35: &str = "# R35 (release), REVISION: 3.1, GCID: 32798077, BOARD: t186ref\n";
    const R36: &str = "# R36 (release), REVISION: 4.3, GCID: 38968081, BOARD: generic\n";

    fn ubuntu() -> Os {
        Os::Linux {
            id: "ubuntu".into(),
            version: "20.04".into(),
        }
    }

    #[test]
    fn os_release_ubuntu_20_04_gives_id_and_version() {
        assert_eq!(
            parse_os_release(UBUNTU_20_04),
            ("ubuntu".to_string(), "20.04".to_string())
        );
    }

    #[test]
    fn nv_tegra_release_r35_3_1_parses_to_jetpack_5_1_1() {
        let l4t = parse_nv_tegra_release(R35).unwrap();
        assert_eq!(l4t, "R35.3.1");
        assert_eq!(jetpack_for_l4t(&l4t), Some("5.1.1"));
    }

    #[test]
    fn r36_revision_4_3_is_jetpack_6_2() {
        let l4t = parse_nv_tegra_release(R36).unwrap();
        assert_eq!(l4t, "R36.4.3");
        assert_eq!(jetpack_for_l4t(&l4t), Some("6.2"));
    }

    #[test]
    fn an_unknown_l4t_has_no_jetpack_rather_than_a_guess() {
        assert_eq!(jetpack_for_l4t("R32.7.1"), None);
    }

    #[test]
    fn jetson_is_detected_without_nvidia_smi_as_tegra() {
        assert_eq!(detect_gpu(Arch::Aarch64, &ubuntu(), true, None), Gpu::Tegra);
        assert_eq!(detect_gpu(Arch::Aarch64, &ubuntu(), false, None), Gpu::None);
        assert_eq!(
            detect_gpu(
                Arch::X86_64,
                &ubuntu(),
                false,
                Some("NVIDIA-SMI 535.183.01  Driver Version: 535.183.01")
            ),
            Gpu::Nvidia
        );
        assert_eq!(
            detect_gpu(
                Arch::Aarch64,
                &Os::MacOs {
                    version: "15.0".into()
                },
                false,
                None
            ),
            Gpu::AppleSilicon
        );
    }

    #[test]
    fn glibc_parsed_from_ubuntu_ldd_first_line() {
        assert_eq!(
            parse_glibc("ldd (Ubuntu GLIBC 2.31-0ubuntu9.9) 2.31"),
            Some((2, 31))
        );
        assert_eq!(parse_glibc("ldd (GNU libc) 2.39"), Some((2, 39)));
    }

    #[test]
    fn musl_loader_present_means_glibc_none() {
        assert!(is_musl_loader("ld-musl-aarch64.so.1"));
        assert!(is_musl_loader("ld-musl-x86_64.so.1"));
        assert!(!is_musl_loader("ld-linux-aarch64.so.1"));
    }

    #[test]
    fn multicast_needed_when_lo_lacks_flag_or_route_missing() {
        let up = "1: lo: <LOOPBACK,MULTICAST,UP,LOWER_UP> mtu 65536";
        let bare = "1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536";
        assert_eq!(
            multicast_ok(up, "224.0.0.0/4 dev lo scope link"),
            (true, true)
        );
        assert_eq!(multicast_ok(bare, ""), (false, false));
    }

    #[test]
    fn memlock_conf_accepts_user_or_star_line_in_kib() {
        let conf = "# dimos\nunitree hard memlock 65536\n* soft memlock 32768\n";
        assert_eq!(memlock_conf_bytes(conf, "unitree"), Some(65536 * 1024));
        assert_eq!(memlock_conf_bytes(conf, "other"), Some(32768 * 1024));
        assert_eq!(memlock_conf_bytes("# nothing\n", "unitree"), None);
    }

    #[test]
    fn nvpmodel_maxn_recognized_in_all_three_spellings() {
        assert!(nvpmodel_is_maxn("NV Power Mode: MAXN\n0\n"));
        assert!(nvpmodel_is_maxn("NVPM VERB: PM_ID=0\n"));
        assert!(nvpmodel_is_maxn("NV Power Mode ID: 0\n"));
        assert!(!nvpmodel_is_maxn("NV Power Mode: 15W\n2\n"));
    }

    #[test]
    fn sysctl_value_parses_bare_and_key_equals_forms() {
        assert_eq!(parse_sysctl_value("212992\n"), Some(212992));
        assert_eq!(
            parse_sysctl_value("net.core.rmem_max = 67108864\n"),
            Some(67108864)
        );
    }

    #[test]
    fn login_path_probe_matches_home_local_bin_only() {
        let home = Path::new("/home/u");
        assert!(path_lists_dir(
            "/usr/bin:/home/u/.local/bin",
            &home.join(".local/bin")
        ));
        assert!(!path_lists_dir(
            "/usr/bin:/home/u/.local/bin2",
            &home.join(".local/bin")
        ));
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
    fn ip_o_4_addr_gives_one_ipv4_per_interface() {
        let text = "1: lo    inet 127.0.0.1/8 scope host lo\\       valid_lft forever\n\
                    3: eth0    inet 192.168.123.51/24 brd 192.168.123.255 scope global eth0\\   valid\n";
        assert_eq!(
            parse_iface_ipv4(text, &ubuntu()),
            [
                ("lo".to_string(), Ipv4Addr::new(127, 0, 0, 1)),
                ("eth0".to_string(), Ipv4Addr::new(192, 168, 123, 51)),
            ]
        );
    }

    #[test]
    fn ifconfig_pairs_each_inet_with_the_interface_above_it() {
        let text = "lo0: flags=8049<UP,LOOPBACK> mtu 16384\n\
                    \tinet 127.0.0.1 netmask 0xff000000\n\
                    en0: flags=8863<UP,BROADCAST> mtu 1500\n\
                    \tinet6 fe80::1%en0 prefixlen 64\n\
                    \tinet 10.0.0.7 netmask 0xffffff00 broadcast 10.0.0.255\n";
        let mac = Os::MacOs {
            version: "15.0".into(),
        };
        assert_eq!(
            parse_iface_ipv4(text, &mac),
            [
                ("lo0".to_string(), Ipv4Addr::new(127, 0, 0, 1)),
                ("en0".to_string(), Ipv4Addr::new(10, 0, 0, 7)),
            ]
        );
    }

    #[test]
    fn unit_files_are_the_first_token_of_each_line() {
        let text = "dimos-multicast.service                enabled\nssh.service   enabled\n";
        assert_eq!(
            parse_unit_files(text),
            ["dimos-multicast.service", "ssh.service"]
        );
    }

    #[test]
    fn passwd_shell_reads_the_last_field_of_the_users_line() {
        let passwd =
            "root:x:0:0:root:/root:/bin/bash\nunitree:x:1000:1000::/home/unitree:/bin/zsh\n";
        assert_eq!(
            passwd_shell(passwd, "unitree"),
            Some("/bin/zsh".to_string())
        );
        assert_eq!(passwd_shell(passwd, "nobody"), None);
    }

    #[test]
    fn device_tree_model_drops_the_trailing_nul() {
        assert_eq!(
            parse_device_tree_model("NVIDIA Jetson Orin NX Engineering Reference\0"),
            "NVIDIA Jetson Orin NX Engineering Reference"
        );
    }
}
