//! Installer paths, `installer.json`, and the redacted append-only action log.

use std::collections::BTreeMap;
use std::ffi::OsString;
use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};

use crate::cli::InstallMode;

pub const SCHEMA: u32 = 1;
pub const UNIT_DIR: &str = "/etc/systemd/system";
pub const SYSCTL_CONF: &str = "/etc/sysctl.d/99-dimos.conf";
pub const MEMLOCK_CONF: &str = "/etc/security/limits.d/99-dimos-memlock.conf";
pub const MULTICAST_UNIT: &str = "dimos-multicast";
pub const JETSON_CLOCKS_UNIT: &str = "dimos-jetson-clocks";

/// Every rc file a block may ever have landed in; `uninstall` sweeps them all.
pub const RC_CANDIDATES: [&str; 6] = [
    ".profile",
    ".bash_profile",
    ".bash_login",
    ".bashrc",
    ".zprofile",
    ".zshrc",
];

/// What `dimos setup` recorded, so `update` and `hardware` re-run against the same install.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Installed {
    pub schema: u32,
    pub installer_version: String,
    pub dimos_version: String,
    pub mode: InstallMode,
    pub dir: PathBuf,
    pub branch: Option<String>,
    pub extras: Vec<String>,
    pub platform: PlatformSummary,
    #[serde(default)]
    pub hardware: BTreeMap<String, HardwareRun>,
    pub last: Option<LastRun>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlatformSummary {
    pub os: String,
    pub distro: String,
    pub version: String,
    pub arch: String,
    pub glibc: Option<String>,
    pub jetson: Option<String>,
}

/// One `hardware <target> setup` run; `update` rebuilds that target's stages from it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HardwareRun {
    pub at: String,
    pub result: String,
    pub robot_ip: Option<String>,
    pub interface: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LastRun {
    pub command: String,
    pub exit_code: i32,
    pub at: String,
}

impl Installed {
    pub fn venv(&self) -> PathBuf {
        venv(&self.dir)
    }

    pub fn venv_python(&self) -> PathBuf {
        venv_python(&self.dir)
    }

    pub fn venv_dimos(&self) -> PathBuf {
        self.venv().join("bin/dimos")
    }
}

/// The one spelling of where a project dir keeps its venv.
pub fn venv(dir: &Path) -> PathBuf {
    dir.join(".venv")
}

pub fn venv_python(dir: &Path) -> PathBuf {
    venv(dir).join("bin/python")
}

fn base(xdg: Option<OsString>, fallback: PathBuf) -> PathBuf {
    match xdg {
        Some(v) if !v.is_empty() => PathBuf::from(v),
        _ => fallback,
    }
}

/// `$XDG_CONFIG_HOME/dimos`, else `~/.config/dimos` — the same dir as dimos/constants.py CONFIG_DIR.
pub fn config_dir(home: &Path) -> PathBuf {
    base(std::env::var_os("XDG_CONFIG_HOME"), home.join(".config")).join("dimos")
}

/// `$XDG_STATE_HOME/dimos`, else `~/.local/state/dimos` — dimos/constants.py STATE_DIR.
pub fn state_dir(home: &Path) -> PathBuf {
    base(
        std::env::var_os("XDG_STATE_HOME"),
        home.join(".local/state"),
    )
    .join("dimos")
}

pub fn installed_bin(home: &Path) -> PathBuf {
    home.join(".local/bin/dimos")
}

pub fn backup_bin(home: &Path) -> PathBuf {
    home.join(".local/bin/dimos.bak")
}

pub fn installer_json(home: &Path) -> PathBuf {
    config_dir(home).join("installer.json")
}

pub fn action_log_path(home: &Path) -> PathBuf {
    state_dir(home).join("installer.jsonl")
}

pub fn unit_path(name: &str) -> PathBuf {
    PathBuf::from(UNIT_DIR).join(format!("{name}.service"))
}

/// The rc files `shell` reads at login, by that shell's own lookup, so a block lands where it counts.
pub fn rc_files(home: &Path, shell: &Path) -> Vec<PathBuf> {
    match shell.file_name().and_then(|n| n.to_str()) {
        Some("zsh") => vec![home.join(".zprofile")],
        // bash -l reads the first of these three and no other; a plain terminal reads .bashrc.
        Some("bash") => vec![
            [".bash_profile", ".bash_login"]
                .iter()
                .map(|n| home.join(n))
                .find(|p| p.exists())
                .unwrap_or_else(|| home.join(".profile")),
            home.join(".bashrc"),
        ],
        _ => vec![home.join(".profile")],
    }
}

pub fn load(home: &Path) -> Result<Option<Installed>> {
    let path = installer_json(home);
    let text = match fs::read_to_string(&path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(e).with_context(|| format!("read {}", path.display())),
    };
    let installed: Installed = serde_json::from_str(&text)
        .with_context(|| format!("{} is not installer schema {SCHEMA}", path.display()))?;
    Ok(Some(installed))
}

/// Write through a sibling `.tmp` so a crash never leaves a half-written installer.json.
pub fn save(home: &Path, installed: &Installed) -> Result<()> {
    let path = installer_json(home);
    let dir = path.parent().context("installer.json has no parent")?;
    fs::create_dir_all(dir).with_context(|| format!("create {}", dir.display()))?;
    let tmp = path.with_extension("json.tmp");
    let mut text = serde_json::to_string_pretty(installed)?;
    text.push('\n');
    fs::write(&tmp, text).with_context(|| format!("write {}", tmp.display()))?;
    fs::rename(&tmp, &path)
        .with_context(|| format!("rename {} -> {}", tmp.display(), path.display()))
}

/// Redacted log view: a Run keeps env KEYS only and a WriteFile keeps a byte count, never contents.
#[derive(Debug, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ActionView<'a> {
    Run {
        argv: &'a [String],
        sudo: bool,
        cwd: Option<&'a Path>,
        env_keys: Vec<&'a str>,
        timeout_s: u64,
    },
    WriteFile {
        path: &'a Path,
        mode: u32,
        bytes: usize,
        sudo: bool,
    },
    EnsureBlock {
        file: &'a Path,
        marker: &'a str,
        present: bool,
    },
    Copy {
        from: &'a Path,
        to: &'a Path,
    },
    Rename {
        from: &'a Path,
        to: &'a Path,
    },
    Remove {
        path: &'a Path,
        sudo: bool,
    },
    VerifySha256 {
        file: &'a Path,
    },
}

#[derive(Debug, Serialize)]
pub struct ActionRecord<'a> {
    pub ts: String,
    pub run: &'a str,
    pub command: &'a str,
    pub stage: &'a str,
    pub action: Option<ActionView<'a>>,
    pub outcome: &'a str,
    pub exit: Option<i32>,
    pub duration_ms: u64,
}

pub struct ActionLog {
    path: PathBuf,
}

impl ActionLog {
    pub fn open(home: &Path) -> Result<ActionLog> {
        let dir = state_dir(home);
        fs::create_dir_all(&dir).with_context(|| format!("create {}", dir.display()))?;
        Ok(ActionLog {
            path: action_log_path(home),
        })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn append(&self, rec: &ActionRecord) -> Result<()> {
        let line = serde_json::to_string(rec)?;
        let mut f = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.path)
            .with_context(|| format!("append to {}", self.path.display()))?;
        writeln!(f, "{line}").with_context(|| format!("append to {}", self.path.display()))
    }
}

pub fn now_iso() -> String {
    chrono::Utc::now().format("%Y-%m-%dT%H:%M:%SZ").to_string()
}

/// `20260901T180140Z-7f3a` — sorts by time and separates concurrent runs.
pub fn run_id() -> String {
    let stamp = chrono::Utc::now().format("%Y%m%dT%H%M%SZ");
    format!("{stamp}-{:04x}", std::process::id() & 0xffff)
}

/// A throwaway HOME: removed on drop, and it owns $XDG_*_HOME for its lifetime so path
/// tests are hermetic on a machine that sets them.
#[cfg(test)]
pub(crate) struct TmpDir {
    dir: PathBuf,
    _lock: std::sync::MutexGuard<'static, ()>,
}

#[cfg(test)]
static XDG_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

#[cfg(test)]
impl TmpDir {
    pub(crate) fn new(tag: &str) -> TmpDir {
        use std::sync::atomic::{AtomicU32, Ordering};
        static N: AtomicU32 = AtomicU32::new(0);
        let lock = XDG_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let n = N.fetch_add(1, Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("dimos-{tag}-{}-{n}", std::process::id()));
        fs::create_dir_all(&dir).expect("create test dir");
        std::env::set_var("XDG_CONFIG_HOME", dir.join(".config"));
        std::env::set_var("XDG_STATE_HOME", dir.join(".local/state"));
        TmpDir { dir, _lock: lock }
    }

    pub(crate) fn path(&self) -> &Path {
        &self.dir
    }
}

#[cfg(test)]
impl Drop for TmpDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.dir);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Installed {
        Installed {
            schema: SCHEMA,
            installer_version: "0.0.14b1".into(),
            dimos_version: "git:aaryan/installer@1fa7cabf".into(),
            mode: InstallMode::Dev,
            dir: PathBuf::from("/home/unitree/dimos"),
            branch: Some("aaryan/installer".into()),
            extras: vec!["unitree".into()],
            platform: PlatformSummary {
                os: "linux".into(),
                distro: "ubuntu".into(),
                version: "20.04".into(),
                arch: "aarch64".into(),
                glibc: Some("2.31".into()),
                jetson: Some("R35.3.1".into()),
            },
            hardware: BTreeMap::new(),
            last: None,
        }
    }

    #[test]
    fn installer_json_round_trips() {
        let home = TmpDir::new("state-roundtrip");
        save(home.path(), &sample()).unwrap();
        assert_eq!(load(home.path()).unwrap(), Some(sample()));
    }

    #[test]
    fn a_hardware_run_survives_the_round_trip_so_update_can_rebuild_its_stages() {
        let home = TmpDir::new("state-hardware");
        let mut installed = sample();
        installed.hardware.insert(
            "g1".into(),
            HardwareRun {
                at: "2026-09-01T18:20:00Z".into(),
                result: "ok".into(),
                robot_ip: Some("192.168.123.161".into()),
                interface: Some("eth0".into()),
            },
        );
        save(home.path(), &installed).unwrap();
        let back = load(home.path()).unwrap().unwrap();
        assert_eq!(back.hardware["g1"].interface.as_deref(), Some("eth0"));
    }

    #[test]
    fn a_file_written_before_the_hardware_field_existed_still_loads() {
        let home = TmpDir::new("state-legacy");
        save(home.path(), &sample()).unwrap();
        let path = installer_json(home.path());
        let text = fs::read_to_string(&path).unwrap();
        let stripped: String = text
            .lines()
            .filter(|l| !l.contains("\"hardware\""))
            .collect::<Vec<_>>()
            .join("\n");
        fs::write(&path, stripped).unwrap();
        assert!(load(home.path()).unwrap().unwrap().hardware.is_empty());
    }

    #[test]
    fn missing_installer_json_is_none_not_an_error() {
        let home = TmpDir::new("state-missing");
        assert_eq!(load(home.path()).unwrap(), None);
    }

    #[test]
    fn dirs_follow_xdg_env_like_constants_py() {
        let fallback = PathBuf::from("/home/u/.config");
        assert_eq!(
            base(Some("/xdg".into()), fallback.clone()),
            PathBuf::from("/xdg")
        );
        assert_eq!(base(Some("".into()), fallback.clone()), fallback);
        assert_eq!(base(None, fallback.clone()), fallback);
    }

    #[test]
    fn save_is_atomic_leaves_no_tmp() {
        let home = TmpDir::new("state-atomic");
        save(home.path(), &sample()).unwrap();
        let left: Vec<String> = fs::read_dir(config_dir(home.path()))
            .unwrap()
            .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
            .collect();
        assert_eq!(left, ["installer.json"]);
    }

    #[test]
    fn log_line_is_one_json_object_per_action_with_env_keys_only() {
        let argv = ["uv".to_string(), "sync".to_string()];
        let rec = ActionRecord {
            ts: "2026-09-01T18:01:42Z".into(),
            run: "20260901T180140Z-7f3a",
            command: "setup",
            stage: "dimos",
            action: Some(ActionView::Run {
                argv: &argv,
                sudo: false,
                cwd: None,
                env_keys: vec!["GIT_LFS_SKIP_SMUDGE"],
                timeout_s: 3600,
            }),
            outcome: "applied",
            exit: Some(0),
            duration_ms: 12,
        };
        let line = serde_json::to_string(&rec).unwrap();
        assert!(line.contains("\"kind\":\"run\""));
        assert!(line.contains("\"env_keys\":[\"GIT_LFS_SKIP_SMUDGE\"]"));
        assert_eq!(line.lines().count(), 1);
    }

    #[test]
    fn write_file_log_view_carries_a_byte_count_not_the_contents() {
        let view = ActionView::WriteFile {
            path: Path::new("/etc/sysctl.d/99-dimos.conf"),
            mode: 0o644,
            bytes: 98,
            sudo: true,
        };
        let line = serde_json::to_string(&view).unwrap();
        assert!(line.contains("\"bytes\":98"));
        assert!(!line.contains("net.core"));
    }

    #[test]
    fn paths_never_under_home_dot_dimos() {
        let home = Path::new("/home/u");
        for p in [
            installer_json(home),
            action_log_path(home),
            installed_bin(home),
            backup_bin(home),
        ] {
            assert!(!p.to_string_lossy().contains("/.dimos"), "{}", p.display());
        }
    }

    #[test]
    fn rc_files_follow_the_login_shells_own_lookup() {
        let home = TmpDir::new("state-rc");
        let at = |name: &str| home.path().join(name);
        assert_eq!(
            rc_files(home.path(), Path::new("/bin/zsh")),
            [at(".zprofile")]
        );
        assert_eq!(
            rc_files(home.path(), Path::new("/bin/bash")),
            [at(".profile"), at(".bashrc")]
        );
        fs::write(at(".bash_profile"), "").unwrap();
        assert_eq!(
            rc_files(home.path(), Path::new("/usr/bin/bash")),
            [at(".bash_profile"), at(".bashrc")]
        );
        assert_eq!(
            rc_files(home.path(), Path::new("/bin/sh")),
            [at(".profile")]
        );
    }

    #[test]
    fn the_action_log_appends_one_line_per_record() {
        let home = TmpDir::new("state-log");
        let log = ActionLog::open(home.path()).unwrap();
        for outcome in ["applied", "already"] {
            log.append(&ActionRecord {
                ts: now_iso(),
                run: "r",
                command: "setup",
                stage: "sysconfig",
                action: None,
                outcome,
                exit: None,
                duration_ms: 0,
            })
            .unwrap();
        }
        let text = fs::read_to_string(log.path()).unwrap();
        assert_eq!(text.lines().count(), 2);
    }

    #[test]
    fn venv_paths_hang_off_the_recorded_dir() {
        let installed = sample();
        assert_eq!(
            installed.venv_dimos(),
            PathBuf::from("/home/unitree/dimos/.venv/bin/dimos")
        );
        assert_eq!(
            installed.venv_python(),
            PathBuf::from("/home/unitree/dimos/.venv/bin/python")
        );
    }
}
