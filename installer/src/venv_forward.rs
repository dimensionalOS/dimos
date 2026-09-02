//! Unknown verbs run the DimOS Python CLI. The venv's `dimos` is resolved to an absolute path
//! and exec'd in place, so `dimos run <blueprint>` works without the venv on PATH.

use std::convert::Infallible;
use std::ffi::OsString;
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{bail, Context, Result};

use crate::install_record::Installed;

/// Verbs the installer owns; `dimos/cli/installer_cli.py` FORWARDED mirrors this literally.
pub const RESERVED: &[&str] = &["setup", "update", "service", "uninstall", "robot"];

pub const NO_VENV_HINT: &str = "no DimOS install found: run `dimos setup` \
(looked in the installer.json dir, $VIRTUAL_ENV, then ./.venv)";

/// Set on the child so a Python CLI that forwards back here exits 2 instead of looping.
const GUARD: &str = "DIMOS_FORWARDED";

/// The venv `dimos`: recorded install, then `$VIRTUAL_ENV`, then `./.venv` — first that exists.
pub fn venv_dimos(
    installed: Option<&Installed>,
    virtual_env: Option<PathBuf>,
    cwd: &Path,
) -> Option<PathBuf> {
    candidates(installed, virtual_env, cwd)
        .into_iter()
        .find(|path| path.is_file())
}

fn candidates(
    installed: Option<&Installed>,
    virtual_env: Option<PathBuf>,
    cwd: &Path,
) -> Vec<PathBuf> {
    [
        installed.map(Installed::venv_dimos),
        virtual_env.map(|env| env.join("bin/dimos")),
        Some(cwd.join(".venv/bin/dimos")),
    ]
    .into_iter()
    .flatten()
    .collect()
}

/// Replace this process with the venv `dimos`; returns only when exec itself fails.
pub fn exec(path: &Path, args: &[OsString]) -> Result<Infallible> {
    let current_exe = std::env::current_exe().context("cannot resolve the running binary")?;
    if is_self(path, &current_exe) {
        bail!(
            "{} is this installer, not the DimOS Python CLI: run `dimos setup` to rebuild the venv",
            path.display()
        );
    }
    let err = Command::new(path).args(args).env(GUARD, "1").exec();
    Err(err).with_context(|| format!("exec {} failed", path.display()))
}

/// Compare resolved paths: `~/.local/bin/dimos` symlinked into the venv would exec forever.
fn is_self(target: &Path, current_exe: &Path) -> bool {
    let resolve = |path: &Path| path.canonicalize().unwrap_or_else(|_| path.to_path_buf());
    resolve(target) == resolve(current_exe)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    use crate::cli::InstallMode;
    use crate::install_record::{PlatformSummary, SCHEMA};

    fn installed_at(dir: &str) -> Installed {
        Installed {
            schema: SCHEMA,
            installer_version: "0.0.14b1".into(),
            dimos_version: "0.0.14b1".into(),
            mode: InstallMode::Library,
            dir: PathBuf::from(dir),
            branch: None,
            extras: vec![],
            platform: PlatformSummary {
                os: "linux".into(),
                distro: "ubuntu".into(),
                version: "22.04".into(),
                arch: "x86_64".into(),
                glibc: Some("2.35".into()),
                jetson: None,
            },
            hardware: Default::default(),
            last: None,
        }
    }

    /// `FORWARDED = ("setup", ...)` -> the quoted names, wrapped across lines or not.
    fn forwarded_tuple(source: &str) -> Vec<String> {
        let start = source
            .find("FORWARDED")
            .expect("installer_cli.py defines FORWARDED");
        let open = start + source[start..].find('(').expect("FORWARDED is a tuple");
        let close = open + source[open..].find(')').expect("FORWARDED tuple closes");
        source[open..close]
            .split('"')
            .skip(1)
            .step_by(2)
            .map(String::from)
            .collect()
    }

    #[test]
    fn venv_dimos_prefers_installer_json_then_virtual_env_then_cwd() {
        let installed = installed_at("/opt/dimos");
        let order = candidates(
            Some(&installed),
            Some(PathBuf::from("/env")),
            Path::new("/work"),
        );
        assert_eq!(
            order,
            vec![
                PathBuf::from("/opt/dimos/.venv/bin/dimos"),
                PathBuf::from("/env/bin/dimos"),
                PathBuf::from("/work/.venv/bin/dimos"),
            ]
        );
    }

    #[test]
    fn absent_sources_drop_out_of_the_order_instead_of_shifting_it() {
        let order = candidates(None, Some(PathBuf::from("/env")), Path::new("/work"));
        assert_eq!(
            order,
            vec![
                PathBuf::from("/env/bin/dimos"),
                PathBuf::from("/work/.venv/bin/dimos"),
            ]
        );
    }

    #[test]
    fn none_found_is_none_so_main_exits_2_with_setup_hint() {
        let found = venv_dimos(None, None, Path::new("/nonexistent-dimos-forward-test"));
        assert_eq!(found, None);
        assert!(NO_VENV_HINT.contains("dimos setup"));
        assert!(NO_VENV_HINT.contains("$VIRTUAL_ENV"));
    }

    #[test]
    fn execing_the_running_binary_is_refused_instead_of_looping() {
        let me = Path::new("/home/u/.local/bin/dimos");
        assert!(is_self(me, me));
        assert!(!is_self(Path::new("/home/u/dimos/.venv/bin/dimos"), me));
    }

    /// Reads a repo file on purpose: this list is the contract between the Rust and Python CLIs.
    #[test]
    fn reserved_list_matches_python_forwarder() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../dimos/cli/installer_cli.py");
        let source =
            fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        let want: Vec<String> = RESERVED.iter().map(|verb| verb.to_string()).collect();
        assert_eq!(forwarded_tuple(&source), want);
    }
}
