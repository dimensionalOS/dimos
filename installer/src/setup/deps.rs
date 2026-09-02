//! The prerequisites DimOS is installed on top of: distro packages, uv, and opt-in nix.
//! Every stage is empty when the machine already has what it needs, so a re-run reports `already`.

use std::path::{Path, PathBuf};

use crate::pkgs::{self, Platforms};
use crate::plan::{Action, Stage};
use crate::probe::{PkgManager, Probes, Tools};

const APT_UPDATE_TIMEOUT_S: u64 = 600;
const PKG_INSTALL_TIMEOUT_S: u64 = 1800;
const DOWNLOAD_TIMEOUT_S: u64 = 120;
/// astral's script downloads the uv tarball itself, so it gets more than the script fetch.
const UV_INSTALL_TIMEOUT_S: u64 = 600;
const NIX_INSTALL_TIMEOUT_S: u64 = 1800;
const UV_INSTALLER_URL: &str = "https://astral.sh/uv/install.sh";
const NIX_INSTALLER_URL: &str = "https://nixos.org/nix/install";

fn spawn(argv: &[String], sudo: bool, timeout_s: u64) -> Action {
    let borrowed: Vec<&str> = argv.iter().map(String::as_str).collect();
    if sudo {
        Action::sudo(&borrowed, timeout_s)
    } else {
        Action::run(&borrowed, timeout_s)
    }
}

fn owned(argv: &[&str]) -> Vec<String> {
    argv.iter().map(|a| (*a).to_string()).collect()
}

fn missing_packages(wanted: &[String], pm: PkgManager, tools: &Tools) -> Vec<String> {
    match pm {
        PkgManager::Apt => pkgs::missing_apt(wanted, &tools.dpkg_status),
        PkgManager::Brew => pkgs::missing_brew(wanted, &tools.brew_list),
        PkgManager::None => Vec::new(),
    }
}

/// The frontend vars ride in argv, not `Action.env`: sudo's env_reset drops the env, and a debconf
/// or needrestart prompt with no TTY hangs until the timeout.
fn apt_argv(tail: &[&str], missing: &[String]) -> Vec<String> {
    let mut argv = owned(&[
        "env",
        "DEBIAN_FRONTEND=noninteractive",
        "NEEDRESTART_MODE=a",
        "apt-get",
    ]);
    argv.extend(owned(tail));
    argv.extend_from_slice(missing);
    argv
}

/// (argv, sudo, timeout_s) per manager; apt refreshes its lists first, brew refuses to run as root.
fn package_commands(pm: PkgManager, missing: &[String]) -> Vec<(Vec<String>, bool, u64)> {
    match pm {
        PkgManager::Apt => vec![
            (
                apt_argv(&["update", "-qq"], &[]),
                true,
                APT_UPDATE_TIMEOUT_S,
            ),
            (
                apt_argv(&["install", "-y", "-qq"], missing),
                true,
                PKG_INSTALL_TIMEOUT_S,
            ),
        ],
        PkgManager::Brew => {
            let mut argv = owned(&["brew", "install"]);
            argv.extend_from_slice(missing);
            vec![(argv, false, PKG_INSTALL_TIMEOUT_S)]
        }
        PkgManager::None => Vec::new(),
    }
}

/// Installs only the packages this machine is missing; consent-gated because it changes the system.
pub fn packages_stage(extras: &[String], probes: &Probes, cfg: &Platforms) -> Stage {
    let pm = probes.platform.pkg;
    let wanted = pkgs::system_packages(extras, pm, cfg);
    let missing = missing_packages(&wanted, pm, &probes.tools);
    let stage = Stage::new("packages", true);
    if missing.is_empty() {
        return stage;
    }
    package_commands(pm, &missing)
        .iter()
        .fold(stage.consent(), |stage, (argv, sudo, timeout_s)| {
            stage.push(spawn(argv, *sudo, *timeout_s))
        })
}

/// Where uv is, or where `uv_stage` will put it; a planned absolute path, never a PATH lookup at exec.
pub fn uv_bin(tools: &Tools, home: &Path) -> PathBuf {
    tools
        .uv
        .clone()
        .unwrap_or_else(|| home.join(".local/bin/uv"))
}

/// Downloads astral's installer and runs it into ~/.local/bin; empty when uv is already there.
pub fn uv_stage(tools: &Tools, home: &Path) -> Stage {
    let stage = Stage::new("uv", true);
    if tools.uv.is_some() {
        return stage;
    }
    let script = std::env::temp_dir().join("uv-install.sh");
    let script_path = script.display().to_string();
    let uv = uv_bin(tools, home);
    let local_bin = home.join(".local/bin").display().to_string();
    // UV_NO_MODIFY_PATH because the rc block is ours; astral's would be a second, unguarded edit.
    let env = [("UV_NO_MODIFY_PATH", "1"), ("UV_INSTALL_DIR", &local_bin)];
    stage
        .run(
            &["curl", "-fsSL", UV_INSTALLER_URL, "-o", &script_path],
            DOWNLOAD_TIMEOUT_S,
        )
        .push(Action::run_in(
            &["sh", &script_path],
            None,
            &env,
            UV_INSTALL_TIMEOUT_S,
        ))
        .post(&[&uv.display().to_string(), "--version"])
}

/// Opt-in only (`--with-nix`); non-critical because DimOS installs from system packages without it.
pub fn nix_stage(tools: &Tools, home: &Path, with_nix: bool) -> Stage {
    let stage = Stage::new("nix", false);
    if !with_nix || tools.nix {
        return stage;
    }
    let script = std::env::temp_dir().join("nix-install.sh");
    let script_path = script.display().to_string();
    stage
        .run(
            &[
                "curl",
                "--proto",
                "=https",
                "--tlsv1.2",
                "-fsSL",
                NIX_INSTALLER_URL,
                "-o",
                &script_path,
            ],
            DOWNLOAD_TIMEOUT_S,
        )
        .sudo(
            &["sh", &script_path, "--daemon", "--yes"],
            NIX_INSTALL_TIMEOUT_S,
        )
        .push(Action::EnsureBlock {
            file: home.join(".config/nix/nix.conf"),
            marker: "nix".to_string(),
            lines: vec!["experimental-features = nix-command flakes".to_string()],
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan::sudo_env_violations;
    use crate::probe::{Arch, Gpu, Os, Platform, Probes};
    use std::path::PathBuf;

    fn platform(pkg: PkgManager) -> Platform {
        Platform {
            os: Os::Linux {
                id: "ubuntu".to_string(),
                version: "22.04".to_string(),
            },
            arch: Arch::X86_64,
            glibc: Some((2, 35)),
            jetson: None,
            gpu: Gpu::None,
            pkg,
            systemd: true,
            home: PathBuf::from("/home/u"),
            user: "u".to_string(),
            shell: PathBuf::from("/bin/bash"),
        }
    }

    fn probes(pkg: PkgManager, dpkg_status: &str, brew_list: &str) -> Probes {
        Probes {
            platform: platform(pkg),
            kernel: Default::default(),
            tools: Tools {
                dpkg_status: dpkg_status.to_string(),
                brew_list: brew_list.to_string(),
                ..Default::default()
            },
            installed: None,
            rc: Vec::new(),
            ifaces: Vec::new(),
            dotenv: None,
            current_exe: PathBuf::from("/usr/local/bin/dimos"),
        }
    }

    fn argv_of(stage: &Stage, index: usize) -> Vec<String> {
        match &stage.actions[index] {
            Action::Run { argv, .. } => argv.clone(),
            other => panic!("expected a Run, got {other:?}"),
        }
    }

    fn every_package_installed() -> String {
        pkgs::system_packages(&["base".to_string()], PkgManager::Apt, &Platforms::load())
            .iter()
            .map(|p| format!("{p} install ok installed\n"))
            .collect()
    }

    #[test]
    fn packages_stage_lists_only_missing_packages_and_is_empty_when_none() {
        let cfg = Platforms::load();
        let extras = ["base".to_string()];
        let complete = probes(PkgManager::Apt, &every_package_installed(), "");
        assert!(packages_stage(&extras, &complete, &cfg).actions.is_empty());

        let partial = probes(
            PkgManager::Apt,
            &every_package_installed().replace("git-lfs install ok installed\n", ""),
            "",
        );
        let stage = packages_stage(&extras, &partial, &cfg);
        let install = argv_of(&stage, 1);
        assert!(install.contains(&"git-lfs".to_string()), "{install:?}");
        assert!(!install.contains(&"curl".to_string()), "{install:?}");
    }

    #[test]
    fn apt_sets_debian_frontend_in_argv_because_sudo_resets_env() {
        let cfg = Platforms::load();
        let stage = packages_stage(
            &["base".to_string()],
            &probes(PkgManager::Apt, "", ""),
            &cfg,
        );
        assert!(stage.critical && stage.consent && stage.needs_sudo());
        assert_eq!(
            argv_of(&stage, 0),
            [
                "env",
                "DEBIAN_FRONTEND=noninteractive",
                "NEEDRESTART_MODE=a",
                "apt-get",
                "update",
                "-qq"
            ]
        );
        assert_eq!(
            &argv_of(&stage, 1)[..7],
            [
                "env",
                "DEBIAN_FRONTEND=noninteractive",
                "NEEDRESTART_MODE=a",
                "apt-get",
                "install",
                "-y",
                "-qq"
            ]
        );
        let plan = crate::plan::Plan {
            command: "setup".to_string(),
            stages: vec![stage],
            notes: Vec::new(),
        };
        assert!(sudo_env_violations(&plan).is_empty());
    }

    #[test]
    fn brew_stage_has_no_sudo_but_needs_consent() {
        let cfg = Platforms::load();
        let stage = packages_stage(
            &["base".to_string()],
            &probes(PkgManager::Brew, "", "git-lfs 3.5.1\n"),
            &cfg,
        );
        assert!(stage.consent && !stage.needs_sudo());
        let argv = argv_of(&stage, 0);
        assert_eq!(&argv[..2], ["brew", "install"]);
        assert!(!argv.contains(&"git-lfs".to_string()), "{argv:?}");
        assert!(argv.contains(&"portaudio".to_string()), "{argv:?}");
    }

    #[test]
    fn uv_stage_empty_when_uv_present_and_uses_local_bin_path_otherwise() {
        let home = Path::new("/home/u");
        let present = Tools {
            uv: Some(PathBuf::from("/opt/uv/uv")),
            ..Default::default()
        };
        assert!(uv_stage(&present, home).actions.is_empty());
        assert_eq!(uv_bin(&present, home), PathBuf::from("/opt/uv/uv"));

        let absent = Tools::default();
        assert_eq!(
            uv_bin(&absent, home),
            PathBuf::from("/home/u/.local/bin/uv")
        );
        let stage = uv_stage(&absent, home);
        assert_eq!(
            stage.post,
            Some(vec![
                "/home/u/.local/bin/uv".to_string(),
                "--version".to_string()
            ])
        );
        match &stage.actions[1] {
            Action::Run { env, sudo, .. } => {
                assert!(!sudo);
                assert!(env.contains(&("UV_NO_MODIFY_PATH".to_string(), "1".to_string())));
                assert!(env.contains(&(
                    "UV_INSTALL_DIR".to_string(),
                    "/home/u/.local/bin".to_string()
                )));
            }
            other => panic!("expected the installer Run, got {other:?}"),
        }
    }

    #[test]
    fn nix_stage_empty_unless_with_nix() {
        let home = Path::new("/home/u");
        let absent = Tools::default();
        assert!(nix_stage(&absent, home, false).actions.is_empty());
        let installed = Tools {
            nix: true,
            ..Default::default()
        };
        assert!(nix_stage(&installed, home, true).actions.is_empty());

        let stage = nix_stage(&absent, home, true);
        assert!(!stage.critical && stage.needs_sudo());
        assert_eq!(
            stage.actions.last(),
            Some(&Action::EnsureBlock {
                file: PathBuf::from("/home/u/.config/nix/nix.conf"),
                marker: "nix".to_string(),
                lines: vec!["experimental-features = nix-command flakes".to_string()],
            })
        );
    }
}
