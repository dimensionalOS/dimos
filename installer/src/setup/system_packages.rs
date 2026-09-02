//! The prerequisites DimOS is installed on top of: distro packages, uv, and opt-in nix.
//! Every stage is empty when the machine already has what it needs, so a re-run reports `already`.

use std::path::{Path, PathBuf};

use crate::install_record;
use crate::plan::{owned, text, Action, Stage};
use crate::platforms::{self, Platforms};
use crate::probe::{PkgManager, Probes, Tools};

const APT_UPDATE_TIMEOUT_S: u64 = 600;
const PKG_INSTALL_TIMEOUT_S: u64 = 1800;
const DOWNLOAD_TIMEOUT_S: u64 = 120;
/// astral's script downloads the uv tarball itself, so it gets more than the script fetch.
const UV_INSTALL_TIMEOUT_S: u64 = 600;
const NIX_INSTALL_TIMEOUT_S: u64 = 1800;
const UV_INSTALLER_URL: &str = "https://astral.sh/uv/install.sh";
const NIX_INSTALLER_URL: &str = "https://nixos.org/nix/install";

fn missing_packages(wanted: &[String], pm: PkgManager, tools: &Tools) -> Vec<String> {
    match pm {
        PkgManager::Apt => platforms::missing_apt(wanted, &tools.dpkg_status),
        PkgManager::Brew => platforms::missing_brew(wanted, &tools.brew_list),
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

/// The install per manager: apt as root, brew as the user because it refuses to run as root.
fn install_action(pm: PkgManager, missing: &[String]) -> Option<Action> {
    let (argv, sudo) = match pm {
        PkgManager::Apt => (apt_argv(&["install", "-y", "-qq"], missing), true),
        PkgManager::Brew => {
            let mut argv = owned(&["brew", "install"]);
            argv.extend_from_slice(missing);
            (argv, false)
        }
        PkgManager::None => return None,
    };
    Some(Action::run_owned(
        argv,
        sudo,
        None,
        &[],
        PKG_INSTALL_TIMEOUT_S,
    ))
}

/// The list refresh and the install, both empty when nothing is missing.
pub fn packages_stages(extras: &[String], probes: &Probes, cfg: &Platforms) -> Vec<Stage> {
    let pm = probes.platform.pkg;
    let wanted = platforms::system_packages(extras, pm, cfg);
    let missing = missing_packages(&wanted, pm, &probes.tools);
    vec![apt_update_stage(pm, &missing), packages_stage(pm, &missing)]
}

/// Best effort: a dead source (the expired ROS key on 20.04 Jetson images exits 100) is a `!!`,
/// and the install still tries the lists apt already has.
fn apt_update_stage(pm: PkgManager, missing: &[String]) -> Stage {
    let stage = Stage::new("apt update", false).warn_only();
    if pm != PkgManager::Apt || missing.is_empty() {
        return stage;
    }
    stage.push(Action::run_owned(
        apt_argv(&["update", "-qq"], &[]),
        true,
        None,
        &[],
        APT_UPDATE_TIMEOUT_S,
    ))
}

/// Installs only the packages this machine is missing; consent-gated because it changes the system.
fn packages_stage(pm: PkgManager, missing: &[String]) -> Stage {
    let stage = Stage::new("packages", true);
    if missing.is_empty() {
        return stage;
    }
    match install_action(pm, missing) {
        Some(install) => stage.consent().push(install),
        None => stage,
    }
}

/// Where uv is, or where `uv_stage` will put it; a planned absolute path, never a PATH lookup at exec.
pub fn uv_bin(tools: &Tools, home: &Path) -> PathBuf {
    tools
        .uv
        .clone()
        .unwrap_or_else(|| home.join(".local/bin/uv"))
}

/// A downloaded installer lives under the user's own state dir, never a shared /tmp name a
/// neighbour could plant, and is removed once it has run.
fn script_path(home: &Path, name: &str) -> PathBuf {
    install_record::state_dir(home).join(name)
}

/// curl to a path; nix's own instructions pin TLS 1.2 and https, astral's do not.
fn download(url: &str, to: &Path, strict_tls: bool) -> Action {
    let mut argv = owned(&["curl"]);
    if strict_tls {
        argv.extend(owned(&["--proto", "=https", "--tlsv1.2"]));
    }
    argv.extend(owned(&["-fsSL", url, "-o", &text(to)]));
    Action::run_owned(argv, false, None, &[], DOWNLOAD_TIMEOUT_S)
}

fn remove(path: PathBuf) -> Action {
    Action::Remove { path, sudo: false }
}

/// Downloads astral's installer and runs it into ~/.local/bin; empty when uv is already there.
pub fn uv_stage(tools: &Tools, home: &Path) -> Stage {
    let stage = Stage::new("uv", true);
    if tools.uv.is_some() {
        return stage;
    }
    let script = script_path(home, "uv-install.sh");
    let uv = uv_bin(tools, home);
    let local_bin = text(&home.join(".local/bin"));
    // UV_NO_MODIFY_PATH because the rc block is ours; astral's would be a second, unguarded edit.
    let env = [("UV_NO_MODIFY_PATH", "1"), ("UV_INSTALL_DIR", &local_bin)];
    stage
        .push(download(UV_INSTALLER_URL, &script, false))
        .push(Action::run_in(
            &["sh", &text(&script)],
            None,
            &env,
            UV_INSTALL_TIMEOUT_S,
        ))
        .push(remove(script))
        .post(&[&text(&uv), "--version"])
}

/// Opt-in only (`--with-nix`); non-critical because DimOS installs from system packages without it.
pub fn nix_stage(tools: &Tools, home: &Path, with_nix: bool) -> Stage {
    let stage = Stage::new("nix", false);
    if !with_nix || tools.nix {
        return stage;
    }
    let script = script_path(home, "nix-install.sh");
    stage
        .push(download(NIX_INSTALLER_URL, &script, true))
        .sudo(
            &["sh", &text(&script), "--daemon", "--yes"],
            NIX_INSTALL_TIMEOUT_S,
        )
        .push(Action::EnsureBlock {
            file: home.join(".config/nix/nix.conf"),
            marker: "nix".to_string(),
            lines: vec!["experimental-features = nix-command flakes".to_string()],
        })
        .push(remove(script))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::probe::{Arch, Gpu, Os, Platform, Probes};
    use crate::run::sudo_env_violations;
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
        platforms::system_packages(&["base".to_string()], PkgManager::Apt, &Platforms::load())
            .iter()
            .map(|p| format!("{p} install ok installed\n"))
            .collect()
    }

    /// `[apt update, packages]`, in that order.
    fn planned(pm: PkgManager, dpkg_status: &str, brew_list: &str) -> Vec<Stage> {
        packages_stages(
            &["base".to_string()],
            &probes(pm, dpkg_status, brew_list),
            &Platforms::load(),
        )
    }

    #[test]
    fn packages_stage_lists_only_missing_packages_and_is_empty_when_none() {
        let complete = planned(PkgManager::Apt, &every_package_installed(), "");
        assert!(complete.iter().all(|s| s.actions.is_empty()));

        let partial = planned(
            PkgManager::Apt,
            &every_package_installed().replace("git-lfs install ok installed\n", ""),
            "",
        );
        let install = argv_of(&partial[1], 0);
        assert!(install.contains(&"git-lfs".to_string()), "{install:?}");
        assert!(!install.contains(&"curl".to_string()), "{install:?}");
    }

    #[test]
    fn apt_sets_debian_frontend_in_argv_because_sudo_resets_env() {
        let stages = planned(PkgManager::Apt, "", "");
        assert!(stages[1].critical && stages[1].consent && stages[1].needs_sudo());
        assert_eq!(
            argv_of(&stages[0], 0),
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
            &argv_of(&stages[1], 0)[..7],
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
            stages,
            notes: Vec::new(),
        };
        assert!(sudo_env_violations(&plan).is_empty());
    }

    #[test]
    fn apt_update_is_best_effort_so_a_dead_source_does_not_block_the_install() {
        let apt = planned(PkgManager::Apt, "", "");
        assert_eq!(apt[0].name, "apt update");
        assert!(apt[0].warn_only && !apt[0].critical && apt[0].needs_sudo());
        assert!(!apt[0].consent);
        assert!(planned(PkgManager::Brew, "", "")[0].actions.is_empty());
    }

    #[test]
    fn brew_stage_has_no_sudo_but_needs_consent() {
        let stage = &planned(PkgManager::Brew, "", "git-lfs 3.5.1\n")[1];
        assert!(stage.consent && !stage.needs_sudo());
        let argv = argv_of(stage, 0);
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
    fn a_downloaded_installer_lives_under_the_users_state_dir_and_is_removed_after_it_runs() {
        let home = Path::new("/home/u");
        for stage in [
            uv_stage(&Tools::default(), home),
            nix_stage(&Tools::default(), home, true),
        ] {
            let Action::Run { argv, .. } = &stage.actions[0] else {
                panic!("the first action downloads the script")
            };
            let script = PathBuf::from(argv.last().expect("curl -o <script>"));
            assert!(
                script.starts_with(install_record::state_dir(home)),
                "{}",
                script.display()
            );
            assert_eq!(
                stage.actions.last(),
                Some(&Action::Remove {
                    path: script,
                    sudo: false
                })
            );
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
            stage.actions[2],
            Action::EnsureBlock {
                file: PathBuf::from("/home/u/.config/nix/nix.conf"),
                marker: "nix".to_string(),
                lines: vec!["experimental-features = nix-command flakes".to_string()],
            }
        );
    }
}
