//! `dimos uninstall`: remove only what the installer wrote. `observe` is the only I/O; every stage
//! is pure over its result, so a machine with nothing installed plans nothing.

use std::path::{Path, PathBuf};

use anyhow::Result;

use crate::action::{self, Action};
use crate::action_log::action_log_path;
use crate::install_record;
use crate::plan::{Plan, Stage};
use crate::run;
use crate::run_context::Ctx;
use crate::setup::self_install::PATH_MARKER;
use crate::systemd_service;
use crate::wizards::unitree::g1::CDDS_MARKER;

/// systemctl and rm return immediately; the budget only bounds a hung sudo.
const REMOVE_TIMEOUT_S: u64 = 30;

const MARKERS: [&str; 2] = [PATH_MARKER, CDDS_MARKER];

const STAYS: &str = "kept: the DimOS project dir, ~/cyclonedds, the Unitree SDK clone, \
                     uv, and the apt/brew packages";

#[derive(Debug, Clone, PartialEq, Eq, Default)]
struct Existing {
    pub bin: bool,
    pub bak: bool,
    pub json: bool,
    pub log: bool,
    pub rc_files_with_blocks: Vec<PathBuf>,
    pub units: Vec<PathBuf>,
    pub sysctl_conf: bool,
    pub memlock_conf: bool,
}

/// The only I/O in this file; read-only, so `--dry-run` may call it.
fn observe(home: &Path) -> Existing {
    Existing {
        bin: install_record::installed_bin(home).exists(),
        bak: install_record::backup_bin(home).exists(),
        json: install_record::installer_json(home).exists(),
        log: action_log_path(home).exists(),
        rc_files_with_blocks: rc_files_with_blocks(home),
        units: systemd_service::installed_units(),
        sysctl_conf: Path::new(install_record::SYSCTL_CONF).exists(),
        memlock_conf: Path::new(install_record::MEMLOCK_CONF).exists(),
    }
}

/// Every candidate, not the current shell's: the block landed under whatever shell `setup` saw.
fn rc_files_with_blocks(home: &Path) -> Vec<PathBuf> {
    install_record::RC_CANDIDATES
        .iter()
        .map(|name| home.join(name))
        .filter(|file| std::fs::read_to_string(file).is_ok_and(|text| has_block(&text)))
        .collect()
}

/// A removal that changes the text is the same question as "is a block there".
fn has_block(text: &str) -> bool {
    MARKERS
        .iter()
        .any(|marker| action::ensure_block(text, marker, &[]).1)
}

fn plan(existing: &Existing, home: &Path) -> Plan {
    Plan {
        command: "uninstall".to_string(),
        stages: vec![
            shell_stage(existing),
            system_stage(existing),
            installer_stage(existing, home),
        ],
        notes: vec![STAYS.to_string()],
    }
}

/// Removing an absent marker is a no-op, so both are planned for every rc file carrying either.
fn shell_stage(existing: &Existing) -> Stage {
    existing
        .rc_files_with_blocks
        .iter()
        .flat_map(|file| block_removals(file.as_path()))
        .fold(Stage::new("shell", false), Stage::push)
        .consent()
}

fn block_removals(file: &Path) -> Vec<Action> {
    MARKERS
        .iter()
        .map(|marker| Action::EnsureBlock {
            file: file.to_path_buf(),
            marker: (*marker).to_string(),
            lines: Vec::new(),
        })
        .collect()
}

fn system_stage(existing: &Existing) -> Stage {
    existing
        .units
        .iter()
        .flat_map(|unit| unit_removal(unit.as_path()))
        .chain(conf_removals(existing))
        .chain(daemon_reload(existing))
        .fold(Stage::new("system", false), Stage::push)
        .consent()
}

/// Disable first: a unit file deleted while still enabled leaves a dangling wants/ symlink.
fn unit_removal(unit: &Path) -> Vec<Action> {
    let name = unit.file_name().unwrap_or_default().to_string_lossy();
    vec![
        Action::sudo(&["systemctl", "disable", "--now", &name], REMOVE_TIMEOUT_S),
        Action::Remove {
            path: unit.to_path_buf(),
            sudo: true,
        },
    ]
}

fn conf_removals(existing: &Existing) -> Vec<Action> {
    [
        (existing.sysctl_conf, install_record::SYSCTL_CONF),
        (existing.memlock_conf, install_record::MEMLOCK_CONF),
    ]
    .into_iter()
    .filter(|(present, _)| *present)
    .map(|(_, path)| Action::Remove {
        path: PathBuf::from(path),
        sudo: true,
    })
    .collect()
}

fn daemon_reload(existing: &Existing) -> Option<Action> {
    (!existing.units.is_empty())
        .then(|| Action::sudo(&["systemctl", "daemon-reload"], REMOVE_TIMEOUT_S))
}

/// The binary, its backup, and the two state files.
fn installer_stage(existing: &Existing, home: &Path) -> Stage {
    [
        (existing.bin, install_record::installed_bin(home)),
        (existing.bak, install_record::backup_bin(home)),
        (existing.json, install_record::installer_json(home)),
        (existing.log, action_log_path(home)), // re-created by this run's own records
    ]
    .into_iter()
    .filter(|(present, _)| *present)
    .map(|(_, path)| Action::Remove { path, sudo: false })
    .fold(Stage::new("installer", false), Stage::push)
    .consent()
}

pub fn run(ctx: &mut Ctx, home: &Path) -> Result<i32> {
    let report = run::run(&plan(&observe(home), home), ctx)?;
    report.print(ctx);
    Ok(report.exit_code())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::install_record::TmpDir;

    const RC: &str = "export EDITOR=vim\n\
                      # DIMOS-ADDED: path start\n\
                      export PATH=\"$HOME/.local/bin:$PATH\"\n\
                      # DIMOS-ADDED: path end\n\
                      # DIMOS-ADDED: cyclonedds start\n\
                      export CYCLONEDDS_HOME=/home/unitree/cyclonedds/install\n\
                      # DIMOS-ADDED: cyclonedds end\n\
                      alias k=kubectl\n";

    const OTHER_UNIT: &str = "dimos-unitree-g1";

    fn full(home: &Path) -> Existing {
        Existing {
            bin: true,
            bak: true,
            json: true,
            log: true,
            rc_files_with_blocks: vec![home.join(".bashrc")],
            units: vec![
                install_record::unit_path(install_record::MULTICAST_UNIT),
                install_record::unit_path(OTHER_UNIT),
            ],
            sysctl_conf: true,
            memlock_conf: true,
        }
    }

    fn actions(plan: &Plan) -> Vec<&Action> {
        plan.stages.iter().flat_map(|s| &s.actions).collect()
    }

    #[test]
    fn plan_never_touches_project_dir_vendor_dirs_or_uv() {
        let tmp = TmpDir::new("uninstall-scope");
        let home = tmp.path();
        let written = [
            install_record::installed_bin(home),
            install_record::backup_bin(home),
            install_record::installer_json(home),
            action_log_path(home),
            home.join(".bashrc"),
            install_record::unit_path(install_record::MULTICAST_UNIT),
            install_record::unit_path(OTHER_UNIT),
            PathBuf::from(install_record::SYSCTL_CONF),
            PathBuf::from(install_record::MEMLOCK_CONF),
        ];
        let built = plan(&full(home), home);
        assert_eq!(actions(&built).len(), 13);
        for action in actions(&built) {
            match action {
                Action::Remove { path, .. } | Action::EnsureBlock { file: path, .. } => {
                    assert!(written.contains(path), "{}", path.display());
                }
                Action::Run { argv, .. } => assert_eq!(argv[0], "systemctl"),
                other => panic!("unexpected action {}", other.display()),
            }
        }
    }

    #[test]
    fn absent_files_produce_no_actions() {
        let tmp = TmpDir::new("uninstall-clean");
        let built = plan(&Existing::default(), tmp.path());
        assert!(actions(&built).is_empty());
    }

    #[test]
    fn rc_blocks_are_removed_by_marker_and_other_lines_survive() {
        let tmp = TmpDir::new("uninstall-rc");
        let home = tmp.path();
        let built = plan(&full(home), home);
        let mut text = RC.to_string();
        for action in actions(&built) {
            if let Action::EnsureBlock { marker, lines, .. } = action {
                text = action::ensure_block(&text, marker, lines).0;
            }
        }
        assert_eq!(text, "export EDITOR=vim\nalias k=kubectl\n");
    }

    #[test]
    fn each_unit_is_disabled_before_its_file_goes_then_systemd_reloads_once() {
        let tmp = TmpDir::new("uninstall-units");
        let home = tmp.path();
        let built = plan(&full(home), home);
        let system = built
            .stages
            .iter()
            .find(|s| s.name == "system")
            .expect("system stage");
        let unit = install_record::unit_path(install_record::MULTICAST_UNIT);
        let disable = ["systemctl", "disable", "--now", "dimos-multicast.service"];
        assert_eq!(system.actions[0], Action::sudo(&disable, REMOVE_TIMEOUT_S));
        assert_eq!(
            system.actions[1],
            Action::Remove {
                path: unit,
                sudo: true
            }
        );
        assert_eq!(
            system.actions.last(),
            Some(&Action::sudo(
                &["systemctl", "daemon-reload"],
                REMOVE_TIMEOUT_S
            ))
        );
    }

    #[test]
    fn every_stage_needs_consent_so_nothing_goes_without_yes_or_a_confirm() {
        let tmp = TmpDir::new("uninstall-consent");
        let home = tmp.path();
        assert!(plan(&full(home), home).stages.iter().all(|s| s.consent));
    }

    #[test]
    fn observe_finds_only_rc_files_that_carry_a_dimos_block() {
        let tmp = TmpDir::new("uninstall-observe");
        let home = tmp.path();
        std::fs::write(home.join(".bashrc"), RC).expect("write rc fixture");
        std::fs::write(home.join(".zshrc"), "alias k=kubectl\n").expect("write rc fixture");
        assert_eq!(
            observe(home).rc_files_with_blocks,
            vec![home.join(".bashrc")]
        );
    }
}
