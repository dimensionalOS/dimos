//! Replacing the installer binary itself: fetch beside it, verify, swap, and put the backup back.

use std::path::{Path, PathBuf};

use anyhow::{bail, Result};

use crate::action::{text, Action};
use crate::cli::UpdateArgs;
use crate::install_record;
use crate::plan::{Plan, Stage};
use crate::platforms::DIMOS_VERSION;
use crate::probe::Probes;
use crate::run;
use crate::run_context::Ctx;
use crate::update::{override_url, Observed};
use crate::version::{artifact, newer, release_base};

const DOWNLOAD_TIMEOUT_S: u64 = 600;
const SHA_TIMEOUT_S: u64 = 120;
const SWAP_TIMEOUT_S: u64 = 30;

/// The swap renames a sibling file over the binary, so it only works on the installed copy.
fn require_installed_exe(home: &Path, current_exe: &Path) -> Result<PathBuf> {
    let installed = install_record::installed_bin(home);
    if current_exe != installed {
        bail!(
            "running {}, not the installed copy: `{} update` updates the binary",
            current_exe.display(),
            installed.display()
        );
    }
    Ok(installed)
}

/// Downloads beside the binary (one filesystem, so the swap is an atomic rename) and swaps only
/// after the sha256 and a `--version` run on the new file both pass.
fn self_update_stage(base: &str, target: &str, bin: &Path, bak: &Path) -> Stage {
    let (new, sums) = (bin.with_extension("new"), bin.with_extension("new.sha256"));
    fetch_actions(base, &artifact(target), &new, &sums)
        .into_iter()
        .chain(swap_actions(bin, bak, new, sums))
        .fold(Stage::new("self-update", false), Stage::push)
}

fn fetch_actions(base: &str, asset: &str, new: &Path, sums: &Path) -> Vec<Action> {
    let (new_s, sums_s) = (text(new), text(sums));
    let (bin_url, sums_url) = (format!("{base}/{asset}"), format!("{base}/{asset}.sha256"));
    vec![
        Action::run(
            &["curl", "-fsSL", "--retry", "3", &bin_url, "-o", &new_s],
            DOWNLOAD_TIMEOUT_S,
        ),
        Action::run(
            &["curl", "-fsSL", "--retry", "3", &sums_url, "-o", &sums_s],
            SHA_TIMEOUT_S,
        ),
        Action::VerifySha256 {
            file: new.to_path_buf(),
            sums_file: sums.to_path_buf(),
        },
        Action::run(&["chmod", "0755", &new_s], SWAP_TIMEOUT_S),
        Action::run(&[&new_s, "--version"], SWAP_TIMEOUT_S),
    ]
}

/// Copy, not rename: the binary is never absent, so a failed swap still leaves a working `dimos`.
fn swap_actions(bin: &Path, bak: &Path, new: PathBuf, sums: PathBuf) -> Vec<Action> {
    vec![
        Action::Copy {
            from: bin.to_path_buf(),
            to: bak.to_path_buf(),
            mode: 0o755,
        },
        Action::Rename {
            from: new,
            to: bin.to_path_buf(),
        },
        Action::Remove {
            path: sums,
            sudo: false,
        },
    ]
}

/// Skipped with a note rather than fatal: the rest of `update` is a repair and should still run.
pub(crate) fn self_update_or_note(
    args: &UpdateArgs,
    obs: &Observed,
    probes: &Probes,
    home: &Path,
) -> (Stage, Option<String>) {
    let empty = Stage::new("self-update", false);
    let bin = match require_installed_exe(home, &probes.current_exe) {
        Ok(bin) => bin,
        Err(e) => return (empty, Some(format!("{e:#}"))),
    };
    let target = match probes.platform.target() {
        Ok(target) => target,
        Err(e) => return (empty, Some(format!("{e:#}"))),
    };
    let Some(candidate) = obs.latest.as_deref() else {
        return (empty, None);
    };
    if !args.force && !newer(DIMOS_VERSION, candidate) {
        return (empty, None);
    }
    let base = release_base(Some(candidate), override_url().as_deref());
    (
        self_update_stage(&base, target, &bin, &install_record::backup_bin(home)),
        None,
    )
}

/// Puts the kept `.bak` back through the same executor, so `--dry-run --rollback` prints the swap.
pub(crate) fn rollback(ctx: &mut Ctx, home: &Path) -> Result<i32> {
    let (bin, bak) = (
        install_record::installed_bin(home),
        install_record::backup_bin(home),
    );
    if !bak.exists() {
        bail!("nothing to roll back: {} does not exist", bak.display());
    }
    let steps = Plan {
        command: "update --rollback".to_string(),
        stages: vec![Stage::new("rollback", true).push(Action::Rename { from: bak, to: bin })],
        notes: Vec::new(),
    };
    let report = run::run(&steps, ctx)?;
    report.print(ctx);
    Ok(report.exit_code())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::action_log::ActionLog;
    use crate::install_record::TmpDir;
    use crate::run_context::Mode;
    use crate::sudo::Sudo;

    fn ctx(home: &Path) -> Ctx {
        Ctx {
            mode: Mode::NonInteractive,
            dry_run: true,
            verbose: false,
            yes: true,
            sudo: Sudo::Root,
            log: ActionLog::open(home).expect("open the action log"),
            run_id: "test".to_string(),
        }
    }

    fn argv_of(stage: &Stage) -> Vec<Vec<String>> {
        stage
            .actions
            .iter()
            .filter_map(|a| match a {
                Action::Run { argv, .. } => Some(argv.clone()),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn self_update_swaps_only_after_the_sha256_and_the_version_run_pass() {
        let bin = PathBuf::from("/home/u/.local/bin/dimos");
        let bak = PathBuf::from("/home/u/.local/bin/dimos.bak");
        let stage = self_update_stage("http://host/d", "x86_64-linux-musl", &bin, &bak);
        let verify_at = stage
            .actions
            .iter()
            .position(|a| matches!(a, Action::VerifySha256 { .. }))
            .expect("a sha256 check");
        let rename_at = stage
            .actions
            .iter()
            .position(|a| matches!(a, Action::Rename { .. }))
            .expect("a swap");
        assert!(verify_at < rename_at);
        assert!(argv_of(&stage)
            .iter()
            .any(|argv| argv == &["/home/u/.local/bin/dimos.new", "--version"]));
    }

    #[test]
    fn self_update_copies_the_old_binary_before_the_rename_so_it_is_never_absent() {
        let bin = PathBuf::from("/home/u/.local/bin/dimos");
        let bak = bin.with_extension("bak");
        let stage = self_update_stage("http://host/d", "x86_64-linux-musl", &bin, &bak);
        let copy_at = stage
            .actions
            .iter()
            .position(|a| matches!(a, Action::Copy { to, .. } if *to == bak))
            .expect("a backup copy");
        let rename_at = stage
            .actions
            .iter()
            .position(|a| matches!(a, Action::Rename { to, .. } if *to == bin))
            .expect("a swap");
        assert!(copy_at < rename_at);
    }

    #[test]
    fn rollback_without_a_backup_names_the_missing_file() {
        let home = TmpDir::new("update-rollback");
        let err = rollback(&mut ctx(home.path()), home.path()).unwrap_err();
        assert!(format!("{err:#}").contains("dimos.bak"));
    }
}
