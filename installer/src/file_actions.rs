//! The filesystem effects an `Action` names: write, block, copy, rename, remove, sha256.

use std::fs;
use std::os::unix::fs::PermissionsExt;
use std::path::Path;

use anyhow::{bail, Context, Result};
use sha2::{Digest, Sha256};

use crate::plan::{ensure_block, Action};
use crate::run_context::Ctx;
use crate::spawn::run_argv;

pub(crate) fn exec_action(action: &Action, ctx: &Ctx) -> Result<Option<i32>> {
    match action {
        Action::Run {
            argv,
            sudo,
            cwd,
            env,
            timeout_s,
        } => run_argv(argv, *sudo, cwd.as_deref(), env, *timeout_s, ctx).map(Some),
        Action::WriteFile {
            path,
            mode,
            contents,
            sudo,
        } => write_file(path, *mode, contents, *sudo, ctx).map(|_| None),
        Action::EnsureBlock {
            file,
            marker,
            lines,
        } => ensure_block_file(file, marker, lines).map(|_| None),
        Action::Copy { from, to, mode } => copy_file(from, to, *mode).map(|_| None),
        Action::Rename { from, to } => fs::rename(from, to)
            .with_context(|| format!("rename {} -> {}", from.display(), to.display()))
            .map(|_| None),
        Action::Remove { path, sudo } => remove_file(path, *sudo, ctx).map(|_| None),
        Action::VerifySha256 { file, sums_file } => verify_sha256(file, sums_file).map(|_| None),
    }
}

fn write_file(path: &Path, mode: u32, contents: &str, sudo: bool, ctx: &Ctx) -> Result<()> {
    if fs::read_to_string(path).is_ok_and(|t| t == contents) {
        return Ok(());
    }
    if !sudo {
        return write_atomic(path, mode, contents);
    }
    let name = path.file_name().unwrap_or_default().to_string_lossy();
    let tmp = std::env::temp_dir().join(format!("dimos-{}-{name}", ctx.run_id));
    write_atomic(&tmp, mode, contents)?;
    let argv = [
        "install".to_string(),
        "-m".to_string(),
        format!("{mode:o}"),
        tmp.display().to_string(),
        path.display().to_string(),
    ];
    let status = run_argv(&argv, true, None, &[], 60, ctx);
    let _ = fs::remove_file(&tmp);
    status.map(|_| ())
}

/// The rename lands on the real file, so a symlinked rc file (stow, chezmoi) keeps its link.
fn write_atomic(path: &Path, mode: u32, contents: &str) -> Result<()> {
    let path = fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf());
    ensure_parent(&path)?;
    let tmp = path.with_extension("dimos-tmp");
    fs::write(&tmp, contents).with_context(|| format!("write {}", tmp.display()))?;
    fs::set_permissions(&tmp, fs::Permissions::from_mode(mode))
        .with_context(|| format!("chmod {mode:o} {}", tmp.display()))?;
    fs::rename(&tmp, &path)
        .with_context(|| format!("rename {} -> {}", tmp.display(), path.display()))
}

fn ensure_block_file(file: &Path, marker: &str, lines: &[String]) -> Result<()> {
    let before = fs::read_to_string(file).unwrap_or_default();
    let (after, changed) = ensure_block(&before, marker, lines);
    if !changed {
        return Ok(());
    }
    let mode = fs::metadata(file).map_or(0o644, |m| m.permissions().mode() & 0o777);
    write_atomic(file, mode, &after)
}

fn copy_file(from: &Path, to: &Path, mode: u32) -> Result<()> {
    ensure_parent(to)?;
    fs::copy(from, to).with_context(|| format!("copy {} -> {}", from.display(), to.display()))?;
    fs::set_permissions(to, fs::Permissions::from_mode(mode))
        .with_context(|| format!("chmod {mode:o} {}", to.display()))
}

fn ensure_parent(path: &Path) -> Result<()> {
    match path.parent() {
        Some(dir) => fs::create_dir_all(dir).with_context(|| format!("create {}", dir.display())),
        None => Ok(()),
    }
}

fn remove_file(path: &Path, sudo: bool, ctx: &Ctx) -> Result<()> {
    if sudo {
        let argv = [
            "rm".to_string(),
            "-f".to_string(),
            path.display().to_string(),
        ];
        return run_argv(&argv, true, None, &[], 30, ctx).map(|_| ());
    }
    match fs::remove_file(path) {
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
        other => other.with_context(|| format!("remove {}", path.display())),
    }
}

fn verify_sha256(file: &Path, sums_file: &Path) -> Result<()> {
    let text =
        fs::read_to_string(sums_file).with_context(|| format!("read {}", sums_file.display()))?;
    let want = parse_sha_file(&text)?;
    let got = sha256_hex(file)?;
    if got != want {
        bail!("sha256 of {} is {got}, want {want}", file.display());
    }
    Ok(())
}

pub fn sha256_hex(path: &Path) -> Result<String> {
    let mut file = fs::File::open(path).with_context(|| format!("read {}", path.display()))?;
    let mut hasher = Sha256::new();
    std::io::copy(&mut file, &mut hasher).with_context(|| format!("hash {}", path.display()))?;
    Ok(hasher
        .finalize()
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect())
}

/// The first 64-hex token, so `sha256sum` and `shasum -a 256` files both parse.
pub fn parse_sha_file(text: &str) -> Result<String> {
    text.split_whitespace()
        .find(|t| t.len() == 64 && t.chars().all(|c| c.is_ascii_hexdigit()))
        .map(|t| t.to_ascii_lowercase())
        .context("no 64-hex sha256 token in the .sha256 file")
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
            dry_run: false,
            verbose: false,
            yes: true,
            sudo: Sudo::Root,
            log: ActionLog::open(home).unwrap(),
            run_id: "test".to_string(),
        }
    }

    #[test]
    fn a_symlinked_rc_file_keeps_its_link_when_a_block_is_written() {
        let home = TmpDir::new("plan-symlink");
        let real = home.path().join("dotfiles/zshrc");
        fs::create_dir_all(real.parent().unwrap()).unwrap();
        fs::write(&real, "export A=1\n").unwrap();
        let link = home.path().join(".zshrc");
        std::os::unix::fs::symlink(&real, &link).unwrap();
        ensure_block_file(&link, "path", &["export B=2".to_string()]).unwrap();
        assert!(fs::symlink_metadata(&link)
            .unwrap()
            .file_type()
            .is_symlink());
        assert!(fs::read_to_string(&real).unwrap().contains("export B=2"));
    }

    #[test]
    fn write_file_equal_contents_is_noop() {
        let home = TmpDir::new("plan-write");
        let path = home.path().join("f.conf");
        let c = ctx(home.path());
        write_file(&path, 0o644, "same\n", false, &c).unwrap();
        let first = fs::metadata(&path).unwrap().modified().unwrap();
        write_file(&path, 0o644, "same\n", false, &c).unwrap();
        assert_eq!(fs::metadata(&path).unwrap().modified().unwrap(), first);
        assert_eq!(fs::read_to_string(&path).unwrap(), "same\n");
    }

    #[test]
    fn parse_sha_file_accepts_sha256sum_and_shasum_formats() {
        let hex = "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";
        assert_eq!(parse_sha_file(&format!("{hex}  dimos\n")).unwrap(), hex);
        assert_eq!(
            parse_sha_file(&format!("SHA256 (dimos) = {}\n", hex.to_uppercase())).unwrap(),
            hex
        );
        assert!(parse_sha_file("not a sum\n").is_err());
    }

    #[test]
    fn verify_sha256_error_is_got_vs_want() {
        let home = TmpDir::new("plan-sha");
        let file = home.path().join("dimos");
        let sums = home.path().join("dimos.sha256");
        fs::write(&file, b"payload").unwrap();
        let wrong = "0".repeat(64);
        fs::write(&sums, format!("{wrong}  dimos\n")).unwrap();
        let err = format!("{:#}", verify_sha256(&file, &sums).unwrap_err());
        assert!(err.contains("is ") && err.contains("want 0000"), "{err}");
    }

    fn sources() -> Vec<(String, String)> {
        fn walk(dir: &Path, out: &mut Vec<(String, String)>) {
            for entry in fs::read_dir(dir).expect("read src/").flatten() {
                let path = entry.path();
                if path.is_dir() {
                    walk(&path, out);
                } else if path.extension().is_some_and(|e| e == "rs") {
                    let name = path.file_name().unwrap().to_string_lossy().into_owned();
                    out.push((name, fs::read_to_string(&path).expect("read source")));
                }
            }
        }
        let mut out = Vec::new();
        walk(&Path::new(env!("CARGO_MANIFEST_DIR")).join("src"), &mut out);
        out
    }

    /// The text before `#[cfg(test)]`: a fixture may write into a TmpDir, the binary may not.
    fn runtime(text: &str) -> &str {
        text.split("#[cfg(test)]").next().unwrap_or(text)
    }

    #[test]
    fn the_filesystem_is_mutated_only_from_file_actions_rs_install_record_rs_and_action_log_rs() {
        const WRITERS: [&str; 6] = [
            "fs::write",
            "fs::create_dir",
            "fs::remove_",
            "fs::rename",
            "fs::copy",
            "fs::set_permissions",
        ];
        const HOMES: [&str; 3] = ["file_actions.rs", "install_record.rs", "action_log.rs"];
        let offenders: Vec<String> = sources()
            .iter()
            .filter(|(name, _)| !HOMES.contains(&name.as_str()))
            .filter(|(_, text)| WRITERS.iter().any(|w| runtime(text).contains(w)))
            .map(|(name, _)| name.clone())
            .collect();
        assert!(
            offenders.is_empty(),
            "writes outside {HOMES:?}: {offenders:?}"
        );
    }
}
