//! Invariants over the whole crate source: one place prompts, three places mutate the filesystem.

use std::fs;
use std::path::Path;

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
fn prompts_live_only_in_run_context_rs() {
    let elsewhere: Vec<String> = sources()
        .iter()
        .filter(|(name, text)| name != "run_context.rs" && text.contains("stdin()"))
        .map(|(name, _)| name.clone())
        .collect();
    assert!(
        elsewhere.is_empty(),
        "stdin read outside run_context.rs: {elsewhere:?}"
    );
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
