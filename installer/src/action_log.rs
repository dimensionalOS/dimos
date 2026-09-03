//! The redacted append-only action log: one JSON object per action in `installer.jsonl`.

use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use serde::Serialize;

use crate::install_record::state_dir;

pub(crate) fn action_log_path(home: &Path) -> PathBuf {
    state_dir(home).join("installer.jsonl")
}

/// Redacted log view: a Run keeps env KEYS only and a WriteFile keeps a byte count, never contents.
#[derive(Debug, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub(crate) enum ActionView<'a> {
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
pub(crate) struct ActionRecord<'a> {
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
    pub(crate) fn open(home: &Path) -> Result<ActionLog> {
        let dir = state_dir(home);
        fs::create_dir_all(&dir).with_context(|| format!("create {}", dir.display()))?;
        Ok(ActionLog {
            path: action_log_path(home),
        })
    }

    pub(crate) fn path(&self) -> &Path {
        &self.path
    }

    pub(crate) fn append(&self, rec: &ActionRecord) -> Result<()> {
        let line = serde_json::to_string(rec)?;
        let mut f = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.path)
            .with_context(|| format!("append to {}", self.path.display()))?;
        writeln!(f, "{line}").with_context(|| format!("append to {}", self.path.display()))
    }
}

/// `20260901T180140Z-7f3a` — sorts by time and separates concurrent runs.
pub(crate) fn run_id() -> String {
    let stamp = chrono::Utc::now().format("%Y%m%dT%H%M%SZ");
    format!("{stamp}-{:04x}", std::process::id() & 0xffff)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::install_record::{now_iso, TmpDir};

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
}
