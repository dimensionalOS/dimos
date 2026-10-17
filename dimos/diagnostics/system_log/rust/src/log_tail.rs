// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Follows every *.jsonl in a directory from its start, across the RotatingFileHandler's renames
//! (main.jsonl → main.jsonl.1 → main.jsonl.2 ... and a fresh main.jsonl). Files are followed by inode, so a
//! backup that appeared and was renamed again between two reads is still read in full.

use std::collections::{BTreeMap, HashSet};
use std::fs::File;
use std::io::Read;
use std::os::unix::fs::MetadataExt;
use std::path::PathBuf;

struct Followed {
    file: File,
    /// the live file's name (main.jsonl), whatever backup number it has been renamed to
    name: String,
    partial: Vec<u8>,
}

pub struct JsonlTail {
    directory: PathBuf,
    followed: BTreeMap<u64, Followed>,
    /// backups that were already there on the first read: older than this module, so not published
    skipped: HashSet<u64>,
    started: bool,
}

/// "main.jsonl" → ("main.jsonl", 0), "main.jsonl.3" → ("main.jsonl", 3), anything else → None.
fn log_name(file_name: &str) -> Option<(String, u32)> {
    if file_name.ends_with(".jsonl") {
        return Some((file_name.to_string(), 0));
    }
    let (live, backup) = file_name.rsplit_once('.')?;
    let backup = backup.parse().ok()?;
    live.ends_with(".jsonl").then(|| (live.to_string(), backup))
}

impl JsonlTail {
    pub fn new(directory: PathBuf) -> Self {
        Self {
            directory,
            followed: BTreeMap::new(),
            skipped: HashSet::new(),
            started: false,
        }
    }

    /// The complete lines written since the last call, as (file name, line).
    pub fn read(&mut self) -> Vec<(String, String)> {
        let mut logs: Vec<(String, u32, u64, PathBuf)> = std::fs::read_dir(&self.directory)
            .into_iter()
            .flatten()
            .flatten()
            .filter_map(|entry| {
                let (name, backup) = log_name(&entry.file_name().to_string_lossy())?;
                let inode = entry.metadata().ok()?.ino();
                Some((name, backup, inode, entry.path()))
            })
            .collect();
        // oldest first: the highest backup number, the live file last
        logs.sort_by(|a, b| a.0.cmp(&b.0).then(b.1.cmp(&a.1)));

        let mut lines = Vec::new();
        let present: HashSet<u64> = logs.iter().map(|log| log.2).collect();
        // deleted past the handler's backupCount: the open handle can still finish it
        let gone: Vec<u64> = self
            .followed
            .keys()
            .filter(|inode| !present.contains(inode))
            .copied()
            .collect();
        for inode in gone {
            if let Some(mut followed) = self.followed.remove(&inode) {
                drain(&mut followed, &mut lines);
            }
        }
        self.skipped.retain(|inode| present.contains(inode));

        for (name, backup, inode, path) in logs {
            if self.skipped.contains(&inode) {
                continue;
            }
            if !self.followed.contains_key(&inode) {
                if !self.started && backup > 0 {
                    self.skipped.insert(inode);
                    continue;
                }
                let Ok(file) = File::open(&path) else {
                    continue;
                };
                self.followed.insert(
                    inode,
                    Followed {
                        file,
                        name,
                        partial: Vec::new(),
                    },
                );
            }
            if let Some(followed) = self.followed.get_mut(&inode) {
                drain(followed, &mut lines);
            }
        }
        self.started = true;
        lines
    }
}

fn drain(followed: &mut Followed, lines: &mut Vec<(String, String)>) {
    if followed.file.read_to_end(&mut followed.partial).is_err() {
        return;
    }
    let Some(last_newline) = followed.partial.iter().rposition(|&b| b == b'\n') else {
        return;
    };
    let rest = followed.partial.split_off(last_newline + 1);
    for line in String::from_utf8_lossy(&followed.partial).lines() {
        if !line.trim().is_empty() {
            lines.push((followed.name.clone(), line.to_string()));
        }
    }
    followed.partial = rest;
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::io::Write;

    #[test]
    fn follows_appends_partial_lines_and_rotation() {
        let dir = std::env::temp_dir().join(format!("system_log_tail_{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        let main = dir.join("main.jsonl");
        fs::write(&main, "{\"event\":\"a\"}\n{\"event\":\"b").unwrap();
        fs::write(dir.join("notes.txt"), "not a log\n").unwrap();
        let mut tail = JsonlTail::new(dir.clone());
        let got = |pairs: Vec<(String, String)>| {
            pairs.into_iter().map(|(_, line)| line).collect::<Vec<_>>()
        };
        assert_eq!(got(tail.read()), vec!["{\"event\":\"a\"}"]);

        fs::OpenOptions::new()
            .append(true)
            .open(&main)
            .unwrap()
            .write_all(b"\"}\n")
            .unwrap();
        assert_eq!(got(tail.read()), vec!["{\"event\":\"b\"}"]);

        // what RotatingFileHandler does: a last line to the old file, rename it, start a new one
        fs::OpenOptions::new()
            .append(true)
            .open(&main)
            .unwrap()
            .write_all(b"{\"event\":\"c\"}\n")
            .unwrap();
        fs::rename(&main, dir.join("main.jsonl.1")).unwrap();
        fs::write(&main, "{\"event\":\"d\"}\n").unwrap();
        assert_eq!(
            got(tail.read()),
            vec!["{\"event\":\"c\"}", "{\"event\":\"d\"}"]
        );
        assert_eq!(got(tail.read()), Vec::<String>::new());
        fs::remove_dir_all(dir).unwrap();
    }

    fn rotate(dir: &std::path::Path, backups: u32) {
        for n in (1..backups).rev() {
            let from = dir.join(format!("main.jsonl.{n}"));
            if from.exists() {
                fs::rename(from, dir.join(format!("main.jsonl.{}", n + 1))).unwrap();
            }
        }
        fs::rename(dir.join("main.jsonl"), dir.join("main.jsonl.1")).unwrap();
    }

    #[test]
    fn two_rotations_between_reads_lose_nothing() {
        let dir = std::env::temp_dir().join(format!("system_log_tail2_{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        // a backup from before the module started isn't published
        fs::write(dir.join("main.jsonl.1"), "{\"event\":\"old\"}\n").unwrap();
        fs::write(dir.join("main.jsonl"), "{\"event\":\"a\"}\n").unwrap();
        let mut tail = JsonlTail::new(dir.clone());
        let got = |pairs: Vec<(String, String)>| pairs.into_iter().collect::<Vec<_>>();
        let line = |event: &str| {
            (
                "main.jsonl".to_string(),
                format!("{{\"event\":\"{event}\"}}"),
            )
        };
        assert_eq!(got(tail.read()), vec![line("a")]);

        rotate(&dir, 3);
        fs::write(dir.join("main.jsonl"), "{\"event\":\"b\"}\n").unwrap();
        rotate(&dir, 3);
        fs::write(dir.join("main.jsonl"), "{\"event\":\"c\"}\n").unwrap();
        assert_eq!(got(tail.read()), vec![line("b"), line("c")]);
        assert_eq!(got(tail.read()), vec![]);
        fs::remove_dir_all(dir).unwrap();
    }
}
