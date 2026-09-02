//! Four fixed prefixes so the output reads the same in a terminal, over ssh, and in a log file.

use std::io::IsTerminal;

pub fn info(msg: &str) {
    line("->", "\x1b[34m", msg);
}

pub fn ok(msg: &str) {
    line("ok", "\x1b[32m", msg);
}

pub fn warn(msg: &str) {
    line("!!", "\x1b[33m", msg);
}

pub fn fail(msg: &str) {
    line("xx", "\x1b[31m", msg);
}

fn line(prefix: &str, colour: &str, msg: &str) {
    if std::io::stderr().is_terminal() && std::env::var_os("NO_COLOR").is_none() {
        eprintln!("{colour}{prefix}\x1b[0m {msg}");
    } else {
        eprintln!("{prefix} {msg}");
    }
}
