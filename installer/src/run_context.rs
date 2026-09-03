//! The run's settings: `Mode`, `Ctx`, and the one stdin read in the crate.

use std::io::{IsTerminal, Write};
use std::path::Path;
use std::process::Command;

use anyhow::Result;

use crate::action_log::{self, ActionLog};
use crate::cli::Cli;
use crate::say;
use crate::sudo::Sudo;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Mode {
    Interactive,
    NonInteractive,
    Agent,
}

impl Mode {
    fn from_flags(non_interactive: bool, agent: bool, stdin_is_tty: bool) -> Mode {
        match (agent, non_interactive || !stdin_is_tty) {
            (true, _) => Mode::Agent,
            (false, true) => Mode::NonInteractive,
            (false, false) => Mode::Interactive,
        }
    }
}

pub struct Ctx {
    pub mode: Mode,
    pub dry_run: bool,
    pub verbose: bool,
    pub yes: bool,
    pub sudo: Sudo,
    pub log: ActionLog,
    pub run_id: String,
}

impl Ctx {
    pub fn new(cli: &Cli, home: &Path) -> Result<Ctx> {
        let mode = Mode::from_flags(cli.non_interactive, cli.agent, stdin_is_tty());
        Ok(Ctx {
            mode,
            dry_run: cli.dry_run,
            verbose: cli.verbose,
            yes: cli.yes,
            sudo: Sudo::resolve(mode),
            log: ActionLog::open(home)?,
            run_id: action_log::run_id(),
        })
    }

    pub(crate) fn confirm(&self, question: &str, default: bool) -> Result<bool> {
        if self.yes {
            return Ok(true);
        }
        if self.mode != Mode::Interactive {
            return Ok(default);
        }
        let hint = if default { "[Y/n]" } else { "[y/N]" };
        let answer = ask(&format!("{question} {hint}: "))?;
        Ok(match answer.trim().to_ascii_lowercase().as_str() {
            "" => default,
            other => other.starts_with('y'),
        })
    }

    pub(crate) fn choose(&self, question: &str, options: &[&str], default: usize) -> Result<usize> {
        if self.mode != Mode::Interactive {
            return Ok(default);
        }
        say::info(question);
        for (i, option) in options.iter().enumerate() {
            eprintln!("   {}) {option}", i + 1);
        }
        let answer = ask(&format!("choice [{}]: ", default + 1))?;
        Ok(answer
            .trim()
            .parse::<usize>()
            .ok()
            .filter(|n| (1..=options.len()).contains(n))
            .map_or(default, |n| n - 1))
    }

    pub(crate) fn input(&self, question: &str, default: &str) -> Result<String> {
        if self.mode != Mode::Interactive {
            return Ok(default.to_string());
        }
        let answer = ask(&format!("{question} [{default}]: "))?;
        let trimmed = answer.trim();
        Ok(if trimmed.is_empty() {
            default.to_string()
        } else {
            trimmed.to_string()
        })
    }
}

/// The one stdin read in the crate; the prompt goes to stderr so `--agent` stdout stays JSON.
fn ask(prompt: &str) -> Result<String> {
    eprint!("{prompt}");
    std::io::stderr().flush()?;
    let mut line = String::new();
    std::io::stdin().read_line(&mut line)?;
    Ok(line)
}

pub(crate) fn stdin_is_tty() -> bool {
    std::io::stdin().is_terminal()
}

pub(crate) fn open_shell() {
    let shell = std::env::var("SHELL").unwrap_or_else(|_| "/bin/sh".to_string());
    say::info("type 'exit' when you have finished");
    let _ = Command::new(shell).status();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode_agent_beats_non_interactive_and_a_pipe_is_non_interactive() {
        assert_eq!(Mode::from_flags(false, true, true), Mode::Agent);
        assert_eq!(Mode::from_flags(true, false, true), Mode::NonInteractive);
        assert_eq!(Mode::from_flags(false, false, false), Mode::NonInteractive);
        assert_eq!(Mode::from_flags(false, false, true), Mode::Interactive);
    }
}
