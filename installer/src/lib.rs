//! The DimOS installer. Every command builds a `plan::Plan` from `probe::Probes` and hands it to
//! `run::run`, which mutates the machine only through `file_actions` and reads stdin only through
//! `run_context`. Probes spawn read-only commands through `probe::capture`, each with a deadline.

pub mod action_log;
pub mod cli;
pub mod file_actions;
pub mod install_record;
pub mod plan;
pub mod platforms;
pub mod probe;
pub mod probe_parse;
pub mod robot_scan;
pub mod run;
pub mod run_context;
pub mod say;
pub mod self_update;
pub mod setup;
pub mod spawn;
pub mod sudo;
pub mod systemd_service;
pub mod uninstall;
pub mod update;
pub mod venv_forward;
pub mod version;
pub mod wizards;
