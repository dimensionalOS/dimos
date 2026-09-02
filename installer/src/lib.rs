//! The DimOS installer. Every command builds a `plan::Plan` from `probe::Probes`; `run::run`
//! is the only place that mutates the machine or reads stdin. Probes spawn read-only commands
//! through `probe::capture`, each with a deadline.

pub mod action_log;
pub mod cli;
pub mod install_record;
pub mod plan;
pub mod platforms;
pub mod probe;
pub mod robot_scan;
pub mod run;
pub mod say;
pub mod setup;
pub mod sudo;
pub mod systemd_service;
pub mod uninstall;
pub mod update;
pub mod venv_forward;
pub mod wizards;
