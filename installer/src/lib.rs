//! The DimOS installer. Every command builds a `plan::Plan` from `probe::Probes`; `run::run`
//! is the only place that mutates the machine or reads stdin. Probes spawn read-only commands
//! through `probe::capture`, each with a deadline.

pub mod action_log;
pub mod cli;
pub mod forward;
pub mod hardware;
pub mod install_record;
pub mod pkgs;
pub mod plan;
pub mod probe;
pub mod robot;
pub mod run;
pub mod say;
pub mod service;
pub mod setup;
pub mod sudo;
pub mod uninstall;
pub mod update;
