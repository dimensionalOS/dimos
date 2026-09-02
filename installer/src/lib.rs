//! The DimOS installer. Every command builds a `plan::Plan` from `probe::Probes`; `plan::run`
//! is the only place that mutates the machine or reads stdin. Probes spawn read-only commands
//! through `probe::capture`, each with a deadline.

pub mod cli;
pub mod forward;
pub mod hardware;
pub mod pkgs;
pub mod plan;
pub mod probe;
pub mod robot;
pub mod service;
pub mod setup;
pub mod state;
pub mod sudo;
pub mod uninstall;
pub mod update;
