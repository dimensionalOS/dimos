//! The DimOS installer. Every command builds a `plan::Plan` from `probe::Probes`; `plan::run`
//! is the only place that spawns a process, writes a file, or reads stdin.

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
