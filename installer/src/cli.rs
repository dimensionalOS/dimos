//! The clap surface. Unknown verbs, `hardware <other>` and `hardware g1|jetson <other>` are
//! external subcommands so main can hand them to the DimOS Python CLI unchanged.

use std::ffi::OsString;
use std::path::PathBuf;

use clap::{Args, Parser, Subcommand, ValueEnum};
use serde::{Deserialize, Serialize};

const FORWARD_NOTE: &str = "\
Any other command runs the DimOS Python CLI, e.g. `dimos run unitree-go2`.
Installer flags go after the verb (`dimos setup --agent`); before it they are forwarded too.
`dimos hardware g1|jetson setup` runs here, every other `hardware ...` verb runs the Python CLI.";

#[derive(Debug, Parser)]
#[command(
    name = "dimos",
    version = env!("DIMOS_VERSION"),
    about = "Install DimOS and bring up robot hardware",
    after_help = FORWARD_NOTE,
)]
pub struct Cli {
    /// Print the plan and touch nothing.
    #[arg(long, global = true)]
    pub dry_run: bool,
    /// Never prompt; also implied when stdin is not a terminal.
    #[arg(long, global = true)]
    pub non_interactive: bool,
    /// Grant consent for system packages and sudo up front.
    #[arg(long, global = true)]
    pub yes: bool,
    /// One JSON plan then one JSON report on stdout; progress on stderr.
    #[arg(long, global = true)]
    pub agent: bool,
    /// Echo every child process line as it arrives.
    #[arg(long, global = true)]
    pub verbose: bool,
    #[command(subcommand)]
    pub command: Command,
}

#[derive(Debug, Subcommand)]
pub enum Command {
    /// Install uv, a Python 3.12 venv, DimOS, and this machine's configuration.
    Setup(SetupArgs),
    /// Update DimOS and repair the machine config; `--dry-run` is the read-only report.
    Update(UpdateArgs),
    /// Run a blueprint as a systemd service.
    Service {
        #[command(subcommand)]
        action: ServiceAction,
    },
    /// Bring up a robot from the machine it runs on.
    Hardware {
        #[command(subcommand)]
        target: HardwareTarget,
    },
    /// Find robots on the network.
    Robot {
        #[command(subcommand)]
        action: RobotAction,
    },
    /// Remove what setup installed, listing what it leaves behind.
    Uninstall,
    #[command(external_subcommand)]
    External(Vec<OsString>),
}

#[derive(Debug, Args)]
pub struct SetupArgs {
    #[arg(long, value_enum)]
    pub mode: Option<InstallMode>,
    #[arg(long, value_delimiter = ',')]
    pub extras: Vec<String>,
    #[arg(long, env = "DIMOS_BRANCH")]
    pub branch: Option<String>,
    #[arg(long)]
    pub dir: Option<PathBuf>,
    /// The blueprint the verify stage must find in `dimos list`.
    #[arg(long)]
    pub blueprint: Option<String>,
    #[arg(long)]
    pub with_nix: bool,
}

#[derive(Debug, Args)]
pub struct UpdateArgs {
    /// Install this installer release instead of the newest one.
    #[arg(long)]
    pub version: Option<String>,
    /// Re-apply every stage even when its probe says the machine is already there.
    #[arg(long)]
    pub force: bool,
    /// Put the kept `.bak` binary back.
    #[arg(long)]
    pub rollback: bool,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, ValueEnum, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum InstallMode {
    /// `dimos[<extras>]==<version>` from PyPI.
    Library,
    /// A git clone plus `uv sync`.
    Dev,
}

#[derive(Debug, Subcommand)]
pub enum ServiceAction {
    Setup {
        blueprint: String,
        #[arg(long = "env")]
        env: Vec<String>,
    },
    Start {
        blueprint: String,
    },
    Stop {
        blueprint: String,
    },
    Restart {
        blueprint: String,
    },
    Status {
        blueprint: String,
    },
    Remove {
        blueprint: String,
    },
}

#[derive(Debug, Subcommand)]
pub enum HardwareTarget {
    /// Unitree G1, run on the robot's Jetson.
    G1 {
        #[command(subcommand)]
        verb: HardwareVerb,
    },
    /// A standalone Jetson Orin.
    Jetson {
        #[command(subcommand)]
        verb: HardwareVerb,
    },
    #[command(external_subcommand)]
    Other(Vec<OsString>),
}

#[derive(Debug, Subcommand)]
pub enum HardwareVerb {
    Setup(HardwareSetupArgs),
    #[command(external_subcommand)]
    Other(Vec<OsString>),
}

#[derive(Debug, Clone, Args)]
pub struct HardwareSetupArgs {
    /// G1Config.ip dials the control computer; the Jetson itself is .164.
    #[arg(long, default_value = "192.168.123.161")]
    pub robot_ip: String,
    #[arg(long, env = "ROBOT_INTERFACE", default_value = "eth0")]
    pub interface: String,
    /// Written to `.env` as DIMOS_TRANSPORT.
    #[arg(long, value_enum)]
    pub transport: Option<Transport>,
    #[arg(long, env = "SDK2_PATH")]
    pub sdk_path: Option<PathBuf>,
    /// The blueprint the verify stage must find in `dimos list`.
    #[arg(long)]
    pub blueprint: Option<String>,
}

/// The two values dimos/core/global_config.py accepts for DIMOS_TRANSPORT.
#[derive(Clone, Copy, PartialEq, Eq, Debug, ValueEnum)]
pub enum Transport {
    Lcm,
    Zenoh,
}

impl Transport {
    pub(crate) fn name(self) -> &'static str {
        match self {
            Transport::Lcm => "lcm",
            Transport::Zenoh => "zenoh",
        }
    }
}

#[derive(Debug, Subcommand)]
pub enum RobotAction {
    /// Print what is on the network and exit; v1 saves nothing.
    Scan(ScanArgs),
}

#[derive(Debug, Args)]
pub struct ScanArgs {
    /// Unitree multicast probe on every non-VPN interface.
    #[arg(long)]
    pub lan: bool,
    /// The wired 192.168.123.0/24 addresses.
    #[arg(long)]
    pub wired: bool,
    /// BLE through the installed DimOS.
    #[arg(long)]
    pub ble: bool,
    #[arg(long = "timeout", value_name = "S", default_value_t = 3)]
    pub timeout_s: u64,
}

impl ScanArgs {
    /// No flag means every kind.
    pub(crate) fn all_kinds(&self) -> bool {
        !self.lan && !self.wired && !self.ble
    }
}

/// The argv the Python CLI expects for a `hardware ...` verb we do not own.
pub fn hardware_argv(target: &HardwareTarget) -> Vec<OsString> {
    let mut argv = vec![OsString::from("hardware")];
    match target {
        HardwareTarget::Other(rest) => argv.extend(rest.iter().cloned()),
        HardwareTarget::G1 { verb } => push_verb(&mut argv, "g1", verb),
        HardwareTarget::Jetson { verb } => push_verb(&mut argv, "jetson", verb),
    }
    argv
}

fn push_verb(argv: &mut Vec<OsString>, name: &str, verb: &HardwareVerb) {
    argv.push(OsString::from(name));
    if let HardwareVerb::Other(rest) = verb {
        argv.extend(rest.iter().cloned());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;

    fn parse(args: &[&str]) -> Cli {
        Cli::try_parse_from(args).unwrap_or_else(|e| panic!("{args:?}: {e}"))
    }

    #[test]
    fn the_clap_surface_is_internally_consistent() {
        Cli::command().debug_assert();
    }

    #[test]
    fn unknown_verb_parses_as_external_with_args_intact() {
        let Command::External(argv) =
            parse(&["dimos", "run", "unitree-go2", "--robot-ip", "x"]).command
        else {
            panic!("want External")
        };
        assert_eq!(argv, ["run", "unitree-go2", "--robot-ip", "x"]);
    }

    #[test]
    fn hardware_a1z_doctor_parses_as_other_target() {
        let Command::Hardware { target } = parse(&["dimos", "hardware", "a1z", "doctor"]).command
        else {
            panic!("want Hardware")
        };
        assert!(matches!(target, HardwareTarget::Other(_)));
        assert_eq!(hardware_argv(&target), ["hardware", "a1z", "doctor"]);
    }

    #[test]
    fn hardware_g1_setup_is_ours_and_any_other_g1_verb_forwards() {
        let Command::Hardware { target } = parse(&["dimos", "hardware", "g1", "setup"]).command
        else {
            panic!("want Hardware")
        };
        assert!(matches!(
            target,
            HardwareTarget::G1 {
                verb: HardwareVerb::Setup(_)
            }
        ));
        let Command::Hardware { target } =
            parse(&["dimos", "hardware", "g1", "calibrate", "--fast"]).command
        else {
            panic!("want Hardware")
        };
        assert!(matches!(
            target,
            HardwareTarget::G1 {
                verb: HardwareVerb::Other(_)
            }
        ));
    }

    #[test]
    fn hardware_argv_rebuilds_full_python_argv() {
        let Command::Hardware { target } =
            parse(&["dimos", "hardware", "jetson", "profile", "--json"]).command
        else {
            panic!("want Hardware")
        };
        assert_eq!(
            hardware_argv(&target),
            ["hardware", "jetson", "profile", "--json"]
        );
    }

    #[test]
    fn global_flags_accepted_after_subcommand() {
        assert!(parse(&["dimos", "setup", "--dry-run", "--agent"]).agent);
        assert!(parse(&["dimos", "--dry-run", "setup"]).dry_run);
        assert!(parse(&["dimos", "update", "--yes"]).yes);
    }

    #[test]
    fn extras_split_on_comma() {
        let Command::Setup(args) = parse(&["dimos", "setup", "--extras", "base,unitree"]).command
        else {
            panic!("want Setup")
        };
        assert_eq!(args.extras, ["base", "unitree"]);
        assert_eq!(args.blueprint, None);
    }

    #[test]
    fn g1_setup_defaults() {
        let Command::Hardware {
            target:
                HardwareTarget::G1 {
                    verb: HardwareVerb::Setup(args),
                },
        } = parse(&["dimos", "hardware", "g1", "setup"]).command
        else {
            panic!("want g1 setup")
        };
        assert_eq!(args.robot_ip, "192.168.123.161");
        assert_eq!(args.interface, "eth0");
        assert_eq!(args.transport, None);
        assert_eq!(args.blueprint, None);
    }

    #[test]
    fn transport_is_lcm_or_zenoh_and_a_typo_is_refused_at_parse_time() {
        let Command::Hardware {
            target:
                HardwareTarget::G1 {
                    verb: HardwareVerb::Setup(args),
                },
        } = parse(&["dimos", "hardware", "g1", "setup", "--transport", "zenoh"]).command
        else {
            panic!("want g1 setup")
        };
        assert_eq!(args.transport, Some(Transport::Zenoh));
        assert_eq!(Transport::Lcm.name(), "lcm");
        assert!(
            Cli::try_parse_from(["dimos", "hardware", "g1", "setup", "--transport", "dds"])
                .is_err()
        );
    }

    #[test]
    fn robot_scan_defaults_to_every_kind_with_a_three_second_timeout() {
        let Command::Robot {
            action: RobotAction::Scan(args),
        } = parse(&["dimos", "robot", "scan"]).command
        else {
            panic!("want robot scan")
        };
        assert!(args.all_kinds());
        assert_eq!(args.timeout_s, 3);
        let Command::Robot {
            action: RobotAction::Scan(args),
        } = parse(&["dimos", "robot", "scan", "--lan", "--timeout", "8"]).command
        else {
            panic!("want robot scan")
        };
        assert!(!args.all_kinds() && args.lan && args.timeout_s == 8);
    }

    #[test]
    fn update_takes_a_version_a_force_and_a_rollback() {
        let Command::Update(args) = parse(&["dimos", "update", "--version", "v0.0.14b1"]).command
        else {
            panic!("want Update")
        };
        assert_eq!(args.version.as_deref(), Some("v0.0.14b1"));
        assert!(!args.force && !args.rollback);
    }

    #[test]
    fn there_is_no_doctor_command() {
        assert!(Cli::try_parse_from(["dimos", "doctor"])
            .is_ok_and(|c| matches!(c.command, Command::External(_))));
    }
}
