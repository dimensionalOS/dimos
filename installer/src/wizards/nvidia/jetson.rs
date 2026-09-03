//! The NVIDIA Jetson wizard: nvpmodel and jetson_clocks; its `stage` is shared by `setup` and the G1.

use anyhow::{bail, Result};

use crate::cli::HardwareSetupArgs;
use crate::install_record::{Installed, JETSON_CLOCKS_UNIT};
use crate::plan::{Plan, Stage};
use crate::platforms::Platforms;
use crate::probe::{Kernel, Platform, Probes};
use crate::setup::{system_config, verify};
use crate::wizards::{checks, notes, Robot};

const STEP_TIMEOUT_S: u64 = 60;
/// Preloading the two libraries claims their TLS slots before torch's late dlopen asks for them.
pub const LD_PRELOAD_FIX: &str = "export LD_PRELOAD=/lib/aarch64-linux-gnu/libGLdispatch.so.0:\
                                  /usr/lib/aarch64-linux-gnu/libgomp.so.1";

/// nvpmodel + a jetson_clocks unit; empty off a Jetson and empty once both are in place.
pub fn stage(platform: &Platform, kernel: &Kernel) -> Stage {
    let mut stage = Stage::new("jetson perf", false);
    if !platform.is_jetson() {
        return stage;
    }
    if needs_maxn(kernel) {
        // mode 0 is MAXN on every Orin SKU, and it survives a reboot.
        stage = stage.sudo(&["nvpmodel", "-m", "0"], STEP_TIMEOUT_S);
    }
    if platform.systemd && !clocks_unit_enabled(kernel) {
        stage = install_clocks_unit(stage);
    }
    stage
}

/// jetson_clocks resets on every boot, so it runs from a unit rather than once at install time.
pub fn render_clocks_unit() -> String {
    "[Unit]\n\
     Description=DimOS Jetson max clocks\n\
     After=nvpmodel.service\n\
     \n\
     [Service]\n\
     Type=oneshot\n\
     RemainAfterExit=yes\n\
     ExecStart=/usr/bin/jetson_clocks\n\
     \n\
     [Install]\n\
     WantedBy=multi-user.target\n"
        .to_string()
}

/// aarch64 glibc below 2.34 has too few static-TLS slots for a late `dlopen` of libgomp.
pub fn static_tls_note(platform: &Platform) -> Option<String> {
    platform.static_tls_risk().then(|| {
        format!(
            "torch may fail with 'cannot allocate memory in static TLS block' on this glibc; \
             the fix is {LD_PRELOAD_FIX}"
        )
    })
}

/// MAXN raises the power ceiling, so an untethered session drains and heats the robot faster.
pub fn thermal_note(kernel: &Kernel) -> Option<String> {
    needs_maxn(kernel)
        .then(|| "MAXN draws more power and runs hotter: keep the robot on the charger".to_string())
}

fn needs_maxn(kernel: &Kernel) -> bool {
    kernel.nvpmodel_maxn == Some(false)
}

fn clocks_unit_enabled(kernel: &Kernel) -> bool {
    kernel
        .enabled_units
        .iter()
        .any(|u| u.trim_end_matches(".service") == JETSON_CLOCKS_UNIT)
}

fn install_clocks_unit(stage: Stage) -> Stage {
    system_config::unit_actions(JETSON_CLOCKS_UNIT, render_clocks_unit())
        .into_iter()
        .fold(stage, Stage::push)
}

pub(crate) fn ready(probes: &Probes) -> Result<()> {
    if probes.platform.is_jetson() {
        return Ok(());
    }
    bail!("not a Jetson: /etc/nv_tegra_release is missing; a plain Linux host needs `dimos setup`")
}

/// The standalone Orin (brief decision 14): performance mode and machine config, then verify.
pub fn plan(
    args: &HardwareSetupArgs,
    probes: &Probes,
    cfg: &Platforms,
    installed: &Installed,
) -> Plan {
    let mut stages = vec![
        stage(&probes.platform, &probes.kernel),
        system_config::stage(&probes.platform, &probes.kernel, cfg),
    ];
    stages.extend(checks(verify::Target::Jetson, installed, args));
    Plan {
        command: Robot::Jetson.command(),
        stages,
        notes: notes(probes),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan::Action;
    use crate::probe::{Arch, Gpu, Jetson, Os, PkgManager};
    use crate::probe_parse::nvpmodel_is_maxn;
    use std::path::PathBuf;

    /// JetPack 6 on an Orin NX already at max performance.
    const JP6_MAXN: &str = "NV Fan Mode:quiet\nNV Power Mode: MAXN\n0\n";
    /// JetPack 5.1.1 as the G1 ships it.
    const JP5_15W: &str = "NV Fan Mode:quiet\nNV Power Mode: MODE_15W\n2\n";

    fn orin(systemd: bool) -> Platform {
        Platform {
            os: Os::Linux {
                id: "ubuntu".into(),
                version: "20.04".into(),
            },
            arch: Arch::Aarch64,
            glibc: Some((2, 31)),
            jetson: Some(Jetson {
                l4t: "R35.3.1".into(),
                jetpack: Some("5.1.1"),
                model: "NVIDIA Orin NX".into(),
            }),
            gpu: Gpu::Tegra,
            pkg: PkgManager::Apt,
            systemd,
            home: PathBuf::from("/home/unitree"),
            user: "unitree".into(),
            shell: PathBuf::from("/bin/bash"),
        }
    }

    fn laptop() -> Platform {
        Platform {
            jetson: None,
            arch: Arch::X86_64,
            glibc: Some((2, 39)),
            gpu: Gpu::None,
            ..orin(true)
        }
    }

    fn kernel(nvpmodel_q: &str, units: &[&str]) -> Kernel {
        Kernel {
            nvpmodel_maxn: Some(nvpmodel_is_maxn(nvpmodel_q)),
            enabled_units: units.iter().map(|u| (*u).to_string()).collect(),
            ..Kernel::default()
        }
    }

    fn argvs(stage: &Stage) -> Vec<Vec<String>> {
        stage
            .actions
            .iter()
            .filter_map(|a| match a {
                Action::Run { argv, .. } => Some(argv.clone()),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn off_jetson_is_empty() {
        let stage = stage(&laptop(), &kernel(JP5_15W, &[]));
        assert!(stage.actions.is_empty(), "{:?}", stage.actions);
    }

    #[test]
    fn maxn_fixture_plans_no_nvpmodel() {
        let stage = stage(&orin(true), &kernel(JP6_MAXN, &[]));
        assert!(!argvs(&stage).iter().any(|a| a[0] == "nvpmodel"));
    }

    #[test]
    fn mode_15w_plans_nvpmodel_m0_with_sudo() {
        let stage = stage(&orin(true), &kernel(JP5_15W, &[]));
        assert_eq!(
            stage.actions[0],
            Action::sudo(&["nvpmodel", "-m", "0"], STEP_TIMEOUT_S)
        );
    }

    #[test]
    fn configured_jetson_plans_nothing() {
        let units = ["dimos-jetson-clocks.service"];
        let stage = stage(&orin(true), &kernel(JP6_MAXN, &units));
        assert!(stage.actions.is_empty(), "{:?}", stage.actions);
    }

    #[test]
    fn clocks_unit_is_written_then_reloaded_then_enabled() {
        let stage = stage(&orin(true), &kernel(JP6_MAXN, &[]));
        assert_eq!(
            stage.actions[0],
            Action::WriteFile {
                path: PathBuf::from("/etc/systemd/system/dimos-jetson-clocks.service"),
                mode: 0o644,
                contents: render_clocks_unit(),
                sudo: true,
            }
        );
        assert_eq!(
            argvs(&stage),
            vec![
                vec!["systemctl".to_string(), "daemon-reload".to_string()],
                vec![
                    "systemctl".to_string(),
                    "enable".to_string(),
                    "--now".to_string(),
                    "dimos-jetson-clocks".to_string()
                ],
            ]
        );
    }

    #[test]
    fn without_systemd_only_nvpmodel_is_planned() {
        let stage = stage(&orin(false), &kernel(JP5_15W, &[]));
        assert_eq!(argvs(&stage), vec![vec!["nvpmodel", "-m", "0"]]);
    }

    #[test]
    fn clocks_unit_runs_after_nvpmodel_and_is_oneshot() {
        let unit = render_clocks_unit();
        assert!(unit.contains("After=nvpmodel.service"), "{unit}");
        assert!(unit.contains("Type=oneshot"), "{unit}");
        assert!(unit.contains("ExecStart=/usr/bin/jetson_clocks"), "{unit}");
    }

    #[test]
    fn static_tls_note_only_on_glibc_below_2_34_and_carries_both_libraries() {
        let note = static_tls_note(&orin(true)).expect("glibc 2.31 aarch64 is at risk");
        assert!(
            note.contains("/lib/aarch64-linux-gnu/libGLdispatch.so.0"),
            "{note}"
        );
        assert!(
            note.contains("/usr/lib/aarch64-linux-gnu/libgomp.so.1"),
            "{note}"
        );
        let mut safe = orin(true);
        safe.glibc = Some((2, 35));
        assert_eq!(static_tls_note(&safe), None);
    }

    #[test]
    fn thermal_note_only_when_nvpmodel_is_planned() {
        assert!(thermal_note(&kernel(JP5_15W, &[])).is_some());
        assert_eq!(thermal_note(&kernel(JP6_MAXN, &[])), None);
        assert_eq!(thermal_note(&Kernel::default()), None);
    }
}
