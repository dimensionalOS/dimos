//! Linux machine-config persistence, shared by `setup`, `update` and every `hardware <t> setup`:
//! LCM's UDP receive buffers, loopback multicast, and zenoh's memlock floor. Pure over probes.

use std::collections::BTreeMap;
use std::path::PathBuf;

use crate::action::Action;
use crate::install_record;
use crate::plan::Stage;
use crate::platforms::{self, Platforms};
use crate::probe::{Kernel, Os, Platform};

/// sysctl, ip and systemctl all return immediately; the budget only bounds a hung sudo.
const CONFIG_TIMEOUT_S: u64 = 30;

const NO_SYSTEMD: &str =
    "no systemd: machine config not persisted; the runtime configurators apply it per run";

/// Non-critical: sudo actions only where a probe shows a gap, so a re-run plans nothing.
pub fn stage(platform: &Platform, kernel: &Kernel, cfg: &Platforms) -> Stage {
    let stage = Stage::new("sysconfig", false);
    if !persistable(platform) {
        return stage;
    }
    sysctl_actions(kernel, &cfg.linux.sysctl)
        .into_iter()
        .chain(multicast_actions(kernel))
        .chain(memlock_actions(
            &platform.user,
            kernel,
            cfg.linux.memlock_bytes,
        ))
        .fold(stage, Stage::push)
}

/// Every persistence path here is a systemd unit or a boot-time drop-in, so without it nothing sticks.
fn persistable(platform: &Platform) -> bool {
    matches!(platform.os, Os::Linux { .. }) && platform.systemd
}

pub fn no_systemd_note(platform: &Platform) -> Option<String> {
    let linux = matches!(platform.os, Os::Linux { .. });
    (linux && !platform.systemd).then(|| NO_SYSTEMD.to_string())
}

fn write_root(path: PathBuf, contents: String) -> Action {
    Action::WriteFile {
        path,
        mode: 0o644,
        contents,
        sudo: true,
    }
}

fn sysctl_actions(kernel: &Kernel, target: &BTreeMap<String, u64>) -> Vec<Action> {
    let conf = render_sysctl_conf(target);
    let mut actions: Vec<Action> = platforms::sysctl_updates(&kernel.sysctl, target)
        .iter()
        .map(|(key, value)| {
            let setting = format!("{key}={value}");
            Action::sudo(&["sysctl", "-w", &setting], CONFIG_TIMEOUT_S)
        })
        .collect();
    if kernel.sysctl_conf.as_deref() != Some(conf.as_str()) {
        actions.push(write_root(PathBuf::from(install_record::SYSCTL_CONF), conf));
    }
    actions
}

/// The unit persists across reboots; the two `ip` calls close the gap in the running kernel now.
fn multicast_actions(kernel: &Kernel) -> Vec<Action> {
    let unit = format!("{}.service", install_record::MULTICAST_UNIT);
    let mut actions = Vec::new();
    if !kernel.lo_multicast {
        actions.push(Action::sudo(
            &["ip", "link", "set", "lo", "multicast", "on"],
            CONFIG_TIMEOUT_S,
        ));
    }
    if !kernel.multicast_route {
        actions.push(Action::sudo(
            &["ip", "route", "add", "224.0.0.0/4", "dev", "lo"],
            CONFIG_TIMEOUT_S,
        ));
    }
    if !kernel.enabled_units.contains(&unit) {
        actions.extend(unit_actions(
            install_record::MULTICAST_UNIT,
            render_multicast_unit(),
        ));
    }
    actions
}

/// Write a unit as root, reload, `enable --now`: what every unit the installer owns needs.
pub fn unit_actions(name: &str, contents: String) -> Vec<Action> {
    vec![
        write_root(install_record::unit_path(name), contents),
        Action::sudo(&["systemctl", "daemon-reload"], CONFIG_TIMEOUT_S),
        Action::sudo(&["systemctl", "enable", "--now", name], CONFIG_TIMEOUT_S),
    ]
}

/// A bigger existing limit is somebody else's tuning; only a shortfall is planned.
fn memlock_actions(user: &str, kernel: &Kernel, bytes: u64) -> Vec<Action> {
    if kernel.memlock_conf_bytes.is_some_and(|have| have >= bytes) {
        return Vec::new();
    }
    vec![write_root(
        PathBuf::from(install_record::MEMLOCK_CONF),
        render_memlock_conf(user, bytes),
    )]
}

pub fn render_sysctl_conf(targets: &BTreeMap<String, u64>) -> String {
    let mut conf = String::from("# Written by dimos: LCM's UDP receive buffers.\n");
    for (key, value) in targets {
        conf.push_str(&format!("{key}={value}\n"));
    }
    conf
}

pub fn render_multicast_unit() -> String {
    // `-` on the route line: a route that already exists exits 2, which must not fail the unit.
    "[Unit]\n\
     Description=DimOS loopback multicast for LCM\n\
     After=network.target\n\
     \n\
     [Service]\n\
     Type=oneshot\n\
     RemainAfterExit=yes\n\
     ExecStart=/sbin/ip link set lo multicast on\n\
     ExecStart=-/sbin/ip route add 224.0.0.0/4 dev lo\n\
     \n\
     [Install]\n\
     WantedBy=multi-user.target\n"
        .to_string()
}

/// pam_limits counts in KiB and applies at next login; same shape as zenoh.py `_persist`.
pub fn render_memlock_conf(user: &str, bytes: u64) -> String {
    format!(
        "# Written by dimos: zenoh's SHM pool must be lockable.\n{user}\t-\tmemlock\t{}\n",
        bytes / 1024
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::probe::{Arch, Gpu, PkgManager};

    fn cfg() -> Platforms {
        Platforms::load()
    }

    fn platform(os: Os, systemd: bool) -> Platform {
        Platform {
            os,
            arch: Arch::Aarch64,
            glibc: Some((2, 31)),
            jetson: None,
            gpu: Gpu::None,
            pkg: PkgManager::Apt,
            systemd,
            home: PathBuf::from("/home/unitree"),
            user: "unitree".to_string(),
            shell: PathBuf::from("/bin/bash"),
        }
    }

    fn ubuntu(systemd: bool) -> Platform {
        platform(
            Os::Linux {
                id: "ubuntu".to_string(),
                version: "20.04".to_string(),
            },
            systemd,
        )
    }

    fn configured(cfg: &Platforms) -> Kernel {
        Kernel {
            sysctl: cfg.linux.sysctl.clone(),
            lo_multicast: true,
            multicast_route: true,
            memlock_conf_bytes: Some(cfg.linux.memlock_bytes),
            nvpmodel_maxn: None,
            sysctl_conf: Some(render_sysctl_conf(&cfg.linux.sysctl)),
            enabled_units: vec![format!("{}.service", install_record::MULTICAST_UNIT)],
        }
    }

    fn displays(stage: &Stage) -> String {
        stage
            .actions
            .iter()
            .map(Action::display)
            .collect::<Vec<_>>()
            .join("\n")
    }

    #[test]
    fn a_machine_already_at_target_plans_nothing() {
        let cfg = cfg();
        let stage = stage(&ubuntu(true), &configured(&cfg), &cfg);
        assert_eq!(stage.actions, []);
        assert!(!stage.critical);
        assert_eq!(stage.name, "sysconfig");
    }

    #[test]
    fn a_kernel_below_target_plans_sysctl_w_and_persists_every_key_with_sudo() {
        let cfg = cfg();
        let stock = Kernel {
            sysctl: cfg
                .linux
                .sysctl
                .keys()
                .map(|k| (k.clone(), 212_992))
                .collect(),
            sysctl_conf: None,
            ..configured(&cfg)
        };
        let stage = stage(&ubuntu(true), &stock, &cfg);
        assert!(stage.needs_sudo());
        let text = displays(&stage);
        for (key, value) in &cfg.linux.sysctl {
            assert!(text.contains(&format!("sysctl -w {key}={value}")), "{text}");
        }
        assert!(
            text.contains(&format!("write {}", install_record::SYSCTL_CONF)),
            "{text}"
        );
        let conf = render_sysctl_conf(&cfg.linux.sysctl);
        assert_eq!(conf.lines().count(), cfg.linux.sysctl.len() + 1);
    }

    #[test]
    fn a_current_sysctl_conf_is_not_rewritten_when_only_the_running_kernel_is_low() {
        let cfg = cfg();
        let low = Kernel {
            sysctl: BTreeMap::new(),
            ..configured(&cfg)
        };
        let text = displays(&stage(&ubuntu(true), &low, &cfg));
        assert!(!text.contains(install_record::SYSCTL_CONF), "{text}");
        assert!(text.contains("sysctl -w"), "{text}");
    }

    #[test]
    fn a_multicast_gap_plans_both_ip_commands_and_enables_the_unit() {
        let cfg = cfg();
        let bare = Kernel {
            lo_multicast: false,
            multicast_route: false,
            enabled_units: Vec::new(),
            ..configured(&cfg)
        };
        let text = displays(&stage(&ubuntu(true), &bare, &cfg));
        assert!(text.contains("sudo ip link set lo multicast on"), "{text}");
        assert!(
            text.contains("sudo ip route add 224.0.0.0/4 dev lo"),
            "{text}"
        );
        assert!(text.contains("systemctl daemon-reload"), "{text}");
        assert!(
            text.contains(&format!(
                "systemctl enable --now {}",
                install_record::MULTICAST_UNIT
            )),
            "{text}"
        );
        assert!(
            text.contains(
                &install_record::unit_path(install_record::MULTICAST_UNIT)
                    .display()
                    .to_string()
            ),
            "{text}"
        );
    }

    #[test]
    fn an_enabled_unit_with_a_live_gap_still_plans_the_ip_commands() {
        let cfg = cfg();
        let dropped = Kernel {
            lo_multicast: false,
            ..configured(&cfg)
        };
        let text = displays(&stage(&ubuntu(true), &dropped, &cfg));
        assert!(text.contains("sudo ip link set lo multicast on"), "{text}");
        assert!(!text.contains("systemctl"), "{text}");
    }

    #[test]
    fn the_memlock_conf_names_the_invoking_user_and_kib_never_a_star() {
        let conf = render_memlock_conf("unitree", 67_108_864);
        assert_eq!(conf.lines().last(), Some("unitree\t-\tmemlock\t65536"));
        assert!(!conf.contains('*'), "{conf}");
    }

    #[test]
    fn a_memlock_limit_above_target_is_left_alone_and_a_shortfall_is_planned() {
        let cfg = cfg();
        let generous = Kernel {
            memlock_conf_bytes: Some(cfg.linux.memlock_bytes * 2),
            ..configured(&cfg)
        };
        assert_eq!(stage(&ubuntu(true), &generous, &cfg).actions, []);
        let short = Kernel {
            memlock_conf_bytes: None,
            ..configured(&cfg)
        };
        let text = displays(&stage(&ubuntu(true), &short, &cfg));
        assert!(
            text.contains(&format!("sudo write {}", install_record::MEMLOCK_CONF)),
            "{text}"
        );
    }

    #[test]
    fn without_systemd_nothing_is_planned_and_the_note_says_the_runtime_does_it() {
        let cfg = cfg();
        let bare = Kernel::default();
        assert_eq!(stage(&ubuntu(false), &bare, &cfg).actions, []);
        assert_eq!(no_systemd_note(&ubuntu(false)).as_deref(), Some(NO_SYSTEMD));
        assert_eq!(no_systemd_note(&ubuntu(true)), None);
    }

    #[test]
    fn macos_has_no_actions_and_no_note() {
        let cfg = cfg();
        let mac = platform(
            Os::MacOs {
                version: "15.0".to_string(),
            },
            true,
        );
        assert_eq!(stage(&mac, &Kernel::default(), &cfg).actions, []);
        assert_eq!(no_systemd_note(&mac), None);
    }

    #[test]
    fn the_multicast_unit_is_a_oneshot_that_survives_an_existing_route() {
        let unit = render_multicast_unit();
        assert!(unit.contains("Type=oneshot"), "{unit}");
        assert!(unit.contains("RemainAfterExit=yes"), "{unit}");
        assert!(
            unit.contains("ExecStart=-/sbin/ip route add 224.0.0.0/4 dev lo"),
            "{unit}"
        );
        assert!(unit.contains("WantedBy=multi-user.target"), "{unit}");
    }
}
