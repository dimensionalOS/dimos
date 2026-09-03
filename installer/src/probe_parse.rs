//! The pure parsers behind `Probes`: every one is a fixture test away from the machine it read.

use std::net::Ipv4Addr;
use std::path::Path;

use crate::probe::{Arch, Gpu, Os};

/// L4T release prefix -> JetPack, from NVIDIA's L4T archive; a longer prefix must come first.
const JETPACK: [(&str, &str); 4] = [
    ("R35.3.1", "5.1.1"),
    ("R35.4.1", "5.1.2"),
    ("R36.3", "6.0"),
    ("R36.4", "6.2"),
];

pub(crate) fn parse_os_release(text: &str) -> (String, String) {
    let field = |key: &str| {
        text.lines()
            .find_map(|l| l.trim().strip_prefix(key))
            .map(|v| v.trim_matches('"').to_string())
            .unwrap_or_default()
    };
    (field("ID="), field("VERSION_ID="))
}

/// `# R35 (release), REVISION: 3.1, GCID: ...` -> `R35.3.1`.
pub(crate) fn parse_nv_tegra_release(text: &str) -> Option<String> {
    let major = text.split_whitespace().find(|t| {
        t.len() > 1 && t.starts_with('R') && t[1..].chars().all(|c| c.is_ascii_digit())
    })?;
    let revision = text.split("REVISION:").nth(1)?.split(',').next()?.trim();
    Some(format!("{major}.{revision}"))
}

pub(crate) fn jetpack_for_l4t(l4t: &str) -> Option<&'static str> {
    JETPACK
        .iter()
        .find(|(prefix, _)| l4t.starts_with(prefix))
        .map(|(_, jetpack)| *jetpack)
}

/// `ldd (Ubuntu GLIBC 2.31-0ubuntu9.9) 2.31` -> (2, 31).
pub(crate) fn parse_glibc(ldd_first_line: &str) -> Option<(u32, u32)> {
    let last = ldd_first_line.split_whitespace().last()?;
    let (major, rest) = last.split_once('.')?;
    let minor: String = rest.chars().take_while(char::is_ascii_digit).collect();
    Some((major.parse().ok()?, minor.parse().ok()?))
}

pub(crate) fn is_musl_loader(file_name: &str) -> bool {
    file_name.starts_with("ld-musl-") && file_name.ends_with(".so.1")
}

pub(crate) fn parse_sysctl_value(text: &str) -> Option<u64> {
    text.rsplit('=')
        .next()?
        .split_whitespace()
        .next()?
        .parse()
        .ok()
}

/// (`lo` carries MULTICAST, a 224.0.0.0/4 route exists) — both are needed for LCM on one host.
pub(crate) fn multicast_ok(ip_link_lo: &str, ip_route_224: &str) -> (bool, bool) {
    (
        ip_link_lo.contains("MULTICAST"),
        ip_route_224.contains("224.0.0.0/4"),
    )
}

/// The largest memlock line for `user` or `*`, in bytes; limits.d counts in KiB.
pub(crate) fn memlock_conf_bytes(limits_conf: &str, user: &str) -> Option<u64> {
    limits_conf
        .lines()
        .filter(|l| !l.trim_start().starts_with('#'))
        .filter_map(|l| {
            let f: Vec<&str> = l.split_whitespace().collect();
            (f.len() == 4 && (f[0] == user || f[0] == "*") && f[2] == "memlock").then(|| f[3])
        })
        .filter_map(|value| match value {
            "unlimited" => Some(u64::MAX),
            kib => kib.parse::<u64>().ok().map(|k| k * 1024),
        })
        .max()
}

pub(crate) fn nvpmodel_is_maxn(nvpmodel_q: &str) -> bool {
    nvpmodel_q.contains("MAXN") || nvpmodel_q.contains("ID=0") || nvpmodel_q.contains("ID: 0")
}

/// /proc/device-tree strings are NUL-terminated.
pub(crate) fn parse_device_tree_model(raw: &str) -> String {
    raw.trim_end_matches('\0').trim().to_string()
}

/// First token of each `systemctl list-unit-files --state=enabled --no-legend` line.
pub(crate) fn parse_unit_files(text: &str) -> Vec<String> {
    text.lines()
        .filter_map(|l| l.split_whitespace().next())
        .map(str::to_string)
        .collect()
}

pub(crate) fn parse_iface_ipv4(text: &str, os: &Os) -> Vec<(String, Ipv4Addr)> {
    match os {
        Os::Linux { .. } => parse_ip_o_4(text),
        Os::MacOs { .. } => parse_ifconfig(text),
    }
}

fn parse_ip_o_4(text: &str) -> Vec<(String, Ipv4Addr)> {
    text.lines()
        .filter_map(|line| {
            let fields: Vec<&str> = line.split_whitespace().collect();
            let at = fields.iter().position(|f| *f == "inet")?;
            let addr = fields.get(at + 1)?.split('/').next()?;
            Some((fields.get(1)?.to_string(), addr.parse().ok()?))
        })
        .collect()
}

fn parse_ifconfig(text: &str) -> Vec<(String, Ipv4Addr)> {
    let mut found = Vec::new();
    let mut iface = String::new();
    for line in text.lines() {
        if let Some(name) = line
            .split(':')
            .next()
            .filter(|_| !line.starts_with(char::is_whitespace))
        {
            iface = name.to_string();
        }
        if let Some(rest) = line.trim_start().strip_prefix("inet ") {
            if let Ok(addr) = rest.split_whitespace().next().unwrap_or("").parse() {
                found.push((iface.clone(), addr));
            }
        }
    }
    found
}

/// Tegra wins on a Jetson, which ships no nvidia-smi on JetPack 5.
pub(crate) fn detect_gpu(arch: Arch, os: &Os, jetson: bool, nvidia_smi: Option<&str>) -> Gpu {
    if jetson {
        return Gpu::Tegra;
    }
    match os {
        Os::MacOs { .. } if arch == Arch::Aarch64 => Gpu::AppleSilicon,
        _ if nvidia_smi.is_some_and(|t| t.contains("NVIDIA-SMI")) => Gpu::Nvidia,
        _ => Gpu::None,
    }
}

/// `dir` must be a whole PATH entry: `/home/u/.local/bin2` must not satisfy `/home/u/.local/bin`.
pub(crate) fn path_lists_dir(path: &str, dir: &Path) -> bool {
    path.split(':').any(|entry| Path::new(entry) == dir)
}

pub(crate) fn passwd_shell(passwd: &str, user: &str) -> Option<String> {
    passwd
        .lines()
        .find(|l| l.starts_with(&format!("{user}:")))
        .and_then(|l| l.rsplit(':').next())
        .map(str::to_string)
}

#[cfg(test)]
mod tests {
    use super::*;

    const UBUNTU_20_04: &str = "NAME=\"Ubuntu\"\nVERSION_ID=\"20.04\"\nID=ubuntu\nID_LIKE=debian\n";
    const R35: &str = "# R35 (release), REVISION: 3.1, GCID: 32798077, BOARD: t186ref\n";
    const R36: &str = "# R36 (release), REVISION: 4.3, GCID: 38968081, BOARD: generic\n";

    fn ubuntu() -> Os {
        Os::Linux {
            id: "ubuntu".into(),
            version: "20.04".into(),
        }
    }

    #[test]
    fn os_release_ubuntu_20_04_gives_id_and_version() {
        assert_eq!(
            parse_os_release(UBUNTU_20_04),
            ("ubuntu".to_string(), "20.04".to_string())
        );
    }

    #[test]
    fn nv_tegra_release_r35_3_1_parses_to_jetpack_5_1_1() {
        let l4t = parse_nv_tegra_release(R35).unwrap();
        assert_eq!(l4t, "R35.3.1");
        assert_eq!(jetpack_for_l4t(&l4t), Some("5.1.1"));
    }

    #[test]
    fn r36_revision_4_3_is_jetpack_6_2() {
        let l4t = parse_nv_tegra_release(R36).unwrap();
        assert_eq!(l4t, "R36.4.3");
        assert_eq!(jetpack_for_l4t(&l4t), Some("6.2"));
    }

    #[test]
    fn an_unknown_l4t_has_no_jetpack_rather_than_a_guess() {
        assert_eq!(jetpack_for_l4t("R32.7.1"), None);
    }

    #[test]
    fn jetson_is_detected_without_nvidia_smi_as_tegra() {
        assert_eq!(detect_gpu(Arch::Aarch64, &ubuntu(), true, None), Gpu::Tegra);
        assert_eq!(detect_gpu(Arch::Aarch64, &ubuntu(), false, None), Gpu::None);
        assert_eq!(
            detect_gpu(
                Arch::X86_64,
                &ubuntu(),
                false,
                Some("NVIDIA-SMI 535.183.01  Driver Version: 535.183.01")
            ),
            Gpu::Nvidia
        );
        assert_eq!(
            detect_gpu(
                Arch::Aarch64,
                &Os::MacOs {
                    version: "15.0".into()
                },
                false,
                None
            ),
            Gpu::AppleSilicon
        );
    }

    #[test]
    fn glibc_parsed_from_ubuntu_ldd_first_line() {
        assert_eq!(
            parse_glibc("ldd (Ubuntu GLIBC 2.31-0ubuntu9.9) 2.31"),
            Some((2, 31))
        );
        assert_eq!(parse_glibc("ldd (GNU libc) 2.39"), Some((2, 39)));
    }

    #[test]
    fn musl_loader_present_means_glibc_none() {
        assert!(is_musl_loader("ld-musl-aarch64.so.1"));
        assert!(is_musl_loader("ld-musl-x86_64.so.1"));
        assert!(!is_musl_loader("ld-linux-aarch64.so.1"));
    }

    #[test]
    fn multicast_needed_when_lo_lacks_flag_or_route_missing() {
        let up = "1: lo: <LOOPBACK,MULTICAST,UP,LOWER_UP> mtu 65536";
        let bare = "1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536";
        assert_eq!(
            multicast_ok(up, "224.0.0.0/4 dev lo scope link"),
            (true, true)
        );
        assert_eq!(multicast_ok(bare, ""), (false, false));
    }

    #[test]
    fn memlock_conf_accepts_user_or_star_line_in_kib() {
        let conf = "# dimos\nunitree hard memlock 65536\n* soft memlock 32768\n";
        assert_eq!(memlock_conf_bytes(conf, "unitree"), Some(65536 * 1024));
        assert_eq!(memlock_conf_bytes(conf, "other"), Some(32768 * 1024));
        assert_eq!(memlock_conf_bytes("# nothing\n", "unitree"), None);
    }

    #[test]
    fn nvpmodel_maxn_recognized_in_all_three_spellings() {
        assert!(nvpmodel_is_maxn("NV Power Mode: MAXN\n0\n"));
        assert!(nvpmodel_is_maxn("NVPM VERB: PM_ID=0\n"));
        assert!(nvpmodel_is_maxn("NV Power Mode ID: 0\n"));
        assert!(!nvpmodel_is_maxn("NV Power Mode: 15W\n2\n"));
    }

    #[test]
    fn sysctl_value_parses_bare_and_key_equals_forms() {
        assert_eq!(parse_sysctl_value("212992\n"), Some(212992));
        assert_eq!(
            parse_sysctl_value("net.core.rmem_max = 67108864\n"),
            Some(67108864)
        );
    }

    #[test]
    fn login_path_probe_matches_home_local_bin_only() {
        let home = Path::new("/home/u");
        assert!(path_lists_dir(
            "/usr/bin:/home/u/.local/bin",
            &home.join(".local/bin")
        ));
        assert!(!path_lists_dir(
            "/usr/bin:/home/u/.local/bin2",
            &home.join(".local/bin")
        ));
    }

    #[test]
    fn ip_o_4_addr_gives_one_ipv4_per_interface() {
        let text = "1: lo    inet 127.0.0.1/8 scope host lo\\       valid_lft forever\n\
                    3: eth0    inet 192.168.123.51/24 brd 192.168.123.255 scope global eth0\\   valid\n";
        assert_eq!(
            parse_iface_ipv4(text, &ubuntu()),
            [
                ("lo".to_string(), Ipv4Addr::new(127, 0, 0, 1)),
                ("eth0".to_string(), Ipv4Addr::new(192, 168, 123, 51)),
            ]
        );
    }

    #[test]
    fn ifconfig_pairs_each_inet_with_the_interface_above_it() {
        let text = "lo0: flags=8049<UP,LOOPBACK> mtu 16384\n\
                    \tinet 127.0.0.1 netmask 0xff000000\n\
                    en0: flags=8863<UP,BROADCAST> mtu 1500\n\
                    \tinet6 fe80::1%en0 prefixlen 64\n\
                    \tinet 10.0.0.7 netmask 0xffffff00 broadcast 10.0.0.255\n";
        let mac = Os::MacOs {
            version: "15.0".into(),
        };
        assert_eq!(
            parse_iface_ipv4(text, &mac),
            [
                ("lo0".to_string(), Ipv4Addr::new(127, 0, 0, 1)),
                ("en0".to_string(), Ipv4Addr::new(10, 0, 0, 7)),
            ]
        );
    }

    #[test]
    fn unit_files_are_the_first_token_of_each_line() {
        let text = "dimos-multicast.service                enabled\nssh.service   enabled\n";
        assert_eq!(
            parse_unit_files(text),
            ["dimos-multicast.service", "ssh.service"]
        );
    }

    #[test]
    fn passwd_shell_reads_the_last_field_of_the_users_line() {
        let passwd =
            "root:x:0:0:root:/root:/bin/bash\nunitree:x:1000:1000::/home/unitree:/bin/zsh\n";
        assert_eq!(
            passwd_shell(passwd, "unitree"),
            Some("/bin/zsh".to_string())
        );
        assert_eq!(passwd_shell(passwd, "nobody"), None);
    }

    #[test]
    fn device_tree_model_drops_the_trailing_nul() {
        assert_eq!(
            parse_device_tree_model("NVIDIA Jetson Orin NX Engineering Reference\0"),
            "NVIDIA Jetson Orin NX Engineering Reference"
        );
    }
}
