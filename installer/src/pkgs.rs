//! `platforms.toml` plus the build-time extras list: validation, system-package needs, and the
//! pip/uv specs. Every "what does this machine still need" answer is pure over captured output.

use std::collections::BTreeMap;

use anyhow::{bail, Result};
use serde::Deserialize;

use crate::probe::{Arch, PkgManager};

include!(concat!(env!("OUT_DIR"), "/extras.rs")); // pub const EXTRAS: &[&str], from pyproject

/// pyproject `[project].version`; installer and package ship on the same tag by construction.
pub const DIMOS_VERSION: &str = env!("DIMOS_VERSION");

#[derive(Debug, Deserialize)]
pub struct Platforms {
    pub always: Pkgs,
    #[serde(default)]
    pub extras: BTreeMap<String, Extra>,
    pub linux: Linux,
}

#[derive(Debug, Default, Deserialize)]
pub struct Pkgs {
    #[serde(default)]
    pub apt: Vec<String>,
    #[serde(default)]
    pub brew: Vec<String>,
}

#[derive(Debug, Default, Deserialize)]
pub struct Extra {
    #[serde(default)]
    pub apt: Vec<String>,
    #[serde(default)]
    pub brew: Vec<String>,
    #[serde(default)]
    pub only_arch: Option<Vec<String>>,
}

#[derive(Debug, Deserialize)]
pub struct Linux {
    pub sysctl: BTreeMap<String, u64>,
    pub memlock_bytes: u64,
}

impl Platforms {
    pub fn load() -> Platforms {
        toml::from_str(include_str!("../platforms.toml"))
            .expect("platforms.toml is compiled in and parsed by platforms_toml_parses_*")
    }
}

fn for_manager<'a>(apt: &'a [String], brew: &'a [String], pm: PkgManager) -> &'a [String] {
    match pm {
        PkgManager::Apt => apt,
        PkgManager::Brew => brew,
        PkgManager::None => &[],
    }
}

fn normalize(extras: &[String]) -> Vec<String> {
    let mut wanted: Vec<String> = Vec::new();
    for extra in extras.iter().map(|e| e.trim()).filter(|e| !e.is_empty()) {
        if !wanted.iter().any(|w| w == extra) {
            wanted.push(extra.to_string());
        }
    }
    if wanted.is_empty() {
        wanted.push("base".to_string());
    }
    wanted
}

fn check_extra(extra: &str, arch: Arch, cfg: &Platforms) -> Result<()> {
    if !EXTRAS.contains(&extra) {
        bail!(
            "unknown extra '{extra}': pyproject offers {}",
            EXTRAS.join(", ")
        );
    }
    let Some(only) = cfg.extras.get(extra).and_then(|e| e.only_arch.as_ref()) else {
        return Ok(());
    };
    if only.iter().any(|a| a == arch.name()) {
        return Ok(());
    }
    bail!(
        "{extra} is {0}-only: pyproject gates its wheels to platform_machine == '{0}', \
         and this machine is {1}; drop it, or use the cpu extra",
        only.join("/"),
        arch.name()
    )
}

/// Trimmed, de-duplicated, `["base"]` when empty; an unknown or wrong-arch extra is a hard error.
pub fn validate_extras(extras: &[String], arch: Arch, cfg: &Platforms) -> Result<Vec<String>> {
    let wanted = normalize(extras);
    for extra in &wanted {
        check_extra(extra, arch, cfg)?;
    }
    Ok(wanted)
}

pub fn system_packages(extras: &[String], pm: PkgManager, cfg: &Platforms) -> Vec<String> {
    let mut wanted = for_manager(&cfg.always.apt, &cfg.always.brew, pm).to_vec();
    for extra in extras.iter().filter_map(|e| cfg.extras.get(e)) {
        wanted.extend_from_slice(for_manager(&extra.apt, &extra.brew, pm));
    }
    wanted.sort();
    wanted.dedup();
    wanted
}

/// Pure over `dpkg-query -W -f='${Package} ${Status}\n'`; only `install ok installed` counts.
pub fn missing_apt(wanted: &[String], dpkg_status: &str) -> Vec<String> {
    let installed: Vec<&str> = dpkg_status
        .lines()
        .filter(|l| l.contains("install ok installed"))
        .filter_map(|l| l.split_whitespace().next())
        .map(|p| p.split(':').next().unwrap_or(p))
        .collect();
    wanted
        .iter()
        .filter(|w| !installed.contains(&w.as_str()))
        .cloned()
        .collect()
}

/// Pure over `brew list --versions`.
pub fn missing_brew(wanted: &[String], brew_list: &str) -> Vec<String> {
    let installed: Vec<&str> = brew_list
        .lines()
        .filter_map(|l| l.split_whitespace().next())
        .collect();
    wanted
        .iter()
        .filter(|w| !installed.contains(&w.as_str()))
        .cloned()
        .collect()
}

/// Only keys the kernel currently holds below target; a bigger value is somebody else's tuning.
pub fn sysctl_updates(
    current: &BTreeMap<String, u64>,
    target: &BTreeMap<String, u64>,
) -> Vec<(String, u64)> {
    target
        .iter()
        .filter(|(key, want)| current.get(*key).is_none_or(|have| have < *want))
        .map(|(key, want)| (key.clone(), *want))
        .collect()
}

pub fn pip_spec(extras: &[String]) -> String {
    format!("dimos[{}]=={DIMOS_VERSION}", extras.join(","))
}

pub fn sync_args(extras: &[String]) -> Vec<String> {
    extras
        .iter()
        .flat_map(|e| ["--extra".to_string(), e.clone()])
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    include!("../build_support.rs"); // the same pyproject parser build.rs used, so the test is honest

    const PYPROJECT: &str = include_str!("../../pyproject.toml");

    fn cfg() -> Platforms {
        Platforms::load()
    }

    #[test]
    fn extras_const_matches_pyproject() {
        assert_eq!(EXTRAS, extras(PYPROJECT).as_slice());
        assert_eq!(DIMOS_VERSION, version(PYPROJECT));
    }

    #[test]
    fn platforms_toml_parses_and_every_extras_key_is_a_pyproject_extra() {
        let cfg = cfg();
        for name in cfg.extras.keys() {
            assert!(
                EXTRAS.contains(&name.as_str()),
                "{name} is not a pyproject extra"
            );
        }
        assert!(cfg.linux.memlock_bytes > 0 && !cfg.linux.sysctl.is_empty());
    }

    #[test]
    fn no_extras_means_base_and_duplicates_collapse() {
        let cfg = cfg();
        assert_eq!(validate_extras(&[], Arch::Aarch64, &cfg).unwrap(), ["base"]);
        let asked = [
            "unitree".to_string(),
            " unitree ".to_string(),
            "".to_string(),
        ];
        assert_eq!(
            validate_extras(&asked, Arch::Aarch64, &cfg).unwrap(),
            ["unitree"]
        );
    }

    #[test]
    fn unknown_extra_error_lists_all_extras() {
        let err = format!(
            "{:#}",
            validate_extras(&["unitre".to_string()], Arch::X86_64, &cfg()).unwrap_err()
        );
        assert!(err.contains("unknown extra 'unitre'"), "{err}");
        assert!(err.contains("unitree"), "{err}");
    }

    #[test]
    fn cuda_refused_on_aarch64_with_pyproject_reason_and_allowed_on_x86_64() {
        let cfg = cfg();
        let cuda = ["cuda".to_string()];
        assert_eq!(
            validate_extras(&cuda, Arch::X86_64, &cfg).unwrap(),
            ["cuda"]
        );
        let err = format!(
            "{:#}",
            validate_extras(&cuda, Arch::Aarch64, &cfg).unwrap_err()
        );
        assert!(err.contains("platform_machine"), "{err}");
        assert!(err.contains("x86_64-only"), "{err}");
    }

    #[test]
    fn missing_apt_from_dpkg_fixture() {
        let status = "git install ok installed\n\
                      curl deinstall ok config-files\n\
                      libturbojpeg:arm64 install ok installed\n";
        let wanted = ["git", "curl", "libturbojpeg", "cmake"].map(String::from);
        assert_eq!(missing_apt(&wanted, status), ["curl", "cmake"]);
    }

    #[test]
    fn missing_brew_from_versions_fixture() {
        let list = "git-lfs 3.5.1\ncmake 3.30.2\n";
        let wanted = ["git-lfs", "portaudio"].map(String::from);
        assert_eq!(missing_brew(&wanted, list), ["portaudio"]);
    }

    #[test]
    fn unitree_adds_cmake_once() {
        let cfg = cfg();
        let extras = ["base".to_string(), "unitree".to_string()];
        let apt = system_packages(&extras, PkgManager::Apt, &cfg);
        assert_eq!(apt.iter().filter(|p| *p == "cmake").count(), 1);
        assert!(apt.contains(&"git-lfs".to_string()));
        assert!(system_packages(&extras, PkgManager::None, &cfg).is_empty());
    }

    #[test]
    fn sysctl_updates_only_below_target() {
        let target = cfg().linux.sysctl;
        let at_target: BTreeMap<String, u64> =
            target.iter().map(|(k, v)| (k.clone(), *v)).collect();
        assert!(sysctl_updates(&at_target, &target).is_empty());
        let stock: BTreeMap<String, u64> = target.keys().map(|k| (k.clone(), 212_992)).collect();
        assert_eq!(sysctl_updates(&stock, &target).len(), target.len());
        assert!(sysctl_updates(&BTreeMap::new(), &target).len() == target.len());
    }

    #[test]
    fn pip_spec_pins_dimos_version() {
        let spec = pip_spec(&["base".to_string(), "unitree".to_string()]);
        assert_eq!(spec, format!("dimos[base,unitree]=={DIMOS_VERSION}"));
        assert_eq!(
            sync_args(&["base".to_string(), "unitree".to_string()]),
            ["--extra", "base", "--extra", "unitree"]
        );
    }
}
