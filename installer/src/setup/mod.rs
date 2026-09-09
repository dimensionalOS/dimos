// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use anyhow::{bail, Context, Result};
use serde::{Deserialize, Serialize};
use std::fs;
use std::io::IsTerminal;
use std::path::Path;
use std::process::Command;

pub mod environment;
mod platform;
mod tools;

use crate::cli::{InstallMode, Profile, SetupArgs};
use environment::{checked, checked_timeout};
use std::time::Duration;

const PIXI_MANIFEST: &str = include_str!("../../resources/pixi.toml");
const PIXI_LOCK: &str = include_str!("../../resources/pixi.lock");
const UV_POLICY: &str = include_str!("../../resources/uv-policy.toml");
const DIMOS_VERSION: &str = "0.0.14b1";

#[derive(Debug, Serialize, Deserialize)]
struct Project {
    mode: InstallMode,
    profile: Profile,
}

fn choices(args: &SetupArgs, unattended: bool) -> Result<Project> {
    let profile = match args.profile {
        Some(profile) => profile,
        None if unattended => {
            bail!("Unattended setup requires --profile navigation or manipulation")
        }
        None => cliclack::select("Which use case are you installing?")
            .item(Profile::Navigation, "Navigation", "unitree-go2")
            .item(
                Profile::Manipulation,
                "Manipulation",
                "xarm7-planner-coordinator",
            )
            .interact()?,
    };
    let mode = match args.mode {
        Some(mode) => mode,
        None if unattended => bail!("Unattended setup requires --mode library or dev"),
        None => cliclack::select("How will you use DimOS?")
            .item(InstallMode::Library, "Library", "dedicated managed project")
            .item(InstallMode::Dev, "Contributor", "editable DimOS checkout")
            .interact()?,
    };
    if mode == InstallMode::Dev && args.wheel.is_some() {
        bail!("--wheel is only valid in library mode");
    }
    Ok(Project { mode, profile })
}

fn library_manifest(profile: Profile, wheel: Option<&Path>) -> Result<String> {
    let mut manifest: toml::Value = toml::from_str(UV_POLICY)?;
    let requirement = format!("dimos[{}]=={DIMOS_VERSION}", profile.extras().join(","));
    manifest
        .as_table_mut()
        .context("uv policy must be a table")?
        .insert(
            "project".into(),
            toml::Value::try_from(serde_json::json!({
                "name": "dimos-project", "version": "0.1.0", "requires-python": ">=3.12,<3.13",
                "dependencies": [requirement],
            }))?,
        );
    if let Some(wheel) = wheel {
        manifest["tool"]["uv"]["sources"]
            .as_table_mut()
            .context("uv sources must be a table")?
            .insert(
                "dimos".into(),
                toml::Value::try_from(serde_json::json!({"path": wheel.to_string_lossy()}))?,
            );
    }
    Ok(toml::to_string_pretty(&manifest)?)
}

fn validate_destination(dir: &Path, project: &Project) -> Result<()> {
    let marker = dir.join(".dimos/project.toml");
    if marker.exists() {
        let old: Project = toml::from_str(&fs::read_to_string(marker)?)?;
        if old.mode != project.mode || old.profile != project.profile {
            bail!("This project has a different mode or profile; choose a new directory");
        }
        return Ok(());
    }
    if project.mode == InstallMode::Library {
        if dir.exists() && fs::read_dir(dir)?.next().is_some() {
            bail!("Library mode requires an empty directory; existing projects are not modified");
        }
    } else if dir.exists() && fs::read_dir(dir)?.next().is_some() {
        let manifest: toml::Value = toml::from_str(
            &fs::read_to_string(dir.join("pyproject.toml"))
                .context("Contributor mode requires a DimOS checkout")?,
        )?;
        if manifest
            .get("project")
            .and_then(|p| p.get("name"))
            .and_then(|n| n.as_str())
            != Some("dimos")
            || !dir.join("dimos/core").is_dir()
        {
            bail!("Contributor mode requires a DimOS checkout");
        }
    }
    Ok(())
}

fn tool_activation(uv: &Path, nix: &Path) -> String {
    format!("export PATH=\"$VIRTUAL_ENV/bin\":{}:{}:\"$PATH\"\nexport NIX_CONFIG=\"${{NIX_CONFIG:-}}\nextra-experimental-features = nix-command flakes\"", environment::esc(&uv.parent().unwrap().to_string_lossy()), environment::esc(&nix.parent().unwrap().to_string_lossy()))
}

// @flow subgraph setup_pipeline["Profile installation"]
// @flow cmd_setup --> choose_profile
// @flow choose_profile["Require profile and mode"] :stage
// @flow choose_profile --> project_boundary
// @flow project_boundary["Validate project ownership"] :process
// @flow project_boundary --> preview
// @flow preview{"Dry run?"} :decision
// @flow preview -->|"yes"| preview_done
// @flow preview_done["Print plan without changes"] :success
// @flow preview -->|"no"| target_platform
// @flow target_platform["Identify Ubuntu or JetPack baseline"] :process
// @flow target_platform --> bootstrap_tools
// @flow bootstrap_tools["Bootstrap uv, Pixi, Nix; check Nix store"] :stage
// @flow bootstrap_tools --> prepare_project
// @flow prepare_project["Create managed project or use contributor checkout"] :process
// @flow prepare_project --> pixi_install
// @flow pixi_install["Install locked non-Python dependencies"] :stage
// @flow pixi_install --> uv_install
// @flow uv_install["uv Python 3.12 and selected profile"] :stage
// @flow uv_install --> dependency_checks
// @flow dependency_checks["Native libraries, reference imports, CLI"] :stage
// @flow dependency_checks --> verified_setup
// @flow verified_setup["Record dependency success; show activation"] :success
// @flow bootstrap_tools -->|"failure"| setup_failed
// @flow pixi_install -->|"failure"| setup_failed
// @flow uv_install -->|"failure"| setup_failed
// @flow dependency_checks -->|"failure or timeout"| setup_failed
// @flow setup_failed["Exit nonzero; no fallback"] :error
pub fn run_setup(
    args: &SetupArgs,
    dry_run: bool,
    _verbose: bool,
    non_interactive: bool,
) -> Result<()> {
    let project = choices(args, non_interactive || !std::io::stdin().is_terminal())?;
    let dir = std::path::absolute(&args.project_dir)?;
    validate_destination(&dir, &project)?;
    let wheel = args.wheel.as_ref().map(fs::canonicalize).transpose()?;
    println!(
        "{} / {:?}: {}",
        project.profile.name(),
        project.mode,
        dir.display()
    );
    if dry_run {
        println!("Install locked Pixi dependencies; uv sync selected extras; verify {} dependencies.\nActivate: source {}/.dimos/activate.sh", project.profile.blueprint(), dir.display());
        return Ok(());
    }
    let verified = dir.join(".dimos/verified.json");
    if verified.exists() {
        fs::remove_file(&verified)?;
    }
    let platform = platform::detect()?;
    if platform.starts_with("unverified") {
        eprintln!("This Linux distribution is outside the verified platform matrix.");
    }
    let bootstrap = tempfile::tempdir()?;
    let tools::Tools { pixi, uv, nix } = tools::prepare(bootstrap.path(), non_interactive)?;
    if project.mode == InstallMode::Dev && !dir.join("pyproject.toml").exists() {
        fs::write(bootstrap.path().join("pixi.toml"), PIXI_MANIFEST)?;
        fs::write(bootstrap.path().join("pixi.lock"), PIXI_LOCK)?;
        checked(
            Command::new(&pixi)
                .args(["run", "--locked", "--manifest-path"])
                .arg(bootstrap.path().join("pixi.toml"))
                .args([
                    "git",
                    "clone",
                    "--branch",
                    &args.branch,
                    "--",
                    "https://github.com/dimensionalOS/dimos.git",
                ])
                .arg(&dir)
                .env("GIT_LFS_SKIP_SMUDGE", "1"),
        )?;
    }
    fs::create_dir_all(dir.join(".dimos"))?;
    fs::write(dir.join(".dimos/project.toml"), toml::to_string(&project)?)?;
    if project.mode == InstallMode::Library {
        let path = dir.join("pyproject.toml");
        let manifest = library_manifest(project.profile, wheel.as_deref())?;
        if path.exists() && fs::read_to_string(&path)? != manifest {
            bail!("Managed project manifest changed; refusing to overwrite it");
        }
        fs::write(path, manifest)?;
    }
    fs::write(dir.join(".dimos/pixi.toml"), PIXI_MANIFEST)?;
    fs::write(dir.join(".dimos/pixi.lock"), PIXI_LOCK)?;
    checked(
        Command::new(&pixi)
            .args(["install", "--locked", "--manifest-path"])
            .arg(dir.join(".dimos/pixi.toml")),
    )?;
    checked(Command::new(&uv).args(["python", "install", "3.12"]))?;
    checked(
        Command::new(&uv)
            .args([
                "venv",
                "--python",
                "3.12",
                "--managed-python",
                "--allow-existing",
            ])
            .arg(dir.join(".venv")),
    )?;
    let dir = dir.canonicalize()?;
    fs::write(
        dir.join(".dimos/activate.sh"),
        format!(
            "{}\n{}",
            environment::activation(&dir, &pixi),
            tool_activation(&uv, &nix)
        ),
    )?;
    let mut sync = vec![
        uv.to_string_lossy().into_owned(),
        "sync".into(),
        "--python".into(),
        "3.12".into(),
        "--managed-python".into(),
        "--no-default-groups".into(),
    ];
    if project.mode == InstallMode::Dev {
        sync.push("--locked".into());
        for extra in project.profile.extras() {
            sync.extend(["--extra".into(), (*extra).into()]);
        }
    } else if dir.join("uv.lock").exists() {
        sync.push("--locked".into());
    }
    checked(&mut environment::command(&dir, &sync))?;
    fs::write(
        dir.join(".dimos/verify.py"),
        include_str!("../../resources/verify.py"),
    )?;
    checked_timeout(
        &mut environment::command(
            &dir,
            &[
                "python".into(),
                dir.join(".dimos/verify.py").to_string_lossy().into_owned(),
                project.profile.name().into(),
                dir.to_string_lossy().into_owned(),
            ],
        ),
        Duration::from_secs(120),
    )?;
    checked_timeout(
        environment::command(&dir, &["dimos".into(), "--help".into()])
            .stdout(std::process::Stdio::null()),
        Duration::from_secs(30),
    )?;
    fs::write(
        verified,
        serde_json::to_string_pretty(
            &serde_json::json!({"profile": project.profile.name(), "reference": project.profile.blueprint(), "platform": platform, "dependency_checks": "passed", "workflow_verification": "not-run"}),
        )?,
    )?;
    println!(
        "Dependency checks passed. Activate with: source {}",
        dir.join(".dimos/activate.sh").display()
    );
    Ok(())
}
// @flow end

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;

    #[test]
    fn unattended_requires_profile() {
        let args =
            SetupArgs::parse_from(["setup", "--project-dir", "/tmp/new", "--mode", "library"]);
        assert!(choices(&args, true)
            .unwrap_err()
            .to_string()
            .contains("--profile"));
    }

    #[test]
    fn library_keeps_uv_overrides_and_selected_extras() {
        let manifest: toml::Value =
            toml::from_str(&library_manifest(Profile::Manipulation, None).unwrap()).unwrap();
        assert!(manifest["project"]["dependencies"][0]
            .as_str()
            .unwrap()
            .contains("manipulation,cpu"));
        assert!(manifest["tool"]["uv"]["override-dependencies"]
            .as_array()
            .unwrap()
            .iter()
            .any(|v| v.as_str().unwrap().contains("opencv-python")));
        assert!(manifest["tool"]["uv"].get("default-groups").is_none());
    }

    #[test]
    fn shell_quotes_project_paths() {
        let script =
            environment::activation(Path::new("/tmp/it's a project"), Path::new("/bin/pixi"));
        assert!(script.contains("'/tmp/it'\\''s a project'"));
        assert!(script.find("shell-hook").unwrap() < script.find(".venv/bin/activate").unwrap());
    }

    #[test]
    fn refuses_an_unrelated_library_project_without_modifying_it() {
        let dir = tempfile::tempdir().unwrap();
        let manifest = dir.path().join("pyproject.toml");
        fs::write(&manifest, "user project").unwrap();
        let project = Project {
            mode: InstallMode::Library,
            profile: Profile::Navigation,
        };
        assert!(validate_destination(dir.path(), &project).is_err());
        assert_eq!(fs::read_to_string(manifest).unwrap(), "user project");
        assert!(!dir.path().join(".dimos").exists());
    }

    #[test]
    fn dry_run_creates_nothing() {
        let parent = tempfile::tempdir().unwrap();
        let dir = parent.path().join("new project");
        let args = SetupArgs {
            mode: Some(InstallMode::Library),
            profile: Some(Profile::Navigation),
            project_dir: dir.clone(),
            branch: "main".into(),
            wheel: None,
        };
        run_setup(&args, true, false, true).unwrap();
        assert!(!dir.exists());
    }

    #[test]
    fn command_preserves_arguments_and_failure() {
        let parent = tempfile::tempdir().unwrap();
        let dir = parent.path().join("project with ' quotes");
        fs::create_dir_all(dir.join(".dimos")).unwrap();
        fs::write(
            dir.join(".dimos/activate.sh"),
            "export DIM_TEST=activated\n",
        )
        .unwrap();
        let output = environment::command(
            &dir,
            &[
                "bash".into(),
                "-c".into(),
                "printf '%s|%s' \"$DIM_TEST\" \"$1\"".into(),
                "test".into(),
                "$(touch unwanted)".into(),
            ],
        )
        .output()
        .unwrap();
        assert!(output.status.success());
        assert_eq!(
            String::from_utf8(output.stdout).unwrap(),
            "activated|$(touch unwanted)"
        );
        assert!(!dir.join("unwanted").exists());
        assert!(checked(&mut environment::command(&dir, &["false".into()])).is_err());
        fs::write(dir.join(".dimos/activate.sh"), "return 7\n").unwrap();
        assert!(checked(&mut environment::command(&dir, &["true".into()])).is_err());
    }

    #[test]
    fn wheel_path_is_toml_data() {
        let manifest: toml::Value = toml::from_str(
            &library_manifest(Profile::Navigation, Some(Path::new("/tmp/a ' wheel.whl"))).unwrap(),
        )
        .unwrap();
        assert_eq!(
            manifest["tool"]["uv"]["sources"]["dimos"]["path"].as_str(),
            Some("/tmp/a ' wheel.whl")
        );
    }

    #[test]
    fn bundled_policy_matches_repository() {
        let root: toml::Value = toml::from_str(
            &fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("../pyproject.toml"))
                .unwrap(),
        )
        .unwrap();
        let bundled: toml::Value = toml::from_str(UV_POLICY).unwrap();
        let mut expected = root["tool"]["uv"].clone();
        expected.as_table_mut().unwrap().remove("default-groups");
        assert_eq!(
            bundled["tool"]["uv"], expected,
            "Refresh resources/uv-policy.toml when dependency policy changes"
        );
        assert_eq!(root["project"]["version"].as_str(), Some(DIMOS_VERSION));
        assert!(!PIXI_LOCK.lines().any(|line| matches!(
            line.trim(),
            "name: python" | "name: pypy" | "name: python-freethreading"
        )));
    }

    #[test]
    fn dependency_check_timeout_is_fatal() {
        let mut command = Command::new("sleep");
        command.arg("30");
        let error = checked_timeout(&mut command, Duration::from_millis(50)).unwrap_err();
        assert!(error.to_string().contains("timed out"));
    }
}
