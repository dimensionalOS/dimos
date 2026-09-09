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

use crate::cli::{InstallMode, Profile, SetupArgs};
use anyhow::{bail, Context, Result};
use environment::{checked, checked_timeout};
use serde::{Deserialize, Serialize};
use std::{
    fs,
    io::IsTerminal,
    path::{Path, PathBuf},
    process::Command,
    time::Duration,
};
pub mod environment;
mod platform;
mod tools;
mod workspace;

const PIXI_MANIFEST: &str = include_str!("../../resources/pixi.toml");
const PIXI_LOCK: &str = include_str!("../../resources/pixi.lock");
const UV_POLICY: &str = include_str!("../../resources/uv-policy.toml");
const SDK_VERSION: &str = match option_env!("DIMOS_SDK_VERSION") {
    Some(v) => v,
    None => "0.0.14b1",
};

#[derive(Debug, Serialize, Deserialize)]
struct Project {
    mode: InstallMode,
    profile: Profile,
    sdk_version: String,
    package: String,
}

fn destination(args: &SetupArgs, unattended: bool) -> Result<PathBuf> {
    let path = match &args.project_dir {
        Some(path) => path.clone(),
        None if unattended => bail!("Provide a workspace directory"),
        None => PathBuf::from(cliclack::input("Project directory").interact::<String>()?),
    };
    let absolute = std::path::absolute(path)?;
    if absolute.exists() {
        Ok(absolute.canonicalize()?)
    } else {
        Ok(absolute)
    }
}

fn project(args: &SetupArgs, dir: &Path, unattended: bool) -> Result<Project> {
    if args.restore {
        let project: Project = toml::from_str(
            &fs::read_to_string(dir.join(".dimos/project.toml"))
                .context("Restore requires an existing DimOS workspace")?,
        )?;
        if project.sdk_version != SDK_VERSION {
            bail!(
                "Restore this workspace with creator version {}",
                project.sdk_version
            );
        }
        if args.profile.is_some_and(|p| p != project.profile) || args.wheel.is_some() {
            bail!(
                "Restore uses the workspace profile and dependency manifest; do not override them"
            );
        }
        for file in [
            "pyproject.toml",
            "uv.lock",
            ".dimos/pixi.toml",
            ".dimos/pixi.lock",
            ".dimos/activate.sh",
            ".dimos/environment.py",
        ] {
            if !dir.join(file).is_file() {
                bail!("Restore requires {file}");
            }
        }
        return Ok(project);
    }
    let profile = match args.profile {
        Some(profile) => profile,
        None if unattended => {
            bail!("Unattended creation requires --profile navigation or manipulation")
        }
        None => cliclack::select("Which profile?")
            .item(Profile::Navigation, "Navigation", "unitree-go2")
            .item(
                Profile::Manipulation,
                "Manipulation",
                "xarm7-planner-coordinator",
            )
            .interact()?,
    };
    let mode = if args.contributor {
        InstallMode::Contributor
    } else {
        InstallMode::Sdk
    };
    if dir.exists() && fs::read_dir(dir)?.next().is_some() {
        if mode == InstallMode::Sdk {
            bail!("Creation requires an empty directory; use --restore for an existing workspace");
        }
        let manifest: toml::Value = toml::from_str(
            &fs::read_to_string(dir.join("pyproject.toml"))
                .context("Contributor mode requires a DimOS checkout")?,
        )?;
        if manifest["project"]["name"].as_str() != Some("dimos") || !dir.join("dimos/core").is_dir()
        {
            bail!("Contributor mode requires a DimOS checkout");
        }
        if dir.join(".dimos/project.toml").exists() {
            bail!("Workspace already configured; use --restore");
        }
    }
    Ok(Project {
        mode,
        profile,
        sdk_version: SDK_VERSION.into(),
        package: if args.contributor {
            "dimos".into()
        } else {
            workspace::package_name(dir)?
        },
    })
}

// @flow subgraph setup_pipeline["Workspace creation or restoration"]
// @flow create["Create or restore workspace"] :stage
// @flow create --> validate["Validate destination and profile"]
// @flow validate --> bootstrap["Prepare uv, Pixi and Nix"]
// @flow bootstrap --> scaffold["Create editable SDK package, or preserve existing project"]
// @flow scaffold --> install["Install locked native dependencies and Python project"]
// @flow install --> verify["Verify dependencies and project registration"]
// @flow verify --> ready["Print activation and development commands"]
pub fn run_setup(args: &SetupArgs) -> Result<()> {
    let unattended = args.non_interactive || args.dry_run || !std::io::stdin().is_terminal();
    let dir = destination(args, unattended)?;
    let project = project(args, &dir, unattended)?;
    let wheel = args.wheel.as_ref().map(fs::canonicalize).transpose()?;
    println!(
        "{} / {:?}: {}",
        project.profile.name(),
        project.mode,
        dir.display()
    );
    if args.dry_run {
        println!("Prepare uv/Pixi/Nix, install dependencies, verify, print activation.");
        return Ok(());
    }
    let verified = dir.join(".dimos/verified.json");
    if verified.exists() {
        fs::remove_file(&verified)?;
    }
    let platform = platform::detect()?;
    if platform.starts_with("unverified") {
        eprintln!("This distribution is outside the verified platform matrix.");
    }
    let bootstrap = tempfile::tempdir()?;
    let tools::Tools { pixi, uv, nix } = tools::prepare(bootstrap.path(), unattended)?;
    if project.mode == InstallMode::Contributor && !dir.join("pyproject.toml").exists() {
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
    if !args.restore && project.mode == InstallMode::Sdk {
        workspace::create(&dir, project.profile, wheel.as_deref())?;
    }
    workspace::write_environment(&dir, !args.restore)?;
    if !args.restore {
        fs::write(dir.join(".dimos/project.toml"), toml::to_string(&project)?)?;
    }
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
    fs::write(
        dir.join(".dimos/tools.json"),
        serde_json::to_string(&serde_json::json!({"pixi":pixi,"uv":uv,"nix":nix}))?,
    )?;
    let dir = dir.canonicalize()?;
    let mut sync = vec![
        uv.to_string_lossy().into_owned(),
        "sync".into(),
        "--python".into(),
        "3.12".into(),
        "--managed-python".into(),
    ];
    if args.restore || project.mode == InstallMode::Contributor {
        sync.push("--locked".into());
    }
    if project.mode == InstallMode::Contributor {
        sync.push("--no-default-groups".into());
        for extra in project.profile.extras() {
            sync.extend(["--extra".into(), (*extra).into()]);
        }
    }
    checked(&mut environment::command(&dir, &sync))?;
    checked_timeout(
        &mut environment::command(
            &dir,
            &[
                "dimos".into(),
                "doctor".into(),
                "--project-dir".into(),
                dir.to_string_lossy().into_owned(),
            ],
        ),
        Duration::from_secs(600),
    )?;
    checked_timeout(
        environment::command(&dir, &["dimos".into(), "--help".into()])
            .stdout(std::process::Stdio::null()),
        Duration::from_secs(30),
    )?;
    fs::write(
        verified,
        serde_json::to_string_pretty(
            &serde_json::json!({"profile":project.profile.name(),"reference":project.profile.blueprint(),"platform":platform,"dependency_checks":"passed","workflow_verification":"not-run"}),
        )?,
    )?;
    println!(
        "Ready.\n  cd {}\n  source .dimos/activate.sh\n  dimos doctor",
        environment::esc(&dir.to_string_lossy())
    );
    if project.mode == InstallMode::Sdk {
        println!("  dimos run {}.hello\n  pytest", project.package);
    }
    println!("Optional direnv: review .envrc, install direnv and its Bash/Zsh hook if needed, then run direnv allow.\nHook instructions: https://direnv.net/docs/hook.html");
    Ok(())
}
// @flow end

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;

    #[test]
    fn creation_requires_profile_without_mutating_destination() {
        let parent = tempfile::tempdir().unwrap();
        let target = parent.path().join("my-robot");
        let args = SetupArgs::parse_from([
            "create-dimos",
            target.to_str().unwrap(),
            "--non-interactive",
        ]);
        assert!(run_setup(&args)
            .unwrap_err()
            .to_string()
            .contains("--profile"));
        assert!(!target.exists());
    }

    #[test]
    fn creation_refuses_existing_files() {
        let dir = tempfile::tempdir().unwrap();
        fs::write(dir.path().join("pyproject.toml"), "user content").unwrap();
        let args = SetupArgs::parse_from([
            "create-dimos",
            dir.path().to_str().unwrap(),
            "--profile",
            "navigation",
        ]);
        assert!(run_setup(&args)
            .unwrap_err()
            .to_string()
            .contains("empty directory"));
        assert_eq!(
            fs::read_to_string(dir.path().join("pyproject.toml")).unwrap(),
            "user content"
        );
    }

    #[test]
    fn scaffold_is_an_editable_package_and_restore_preserves_edits() {
        let parent = tempfile::tempdir().unwrap();
        let dir = parent.path().join("my-robot");
        workspace::create(&dir, Profile::Navigation, None).unwrap();
        workspace::write_environment(&dir, true).unwrap();
        fs::write(dir.join("uv.lock"), "version = 1").unwrap();
        fs::write(
            dir.join(".dimos/project.toml"),
            toml::to_string(&Project {
                mode: InstallMode::Sdk,
                profile: Profile::Navigation,
                sdk_version: SDK_VERSION.into(),
                package: "my-robot".into(),
            })
            .unwrap(),
        )
        .unwrap();
        let manifest = dir.join("pyproject.toml");
        let original = fs::read_to_string(&manifest).unwrap();
        let value: toml::Value = toml::from_str(&original).unwrap();
        assert_eq!(
            value["project"]["entry-points"]["dimos.blueprints"]["hello"].as_str(),
            Some("my_robot.hello:Hello")
        );
        assert!(value.get("build-system").is_some());
        let edited = format!("{original}\n# A developer's change\n");
        fs::write(&manifest, &edited).unwrap();
        fs::write(dir.join("src/my_robot/hello.py"), "user source").unwrap();
        let args = SetupArgs::parse_from([
            "create-dimos",
            "--restore",
            dir.to_str().unwrap(),
            "--dry-run",
        ]);
        run_setup(&args).unwrap();
        assert_eq!(fs::read_to_string(manifest).unwrap(), edited);
        assert_eq!(
            fs::read_to_string(dir.join("src/my_robot/hello.py")).unwrap(),
            "user source"
        );
    }

    #[test]
    fn dry_run_creates_nothing_and_wheel_path_is_data() {
        let parent = tempfile::tempdir().unwrap();
        let dir = parent.path().join("my project");
        let args = SetupArgs::parse_from([
            "create-dimos",
            dir.to_str().unwrap(),
            "--profile",
            "manipulation",
            "--dry-run",
        ]);
        run_setup(&args).unwrap();
        assert!(!dir.exists());
        let manifest: toml::Value = toml::from_str(
            &workspace::manifest(
                "my-project",
                Profile::Manipulation,
                Some(Path::new("/tmp/a ' wheel.whl")),
            )
            .unwrap(),
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
        assert_eq!(bundled["tool"]["uv"], expected);
        assert_eq!(root["project"]["version"].as_str(), Some(SDK_VERSION));
        assert!(!PIXI_LOCK.lines().any(|line| matches!(
            line.trim(),
            "name: python" | "name: pypy" | "name: python-freethreading"
        )));
    }

    #[test]
    fn command_preserves_arguments_and_exit_status() {
        let dir = tempfile::tempdir().unwrap();
        fs::create_dir(dir.path().join(".dimos")).unwrap();
        fs::write(
            dir.path().join(".dimos/activate.sh"),
            "export DIM_TEST=activated\n",
        )
        .unwrap();
        let output = environment::command(
            dir.path(),
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
        assert!(!dir.path().join("unwanted").exists());
        fs::write(dir.path().join(".dimos/activate.sh"), "return 7\n").unwrap();
        assert!(checked(&mut environment::command(dir.path(), &["true".into()])).is_err());
    }

    #[test]
    fn dependency_timeout_is_fatal() {
        assert!(
            checked_timeout(Command::new("sleep").arg("30"), Duration::from_millis(30))
                .unwrap_err()
                .to_string()
                .contains("timed out")
        );
    }
}
