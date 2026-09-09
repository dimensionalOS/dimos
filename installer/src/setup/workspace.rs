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

use super::{PIXI_LOCK, PIXI_MANIFEST, SDK_VERSION, UV_POLICY};
use crate::cli::Profile;
use anyhow::{bail, Result};
use std::{fs, path::Path};

pub fn package_name(dir: &Path) -> Result<String> {
    let raw = dir.file_name().and_then(|s| s.to_str()).unwrap_or("");
    let name = raw
        .to_ascii_lowercase()
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|s| !s.is_empty())
        .collect::<Vec<_>>()
        .join("-");
    if name.is_empty() || !name.as_bytes()[0].is_ascii_alphabetic() || name == "dimos" {
        bail!("Choose a project directory name starting with a letter, other than dimos");
    }
    Ok(name)
}

pub fn manifest(name: &str, profile: Profile, wheel: Option<&Path>) -> Result<String> {
    let module = name.replace('-', "_");
    let mut value: toml::Value = toml::from_str(UV_POLICY)?;
    let fields: toml::Value = toml::from_str(&format!(
        r#"
[project]
name = "{name}"
version = "0.1.0"
requires-python = ">=3.12,<3.13"
dependencies = ["dimos[{}]=={SDK_VERSION}"]

[project.entry-points."dimos.blueprints"]
hello = "{module}.hello:Hello"

[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[tool.setuptools.packages.find]
where = ["src"]

[tool.pytest.ini_options]
testpaths = ["tests"]

[dependency-groups]
dev = ["pytest>=8"]
"#,
        profile.extras().join(",")
    ))?;
    for (key, field) in fields.as_table().unwrap() {
        if key == "tool" {
            value["tool"]
                .as_table_mut()
                .unwrap()
                .extend(field.as_table().unwrap().clone());
        } else {
            value
                .as_table_mut()
                .unwrap()
                .insert(key.clone(), field.clone());
        }
    }
    if let Some(wheel) = wheel {
        value["tool"]["uv"]["sources"]
            .as_table_mut()
            .unwrap()
            .insert(
                "dimos".into(),
                toml::Value::try_from(serde_json::json!({"path":wheel.to_string_lossy()}))?,
            );
    }
    Ok(toml::to_string_pretty(&value)?)
}

pub fn create(dir: &Path, profile: Profile, wheel: Option<&Path>) -> Result<String> {
    let name = package_name(dir)?;
    let module = name.replace('-', "_");
    let source = dir.join("src").join(&module);
    fs::create_dir_all(&source)?;
    fs::create_dir_all(dir.join("tests"))?;
    fs::write(dir.join("pyproject.toml"), manifest(&name, profile, wheel)?)?;
    fs::write(source.join("__init__.py"), "")?;
    fs::write(
        source.join("hello.py"),
        include_str!("../../resources/hello.py"),
    )?;
    fs::write(dir.join("tests/test_hello.py"), format!("from {module}.hello import greeting\n\n\ndef test_greeting():\n    assert greeting(\"robot\") == \"Hello, robot!\"\n"))?;
    fs::write(dir.join(".gitignore"), ".venv/\n.dimos/.pixi/\n.dimos/tools.json\n.dimos/verified.json\n.direnv/\n.env\n__pycache__/\n*.egg-info/\n.pytest_cache/\nbuild/\ndist/\n")?;
    fs::write(
        dir.join(".envrc"),
        include_str!("../../resources/envrc.example"),
    )?;
    fs::write(
        dir.join("README.md"),
        format!(
            r#"# {name}

An editable DimOS SDK project. Your code and `pyproject.toml` belong to you.

```bash
source .dimos/activate.sh
dimos run {name}.hello
pytest
uv add <dependency>
dimos doctor
```

Edit `src/{module}/hello.py` and rerun the blueprint. Stop it with Ctrl-C.
Register additional blueprints in `[project.entry-points."dimos.blueprints"]`.
After changing entry points, run `uv sync` to refresh the editable installation.

For automatic activation, install direnv, configure its Bash/Zsh hook, and run
`direnv allow` after reviewing `.envrc`. Leaving the directory restores the prior
environment. Manual activation can be undone with `deactivate`.

Commit source, manifests, lockfiles, `.envrc`, and `.dimos` configuration. Installed
environments and machine-specific tool paths are ignored. After cloning, run the
same versioned workspace initializer with `--restore .` before activation. Restore
uses your lockfiles and never regenerates your source or dependency manifest.
"#
        ),
    )?;
    Ok(name)
}

pub fn write_environment(dir: &Path, fresh: bool) -> Result<()> {
    fs::create_dir_all(dir.join(".dimos"))?;
    if fresh {
        fs::write(dir.join(".dimos/pixi.toml"), PIXI_MANIFEST)?;
        fs::write(dir.join(".dimos/pixi.lock"), PIXI_LOCK)?;
        fs::write(
            dir.join(".dimos/activate.sh"),
            include_str!("../../resources/activate.sh"),
        )?;
        fs::write(
            dir.join(".dimos/environment.py"),
            include_str!("../../resources/environment.py"),
        )?;
    }
    Ok(())
}
