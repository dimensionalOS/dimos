// Shared by build.rs (include!) and tests/build_support.rs so the parsers have one definition.

/// Sorted names of `[project.optional-dependencies]`; `[dependency-groups]` are not extras.
pub fn extras(pyproject: &str) -> Vec<String> {
    let doc: toml::Value = toml::from_str(pyproject).expect("pyproject.toml is not valid TOML");
    let table = doc
        .get("project")
        .and_then(|p| p.get("optional-dependencies"))
        .and_then(|t| t.as_table())
        .expect("pyproject.toml has no [project.optional-dependencies] table");
    let mut names: Vec<String> = table.keys().cloned().collect();
    names.sort();
    names
}

/// `[project].version` verbatim (PEP 440, e.g. `0.0.14b1`).
pub fn version(pyproject: &str) -> String {
    let doc: toml::Value = toml::from_str(pyproject).expect("pyproject.toml is not valid TOML");
    doc.get("project")
        .and_then(|p| p.get("version"))
        .and_then(|v| v.as_str())
        .map(str::to_owned)
        .expect("pyproject.toml has no [project].version")
}
