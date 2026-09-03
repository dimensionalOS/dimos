include!("../pyproject.rs");

const FIXTURE: &str = r#"
[project]
name = "dimos"
version = "0.0.14b1"

[project.optional-dependencies]
unitree = ["dimos[base]"]
base = ["numpy"]
sim = ["mujoco"]

[dependency-groups]
tests = ["pytest"]
"#;

#[test]
fn extras_reads_every_optional_dependency_name_sorted() {
    assert_eq!(extras(FIXTURE), ["base", "sim", "unitree"]);
}

#[test]
fn extras_ignores_dependency_groups() {
    assert!(!extras(FIXTURE).iter().any(|n| n == "tests"));
}

#[test]
fn version_is_pyproject_verbatim() {
    assert_eq!(version(FIXTURE), "0.0.14b1");
}
