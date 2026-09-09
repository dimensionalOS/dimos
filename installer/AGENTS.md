# Installer development

Follow root AGENTS.md. This independent Rust workspace contains installation code
only. Run Cargo tests and Clippy with --manifest-path installer/Cargo.toml.
Keep required failures fatal; do not restore app/service management or publishing.
