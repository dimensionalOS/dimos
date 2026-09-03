//! Version comparison and the release URLs the artifacts are fetched from.

pub(crate) const RELEASES: &str = "https://github.com/dimensionalOS/dimos/releases";
/// A pre-release ranks below every marker, so a final release takes the highest rank.
const FINAL: u8 = 3;

/// `https://github.com/x/y/releases/tag/v0.0.15` -> `0.0.15`.
pub(crate) fn parse_latest_tag(url: &str) -> Option<String> {
    let (_, tag) = url.trim().rsplit_once("/tag/")?;
    let version = normalize(tag);
    (!version.is_empty()).then_some(version)
}

/// The 40-hex object id `git ls-remote` prints before the ref name.
pub(crate) fn parse_ls_remote(text: &str) -> Option<String> {
    let sha = text.split_whitespace().next()?;
    (sha.len() == 40 && sha.chars().all(|c| c.is_ascii_hexdigit())).then(|| sha.to_string())
}

pub(crate) fn normalize(tag: &str) -> String {
    tag.trim().trim_start_matches('v').to_string()
}

/// PEP 440-lite: the release numbers, then a rank that sorts `0.0.14b1` below `0.0.14`.
fn version_key(v: &str) -> (Vec<u32>, (u8, u32)) {
    let stripped = normalize(v);
    let (release, pre) = split_pre(&stripped);
    let numbers = release.split('.').map(|p| p.parse().unwrap_or(0)).collect();
    (numbers, pre)
}

/// `rc` first: it carries a `c`, so a later `a`/`b` split would cut a release candidate in half.
fn split_pre(v: &str) -> (&str, (u8, u32)) {
    for (marker, rank) in [("rc", 2u8), ("a", 0), ("b", 1)] {
        if let Some((release, n)) = v.split_once(marker) {
            return (release, (rank, n.parse().unwrap_or(0)));
        }
    }
    (v, (FINAL, 0))
}

pub(crate) fn newer(current: &str, candidate: &str) -> bool {
    version_key(candidate) > version_key(current)
}

pub(crate) fn release_base(version: Option<&str>, override_url: Option<&str>) -> String {
    match (override_url, version) {
        (Some(url), _) => url.trim_end_matches('/').to_string(),
        (None, Some(v)) => format!("{RELEASES}/download/v{}", normalize(v)),
        (None, None) => format!("{RELEASES}/latest/download"),
    }
}

pub(crate) fn artifact(target: &str) -> String {
    format!("dimos-{target}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn artifact_is_dimos_dash_target() {
        assert_eq!(artifact("aarch64-linux-musl"), "dimos-aarch64-linux-musl");
    }

    #[test]
    fn release_base_prefers_the_url_override_then_the_pinned_tag_then_latest() {
        assert_eq!(
            release_base(Some("0.0.15"), Some("http://10.0.0.5:8000/")),
            "http://10.0.0.5:8000"
        );
        assert_eq!(
            release_base(Some("v0.0.15"), None),
            format!("{RELEASES}/download/v0.0.15")
        );
        assert_eq!(
            release_base(None, None),
            format!("{RELEASES}/latest/download")
        );
    }

    #[test]
    fn version_key_sorts_a_prerelease_below_its_final_release() {
        assert!(version_key("0.0.14b1") < version_key("0.0.14"));
        assert!(version_key("0.0.14") < version_key("0.0.15"));
        assert!(version_key("0.0.14a2") < version_key("0.0.14b1"));
        assert!(version_key("0.0.14b1") < version_key("0.0.14rc1"));
    }

    #[test]
    fn newer_is_false_for_the_same_version() {
        assert!(!newer("0.0.14", "0.0.14"));
        assert!(newer("0.0.14", "0.0.15"));
        assert!(!newer("0.0.15", "0.0.14b1"));
    }

    #[test]
    fn parse_latest_tag_reads_the_version_out_of_the_redirect_url() {
        assert_eq!(
            parse_latest_tag("https://github.com/dimensionalOS/dimos/releases/tag/v0.0.15\n"),
            Some("0.0.15".to_string())
        );
        assert_eq!(parse_latest_tag("https://github.com/x/y/releases"), None);
    }

    #[test]
    fn parse_ls_remote_takes_the_object_id_not_the_ref_name() {
        let line = "9f8b2c1d0e4a6b7c8d9e0f1a2b3c4d5e6f708192\trefs/heads/main\n";
        assert_eq!(
            parse_ls_remote(line),
            Some("9f8b2c1d0e4a6b7c8d9e0f1a2b3c4d5e6f708192".to_string())
        );
        assert_eq!(parse_ls_remote("fatal: could not read"), None);
    }
}
