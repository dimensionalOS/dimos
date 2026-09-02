//! How root is obtained, resolved once before any plan runs; never hangs, never leaks the password.

use std::fmt;
use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::time::Duration;

use anyhow::{bail, Result};

use crate::plan::{self, Mode};
use crate::probe;

const NO_SUDO: &str =
    "sudo is not installed: run as root, or install sudo (apt-get install -y sudo)";
const NO_ROOT: &str =
    "root is needed and unavailable: no tty, no SUDO_ASKPASS, no DIMOS_SUDO_PASSWORD";
const REJECTED: &str = "DIMOS_SUDO_PASSWORD rejected: `sudo -S -k -v` did not accept it";
const PROBE_TIMEOUT_S: u64 = 10;
/// PAM on a slow box can take seconds; a hung PAM must not hang the run.
const VALIDATE_TIMEOUT_S: u64 = 30;

/// A password. No `Serialize` impl exists, and Debug/Display print `<redacted>`.
pub struct Secret(String);

impl Secret {
    pub fn new(value: String) -> Secret {
        Secret(value)
    }

    fn expose(&self) -> &str {
        &self.0
    }
}

impl fmt::Debug for Secret {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("<redacted>")
    }
}

impl fmt::Display for Secret {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("<redacted>")
    }
}

#[derive(Debug)]
pub enum Sudo {
    Root,
    Passwordless,
    Askpass(PathBuf),
    Stdin(Secret),
    Tty,
    Unavailable(String),
}

impl Sudo {
    /// Probe the machine once, then pick by the pure priority in `choose`.
    pub fn resolve(mode: Mode) -> Sudo {
        Sudo::pick(
            euid(),
            which::which("sudo").is_ok(),
            sudo_n_ok(),
            askpass(),
            env_password(),
            mode == Mode::Interactive && crate::plan::stdin_is_tty(),
            validate,
        )
    }

    /// `choose`, then test the password only when it is the way root would be obtained.
    fn pick(
        euid: u32,
        sudo_installed: bool,
        sudo_n_ok: bool,
        askpass: Option<PathBuf>,
        password: Option<Secret>,
        interactive_tty: bool,
        accepted: impl Fn(&Secret) -> bool,
    ) -> Sudo {
        let picked = Sudo::choose(
            euid,
            sudo_installed,
            sudo_n_ok,
            askpass,
            password,
            interactive_tty,
        );
        match picked {
            Sudo::Stdin(secret) if !accepted(&secret) => Sudo::Unavailable(REJECTED.to_string()),
            other => other,
        }
    }

    pub fn choose(
        euid: u32,
        sudo_installed: bool,
        sudo_n_ok: bool,
        askpass: Option<PathBuf>,
        password: Option<Secret>,
        interactive_tty: bool,
    ) -> Sudo {
        if euid == 0 {
            return Sudo::Root;
        }
        if !sudo_installed {
            return Sudo::Unavailable(NO_SUDO.to_string());
        }
        if sudo_n_ok {
            return Sudo::Passwordless;
        }
        if let Some(path) = askpass {
            return Sudo::Askpass(path);
        }
        if let Some(secret) = password {
            return Sudo::Stdin(secret);
        }
        if interactive_tty {
            return Sudo::Tty;
        }
        Sudo::Unavailable(NO_ROOT.to_string())
    }

    pub fn available(&self) -> bool {
        !matches!(self, Sudo::Unavailable(_))
    }

    /// A terminal prompt happens here, on the human's clock, never inside an action's deadline.
    pub fn refresh(&self) -> Result<()> {
        if !matches!(self, Sudo::Tty) {
            return Ok(());
        }
        let status = Command::new("sudo").arg("-v").status();
        if status.is_ok_and(|s| s.success()) {
            return Ok(());
        }
        bail!("sudo -v did not accept a password: re-run, or set DIMOS_SUDO_PASSWORD")
    }

    /// A `Tty` that cannot get a ticket becomes `Unavailable`, so every sudo stage is an exit 2.
    pub fn refresh_or_demote(&mut self) {
        if let Err(why) = self.refresh() {
            *self = Sudo::Unavailable(format!("{why:#}"));
        }
    }

    /// The argv to spawn plus the bytes to feed its stdin; the password is only ever in those bytes.
    pub fn wrap(&self, argv: &[String]) -> (Vec<String>, Option<Vec<u8>>) {
        let prefix: &[&str] = match self {
            Sudo::Root | Sudo::Unavailable(_) => &[],
            Sudo::Passwordless => &["sudo", "-n", "--"],
            Sudo::Askpass(_) => &["sudo", "-A", "--"],
            Sudo::Stdin(_) => &["sudo", "-S", "-p", "", "--"],
            Sudo::Tty => &["sudo", "--"],
        };
        let mut out: Vec<String> = prefix.iter().map(|s| (*s).to_string()).collect();
        out.extend_from_slice(argv);
        let stdin = match self {
            Sudo::Stdin(secret) => Some(format!("{}\n", secret.expose()).into_bytes()),
            _ => None,
        };
        (out, stdin)
    }

    /// The exit-2 text: what the operator types so the same run works next time.
    pub fn human_fix(&self) -> String {
        match self {
            Sudo::Unavailable(why) => format!(
                "{why}\n  fix any one of:\n\
                 \x20   read -rs DIMOS_SUDO_PASSWORD && export DIMOS_SUDO_PASSWORD   (typed, never on a command line)\n\
                 \x20   export SUDO_ASKPASS=/path/to/askpass-helper\n\
                 \x20   re-run in a terminal, or add a sudoers NOPASSWD line for the commands above"
            ),
            _ => "root is already available".to_string(),
        }
    }
}

/// `id -u` is the portable euid read without a libc dependency; unreadable means assume not root.
fn euid() -> u32 {
    probe::capture("id", &["-u"], &[], PROBE_TIMEOUT_S)
        .and_then(|out| out.parse().ok())
        .unwrap_or(1000)
}

fn sudo_n_ok() -> bool {
    probe::capture("sudo", &["-n", "true"], &[], PROBE_TIMEOUT_S).is_some()
}

fn askpass() -> Option<PathBuf> {
    use std::os::unix::fs::PermissionsExt;
    let path = PathBuf::from(std::env::var_os("SUDO_ASKPASS")?);
    let mode = std::fs::metadata(&path).ok()?.permissions().mode();
    (mode & 0o111 != 0).then_some(path)
}

fn env_password() -> Option<Secret> {
    let raw = std::env::var_os("DIMOS_SUDO_PASSWORD")?;
    std::env::remove_var("DIMOS_SUDO_PASSWORD"); // no child of ours may inherit it
    Some(Secret::new(raw.to_string_lossy().into_owned()))
}

/// `-k` drops any cached ticket, so this tests the password and not a live sudo session.
fn validate(secret: &Secret) -> bool {
    let Ok(mut child) = Command::new("sudo")
        .args(["-S", "-k", "-p", "", "-v"])
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
    else {
        return false;
    };
    if let Some(mut pipe) = child.stdin.take() {
        let _ = pipe.write_all(format!("{}\n", secret.expose()).as_bytes());
    }
    plan::wait_until(&mut child, Duration::from_secs(VALIDATE_TIMEOUT_S))
        .ok()
        .flatten()
        .is_some_and(|s| s.success())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn argv() -> Vec<String> {
        vec!["apt-get".to_string(), "install".to_string()]
    }

    #[test]
    fn root_wins_over_everything() {
        let picked = Sudo::choose(
            0,
            true,
            true,
            Some(PathBuf::from("/a")),
            Some(Secret::new("p".into())),
            true,
        );
        assert!(matches!(picked, Sudo::Root));
        assert_eq!(picked.wrap(&argv()).0, argv());
    }

    #[test]
    fn passwordless_beats_askpass_beats_stdin_beats_tty() {
        let ap = || Some(PathBuf::from("/a"));
        let pw = || Some(Secret::new("p".into()));
        assert!(matches!(
            Sudo::choose(1000, true, true, ap(), pw(), true),
            Sudo::Passwordless
        ));
        assert!(matches!(
            Sudo::choose(1000, true, false, ap(), pw(), true),
            Sudo::Askpass(_)
        ));
        assert!(matches!(
            Sudo::choose(1000, true, false, None, pw(), true),
            Sudo::Stdin(_)
        ));
        assert!(matches!(
            Sudo::choose(1000, true, false, None, None, true),
            Sudo::Tty
        ));
    }

    #[test]
    fn missing_sudo_binary_is_unavailable_naming_the_install_command() {
        let picked = Sudo::choose(1000, false, false, None, None, true);
        let Sudo::Unavailable(why) = &picked else {
            panic!("want Unavailable, got {picked:?}")
        };
        assert!(why.contains("apt-get install -y sudo"), "{why}");
    }

    #[test]
    fn no_tty_no_creds_is_unavailable_with_both_env_names_in_human_fix() {
        let picked = Sudo::choose(1000, true, false, None, None, false);
        assert!(!picked.available());
        let fix = picked.human_fix();
        assert!(fix.contains("read -rs DIMOS_SUDO_PASSWORD"), "{fix}");
        assert!(fix.contains("SUDO_ASKPASS"), "{fix}");
    }

    #[test]
    fn a_stale_password_is_never_consulted_when_root_or_passwordless_sudo_is_there() {
        let pw = || Some(Secret::new("stale".into()));
        let never = |_: &Secret| panic!("the password was tested before the cheaper routes");
        assert!(matches!(
            Sudo::pick(0, true, false, None, pw(), false, never),
            Sudo::Root
        ));
        assert!(matches!(
            Sudo::pick(1000, true, true, None, pw(), false, never),
            Sudo::Passwordless
        ));
        assert!(matches!(
            Sudo::pick(1000, false, false, None, pw(), false, never),
            Sudo::Unavailable(why) if why == NO_SUDO
        ));
    }

    #[test]
    fn a_rejected_password_is_unavailable_only_when_it_would_have_been_used() {
        let pw = || Some(Secret::new("wrong".into()));
        assert!(matches!(
            Sudo::pick(1000, true, false, None, pw(), true, |_| false),
            Sudo::Unavailable(why) if why == REJECTED
        ));
        assert!(matches!(
            Sudo::pick(1000, true, false, None, pw(), true, |_| true),
            Sudo::Stdin(_)
        ));
    }

    #[test]
    fn refresh_is_a_noop_for_every_route_but_the_terminal() {
        for picked in [
            Sudo::Root,
            Sudo::Passwordless,
            Sudo::Askpass(PathBuf::from("/a")),
            Sudo::Stdin(Secret::new("p".into())),
            Sudo::Unavailable("no".into()),
        ] {
            assert!(picked.refresh().is_ok(), "{picked:?}");
        }
    }

    #[test]
    fn stdin_wrap_puts_password_only_in_the_stdin_bytes_never_argv() {
        let (cmd, stdin) = Sudo::Stdin(Secret::new("hunter2".into())).wrap(&argv());
        assert_eq!(cmd[..5], ["sudo", "-S", "-p", "", "--"]);
        assert!(!cmd.iter().any(|a| a.contains("hunter2")));
        assert_eq!(stdin.unwrap(), b"hunter2\n");
    }

    #[test]
    fn askpass_wrap_uses_dash_a_and_passwordless_dash_n() {
        assert_eq!(
            Sudo::Askpass(PathBuf::from("/a")).wrap(&argv()).0[..3],
            ["sudo", "-A", "--"]
        );
        assert_eq!(
            Sudo::Passwordless.wrap(&argv()).0[..3],
            ["sudo", "-n", "--"]
        );
        assert_eq!(Sudo::Tty.wrap(&argv()).0[..2], ["sudo", "--"]);
    }

    #[test]
    fn secret_debug_and_display_are_redacted() {
        let secret = Secret::new("hunter2".into());
        assert_eq!(format!("{secret:?}"), "<redacted>");
        assert_eq!(format!("{secret}"), "<redacted>");
        assert_eq!(format!("{:?}", Sudo::Stdin(secret)), "Stdin(<redacted>)");
    }
}
