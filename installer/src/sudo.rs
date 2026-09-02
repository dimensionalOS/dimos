//! How root is obtained, resolved once before any plan runs; never hangs, never leaks the password.

use std::fmt;
use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Stdio};

use crate::plan::Mode;

const NO_SUDO: &str =
    "sudo is not installed: run as root, or install sudo (apt-get install -y sudo)";
const NO_ROOT: &str =
    "root is needed and unavailable: no tty, no SUDO_ASKPASS, no DIMOS_SUDO_PASSWORD";
const REJECTED: &str = "DIMOS_SUDO_PASSWORD rejected: `sudo -S -k -v` did not accept it";

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
        match env_password() {
            Err(why) => Sudo::Unavailable(why),
            Ok(password) => Sudo::choose(
                euid(),
                which::which("sudo").is_ok(),
                sudo_n_ok(),
                askpass(),
                password,
                mode == Mode::Interactive && crate::plan::stdin_is_tty(),
            ),
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
                 \x20   export DIMOS_SUDO_PASSWORD='...'   (env only, never on the command line)\n\
                 \x20   export SUDO_ASKPASS=/path/to/askpass-helper\n\
                 \x20   re-run in a terminal, or add a sudoers NOPASSWD line for the commands above"
            ),
            _ => "root is already available".to_string(),
        }
    }
}

/// `id -u` is the portable euid read without a libc dependency; unreadable means assume not root.
fn euid() -> u32 {
    Command::new("id")
        .arg("-u")
        .output()
        .ok()
        .filter(|o| o.status.success())
        .and_then(|o| String::from_utf8_lossy(&o.stdout).trim().parse().ok())
        .unwrap_or(1000)
}

fn sudo_n_ok() -> bool {
    Command::new("sudo")
        .args(["-n", "true"])
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

fn askpass() -> Option<PathBuf> {
    use std::os::unix::fs::PermissionsExt;
    let path = PathBuf::from(std::env::var_os("SUDO_ASKPASS")?);
    let mode = std::fs::metadata(&path).ok()?.permissions().mode();
    (mode & 0o111 != 0).then_some(path)
}

/// `Err` when the var was set but sudo refused it, so a bad password never falls through to a prompt.
fn env_password() -> Result<Option<Secret>, String> {
    let Some(raw) = std::env::var_os("DIMOS_SUDO_PASSWORD") else {
        return Ok(None);
    };
    std::env::remove_var("DIMOS_SUDO_PASSWORD"); // no child of ours may inherit it
    let secret = Secret::new(raw.to_string_lossy().into_owned());
    if validate(&secret) {
        Ok(Some(secret))
    } else {
        Err(REJECTED.to_string())
    }
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
    child.wait().map(|s| s.success()).unwrap_or(false)
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
        assert!(fix.contains("DIMOS_SUDO_PASSWORD"), "{fix}");
        assert!(fix.contains("SUDO_ASKPASS"), "{fix}");
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
