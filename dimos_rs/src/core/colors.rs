pub fn green(text: &str) -> String {
    format!("\x1b[92m{}\x1b[0m", text)
}

pub fn blue(text: &str) -> String {
    format!("\x1b[94m{}\x1b[0m", text)
}

pub fn red(text: &str) -> String {
    format!("\x1b[91m{}\x1b[0m", text)
}

pub fn yellow(text: &str) -> String {
    format!("\x1b[93m{}\x1b[0m", text)
}

pub fn purple(text: &str) -> String {
    format!("\x1b[95m{}\x1b[0m", text)
}
