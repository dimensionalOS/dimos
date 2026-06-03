use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::OnceLock;
use std::time::Instant;

pub fn process_uptime_ns() -> u64 {
    static START: OnceLock<Instant> = OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_nanos() as u64
}

#[inline]
#[doc(hidden)]
pub fn check_and_record(last_ns: &AtomicU64, interval_ns: u64) -> bool {
    let now_ns = process_uptime_ns().max(1);
    let last = last_ns.load(Ordering::Relaxed);
    (last == 0 || now_ns.saturating_sub(last) >= interval_ns)
        && last_ns
            .compare_exchange(last, now_ns, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
}

#[doc(hidden)]
#[macro_export]
macro_rules! log_throttled {
    ($level:expr, $interval:expr, $($arg:tt)*) => {{
        if ::tracing::enabled!($level) {
            static LAST_NS: ::std::sync::atomic::AtomicU64 =
                ::std::sync::atomic::AtomicU64::new(0);
            if $crate::log::check_and_record(&LAST_NS, $interval.as_nanos() as u64) {
                ::tracing::event!($level, $($arg)*);
            }
        }
    }};
}

#[macro_export]
macro_rules! trace_throttled {
    ($($arg:tt)*) => { $crate::log_throttled!(::tracing::Level::TRACE, $($arg)*) };
}

#[macro_export]
macro_rules! debug_throttled {
    ($($arg:tt)*) => { $crate::log_throttled!(::tracing::Level::DEBUG, $($arg)*) };
}

#[macro_export]
macro_rules! info_throttled {
    ($($arg:tt)*) => { $crate::log_throttled!(::tracing::Level::INFO, $($arg)*) };
}

#[macro_export]
macro_rules! warn_throttled {
    ($($arg:tt)*) => { $crate::log_throttled!(::tracing::Level::WARN, $($arg)*) };
}

#[macro_export]
macro_rules! error_throttled {
    ($($arg:tt)*) => { $crate::log_throttled!(::tracing::Level::ERROR, $($arg)*) };
}

#[cfg(test)]
mod tests {
    use super::check_and_record;
    use std::sync::atomic::AtomicU64;
    use std::time::Duration;

    #[test]
    fn throttles_within_interval_then_fires_after() {
        let counter = AtomicU64::new(0);
        let interval_ns = Duration::from_millis(50).as_nanos() as u64;
        assert!(check_and_record(&counter, interval_ns));
        assert!(!check_and_record(&counter, interval_ns));

        // after waiting for > interval we should fire again
        std::thread::sleep(Duration::from_millis(75));
        assert!(check_and_record(&counter, interval_ns));
    }
}
