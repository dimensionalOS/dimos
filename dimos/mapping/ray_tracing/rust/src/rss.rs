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

//! This process's resident memory and state dumps, for the periodic memory log.

/// Resident set size in MB, None when the platform can't say.
#[cfg(target_os = "linux")]
pub fn rss_mb() -> Option<f64> {
    let statm = std::fs::read_to_string("/proc/self/statm").ok()?;
    let pages: f64 = statm.split_whitespace().nth(1)?.parse().ok()?;
    // SAFETY: sysconf has no preconditions.
    let page = unsafe { libc::sysconf(libc::_SC_PAGESIZE) } as f64;
    Some(pages * page / 1e6)
}

/// Resident set size in MB, None when the platform can't say.
#[cfg(target_os = "macos")]
pub fn rss_mb() -> Option<f64> {
    let mut info: libc::proc_taskinfo = unsafe { std::mem::zeroed() };
    let size = std::mem::size_of::<libc::proc_taskinfo>() as libc::c_int;
    // SAFETY: info is a proc_taskinfo of exactly `size` bytes.
    let got = unsafe {
        libc::proc_pidinfo(
            libc::getpid(),
            libc::PROC_PIDTASKINFO,
            0,
            (&mut info as *mut libc::proc_taskinfo).cast(),
            size,
        )
    };
    (got == size).then(|| info.pti_resident_size as f64 / 1e6)
}

/// Resident set size in MB, None when the platform can't say.
#[cfg(not(any(target_os = "linux", target_os = "macos")))]
pub fn rss_mb() -> Option<f64> {
    None
}

/// Write flat little-endian f32s to `path` atomically: a temp file renamed over it.
pub fn dump_f32(path: &str, values: &[f32]) -> std::io::Result<()> {
    let bytes: Vec<u8> = values.iter().flat_map(|v| v.to_le_bytes()).collect();
    let tmp = format!("{path}.tmp");
    std::fs::write(&tmp, bytes)?;
    std::fs::rename(&tmp, path)
}
