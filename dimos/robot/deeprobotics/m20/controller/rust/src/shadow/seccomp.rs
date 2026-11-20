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

//! A seccomp filter for the shadow process (aarch64 Linux): sendto / sendmsg / sendmmsg / socket / connect fail for
//! every thread, and write / writev / pwrite* fail on the listed descriptors (its input sockets). Installed after
//! the inputs are open, for all threads at once (TSYNC), so a send path cannot exist even by mistake.

use anyhow::Result;

#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
pub fn lock_down(input_fds: &[i32]) -> Result<()> {
    use anyhow::bail;
    // aarch64 syscall numbers (asm-generic)
    const WRITE: u32 = 64;
    const WRITEV: u32 = 66;
    const PWRITE64: u32 = 68;
    const PWRITEV: u32 = 70;
    const PWRITEV2: u32 = 287;
    const SOCKET: u32 = 198;
    const SOCKETPAIR: u32 = 199;
    const CONNECT: u32 = 203;
    const SENDTO: u32 = 206;
    const SENDMSG: u32 = 211;
    const SENDMMSG: u32 = 269;
    const AUDIT_ARCH_AARCH64: u32 = 0xC000_00B7;
    const RET_ALLOW: u32 = 0x7fff_0000;
    const RET_ERRNO_EPERM: u32 = 0x0005_0000 | 1;
    const LD_W_ABS: u16 = 0x20;
    const JEQ_K: u16 = 0x15;
    const RET_K: u16 = 0x06;
    let stmt = |code: u16, k: u32| libc::sock_filter {
        code,
        jt: 0,
        jf: 0,
        k,
    };
    let jeq = |k: u32, jt: u8, jf: u8| libc::sock_filter {
        code: JEQ_K,
        jt,
        jf,
        k,
    };
    // seccomp_data: nr at 0, arch at 4, args[0] low word at 16
    let mut p = vec![
        stmt(LD_W_ABS, 4),
        jeq(AUDIT_ARCH_AARCH64, 1, 0),
        stmt(RET_K, RET_ERRNO_EPERM),
        stmt(LD_W_ABS, 0),
    ];
    for nr in [SENDTO, SENDMSG, SENDMMSG, SOCKET, SOCKETPAIR, CONNECT] {
        p.push(jeq(nr, 0, 1));
        p.push(stmt(RET_K, RET_ERRNO_EPERM));
    }
    // writes: if the nr is one of them, check the fd
    let writes = [WRITE, WRITEV, PWRITE64, PWRITEV, PWRITEV2];
    for (k, nr) in writes.iter().enumerate() {
        // jump over the remaining nr tests to the fd check when it matches
        let skip = (writes.len() - k) as u8;
        p.push(jeq(*nr, skip, 0));
    }
    p.push(stmt(RET_K, RET_ALLOW));
    p.push(stmt(LD_W_ABS, 16));
    for &fd in input_fds {
        p.push(jeq(fd as u32, 0, 1));
        p.push(stmt(RET_K, RET_ERRNO_EPERM));
    }
    p.push(stmt(RET_K, RET_ALLOW));
    let prog = libc::sock_fprog {
        len: p.len() as u16,
        filter: p.as_mut_ptr(),
    };
    unsafe {
        if libc::prctl(libc::PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) != 0 {
            bail!("PR_SET_NO_NEW_PRIVS: {}", std::io::Error::last_os_error());
        }
        // SECCOMP_SET_MODE_FILTER with TSYNC: every thread of the process, including ones already running
        const SECCOMP_SET_MODE_FILTER: libc::c_long = 1;
        const SECCOMP_FILTER_FLAG_TSYNC: libc::c_long = 1;
        let r = libc::syscall(
            libc::SYS_seccomp,
            SECCOMP_SET_MODE_FILTER,
            SECCOMP_FILTER_FLAG_TSYNC,
            &prog as *const libc::sock_fprog,
        );
        if r != 0 {
            bail!(
                "seccomp: {} (thread sync failed: {r})",
                std::io::Error::last_os_error()
            );
        }
    }
    Ok(())
}

/// Prove the filter is on: a send, a socket and a write on each input fd must all fail with EPERM.
#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
pub fn verify(input_fds: &[i32]) -> Result<String> {
    use anyhow::bail;
    let eperm =
        |r: isize| r == -1 && std::io::Error::last_os_error().raw_os_error() == Some(libc::EPERM);
    let byte = [0u8; 1];
    unsafe {
        if !eperm(libc::sendto(
            -1,
            byte.as_ptr() as *const libc::c_void,
            1,
            0,
            std::ptr::null(),
            0,
        )) {
            bail!("sendto was not refused");
        }
        if !eperm(libc::socket(libc::AF_PACKET, libc::SOCK_RAW, 0) as isize) {
            bail!("socket() was not refused");
        }
        for &fd in input_fds {
            if !eperm(libc::write(fd, byte.as_ptr() as *const libc::c_void, 1)) {
                bail!("write on fd {fd} was not refused");
            }
        }
    }
    Ok(format!(
        "sendto, socket() and writes on fds {input_fds:?} all refused with EPERM"
    ))
}

#[cfg(not(all(target_os = "linux", target_arch = "aarch64")))]
pub fn verify(_input_fds: &[i32]) -> Result<String> {
    Ok("no seccomp off the robot".into())
}

#[cfg(not(all(target_os = "linux", target_arch = "aarch64")))]
pub fn lock_down(_input_fds: &[i32]) -> Result<()> {
    // the shadow runs on the robot (aarch64 Linux); elsewhere it only replays files and opens no sockets
    Ok(())
}
