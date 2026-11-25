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

//! basic_server's app side ([app_server]): UDP / TCP listeners, heratBeat's clients, reports on the loop's ticks.

use std::collections::HashMap;
use std::io::Read;
use std::net::{Ipv4Addr, Shutdown, SocketAddr, TcpListener, TcpStream, UdpSocket};
use std::os::fd::AsRawFd;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use anyhow::{Context, Result};
use tracing::{info, warn};

use crate::app_commands::{App, Links};
use crate::app_frame::{decode_datagram, Frame, Stream, HEADER_SIZE};
use crate::app_reports::Snapshots;
use crate::config::AppServerConfig;

/// The largest frame the 16-bit length allows.
const MAX_FRAME: usize = HEADER_SIZE + u16::MAX as usize;

/// One whole write without waiting (MSG_DONTWAIT): a client whose socket is full is dropped, never waited on.
fn send_now(stream: &TcpStream, bytes: &[u8]) -> bool {
    // SAFETY: send reads `bytes` only, on a socket this stream owns
    let n = unsafe {
        libc::send(
            stream.as_raw_fd(),
            bytes.as_ptr().cast(),
            bytes.len(),
            libc::MSG_DONTWAIT,
        )
    };
    n == bytes.len() as isize
}

struct Client {
    /// a heartbeat (100/100) since the last check
    alive: bool,
    /// the header bytes of its first heartbeat (client class, format), which its reports carry
    like: Frame,
}

#[derive(Default)]
struct ClientList {
    /// keyed by ip and port alone: a UDP and a TCP client on one address are one client
    clients: HashMap<SocketAddr, Client>,
    /// the open TCP connections: a client on one gets its reports there, else from the UDP socket
    tcp: HashMap<SocketAddr, TcpStream>,
    /// a frame from anyone since the last check
    heard: bool,
    /// "Heartbeat timeout!" logged for this silence
    silence_logged: bool,
}

/// heratBeat: a heartbeat registers its sender; each check drops the clients that sent none since the last.
#[derive(Default)]
pub struct Clients {
    list: Mutex<ClientList>,
}

impl Clients {
    fn lock(&self) -> std::sync::MutexGuard<'_, ClientList> {
        self.list.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// ParseData: any frame with a good header counts as activity.
    pub fn activity(&self) {
        let mut l = self.lock();
        l.heard = true;
        l.silence_logged = false;
    }

    /// heratBeat::wakeUp: register a client (its header bytes kept from this first one) or mark it alive.
    pub fn heartbeat(&self, from: SocketAddr, like: &Frame) {
        let mut l = self.lock();
        l.clients
            .entry(from)
            .or_insert_with(|| {
                info!(client = %from, "app client connected");
                Client {
                    alive: true,
                    like: Frame::answering(like, 0, Vec::new()),
                }
            })
            .alive = true;
    }

    /// heartbeatCheckThread: after any frame, drop the clients with no heartbeat since the last check; else all.
    pub fn check(&self) {
        let mut l = self.lock();
        if std::mem::take(&mut l.heard) {
            l.clients.retain(|from, c| {
                if !c.alive {
                    info!(client = %from, "app client missed its heartbeat");
                }
                std::mem::take(&mut c.alive)
            });
        } else if !l.clients.is_empty() {
            if !l.silence_logged {
                info!("app heartbeat timeout");
            }
            l.silence_logged = true;
            l.clients.clear();
        }
    }

    pub fn live(&self) -> Vec<SocketAddr> {
        self.lock().clients.keys().copied().collect()
    }

    fn connected(&self, from: SocketAddr, stream: TcpStream) {
        self.lock().tcp.insert(from, stream);
    }

    fn disconnected(&self, from: SocketAddr) {
        self.lock().tcp.remove(&from);
    }

    /// Close every connection (their readers see the end).
    fn close_all(&self) {
        for stream in self.lock().tcp.values() {
            let _ = stream.shutdown(Shutdown::Both);
        }
    }

    /// netSend: each client its report, on its TCP connection or from the UDP socket; a failed send drops it.
    fn broadcast(&self, udp: &UdpSocket, payload: &[u8]) {
        let mut l = self.lock();
        let ClientList { clients, tcp, .. } = &mut *l;
        clients.retain(|from, c| {
            let bytes = Frame::answering(&c.like, 0, payload).encode();
            match tcp.get(from) {
                Some(stream) if send_now(stream, &bytes) => true,
                Some(stream) => {
                    // a frame cut short would garble the stream: the connection ends
                    let _ = stream.shutdown(Shutdown::Both);
                    false
                }
                None => udp.send_to(&bytes, from).is_ok(),
            }
        });
    }
}

/// The running server; stop() wakes each thread from its blocking wait.
pub struct AppServer {
    stop: Arc<AtomicBool>,
    threads: Vec<std::thread::JoinHandle<()>>,
    snapshots: Arc<Snapshots>,
    pub udp_addr: SocketAddr,
    pub tcp_addr: SocketAddr,
    pub clients: Arc<Clients>,
}

impl AppServer {
    /// Bind both listeners and start serving; the reports follow the snapshots' ticks.
    pub fn start(cfg: &AppServerConfig, links: Links) -> Result<AppServer> {
        let ticks = links
            .snapshots
            .take_ticks()
            .context("the snapshots' ticks are taken by another server")?;
        let udp = UdpSocket::bind((cfg.bind.as_str(), cfg.udp_port))
            .with_context(|| format!("binding UDP {}:{}", cfg.bind, cfg.udp_port))?;
        let tcp = TcpListener::bind((cfg.bind.as_str(), cfg.tcp_port))
            .with_context(|| format!("binding TCP {}:{}", cfg.bind, cfg.tcp_port))?;
        let (udp_addr, tcp_addr) = (udp.local_addr()?, tcp.local_addr()?);
        let udp = Arc::new(udp);
        let stop = Arc::new(AtomicBool::new(false));
        let clients = Arc::new(Clients::default());
        let snapshots = links.snapshots.clone();
        let app = Arc::new(App::new(cfg.clone(), links));
        let mut threads = Vec::new();
        let spawn = |name: &str, f: Box<dyn FnOnce() + Send>| {
            std::thread::Builder::new()
                .name(name.into())
                .spawn(f)
                .with_context(|| format!("starting {name}"))
        };
        {
            let (udp, stop, clients, app) =
                (udp.clone(), stop.clone(), clients.clone(), app.clone());
            threads.push(spawn(
                "m20-app-udp",
                Box::new(move || serve_udp(&udp, &stop, &clients, &app)),
            )?);
        }
        {
            let (stop, clients, app) = (stop.clone(), clients.clone(), app.clone());
            threads.push(spawn(
                "m20-app-tcp",
                Box::new(move || serve_tcp(&tcp, &stop, &clients, &app)),
            )?);
        }
        {
            let (stop, clients, app) = (stop.clone(), clients.clone(), app.clone());
            let cfg = cfg.clone();
            threads.push(spawn(
                "m20-app-reports",
                Box::new(move || report_loop(&cfg, &ticks, &udp, &stop, &clients, &app)),
            )?);
        }
        info!(%udp_addr, %tcp_addr, "vendor app server listening");
        Ok(AppServer {
            stop,
            threads,
            snapshots,
            udp_addr,
            tcp_addr,
            clients,
        })
    }

    pub fn stop(mut self) {
        self.stop.store(true, Ordering::Relaxed);
        // each blocking wait gets the event it waits for
        let local = |a: SocketAddr| {
            let ip = if a.ip().is_unspecified() {
                Ipv4Addr::LOCALHOST.into()
            } else {
                a.ip()
            };
            SocketAddr::new(ip, a.port())
        };
        if let Ok(s) = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)) {
            let _ = s.send_to(&[], local(self.udp_addr));
        }
        let _ = TcpStream::connect(local(self.tcp_addr));
        self.clients.close_all();
        self.snapshots.wake();
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
    }
}

/// Each datagram's frame, answered to its sender from this socket.
fn serve_udp(udp: &UdpSocket, stop: &AtomicBool, clients: &Clients, app: &App) {
    let mut buf = vec![0u8; MAX_FRAME];
    while !stop.load(Ordering::Relaxed) {
        let Ok((n, from)) = udp.recv_from(&mut buf) else {
            continue;
        };
        let Some(frame) = decode_datagram(&buf[..n]) else {
            continue;
        };
        if let Some(reply) = app.handle(&frame, from, clients) {
            let _ = udp.send_to(&reply.encode(), from);
        }
    }
}

/// Accept connections; each one's frames are read (and answered) on a thread of its own.
fn serve_tcp(
    listener: &TcpListener,
    stop: &Arc<AtomicBool>,
    clients: &Arc<Clients>,
    app: &Arc<App>,
) {
    let mut connections = Vec::new();
    for accepted in listener.incoming() {
        if stop.load(Ordering::Relaxed) {
            break;
        }
        let Ok(stream) = accepted else {
            continue;
        };
        let (stop, clients, app) = (stop.clone(), clients.clone(), app.clone());
        let spawned = std::thread::Builder::new()
            .name("m20-app-tcp-client".into())
            .spawn(move || serve_connection(stream, &stop, &clients, &app));
        match spawned {
            Ok(t) => connections.push(t),
            Err(error) => warn!(%error, "cannot serve an app connection"),
        }
        connections.retain(|t| !t.is_finished());
    }
    for t in connections {
        let _ = t.join();
    }
}

fn serve_connection(mut stream: TcpStream, stop: &AtomicBool, clients: &Clients, app: &App) {
    let Ok(from) = stream.peer_addr() else {
        return;
    };
    let _ = stream.set_nodelay(true);
    let Ok(writer) = stream.try_clone() else {
        return;
    };
    info!(client = %from, "app TCP connection");
    clients.connected(from, writer);
    let mut frames = Stream::default();
    let mut buf = vec![0u8; MAX_FRAME];
    'read: while !stop.load(Ordering::Relaxed) {
        match stream.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => {
                for frame in frames.push(&buf[..n]) {
                    let Some(reply) = app.handle(&frame, from, clients) else {
                        continue;
                    };
                    if !send_now(&stream, &reply.encode()) {
                        break 'read;
                    }
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
            Err(_) => break,
        }
    }
    let _ = stream.shutdown(Shutdown::Both);
    clients.disconnected(from);
}

/// UploadThread on the loop's snapshot ticks: motion each, device / basic, heartbeat checks and errors every n-th.
fn report_loop(
    cfg: &AppServerConfig,
    ticks: &std::sync::mpsc::Receiver<()>,
    udp: &UdpSocket,
    stop: &AtomicBool,
    clients: &Clients,
    app: &App,
) {
    let mut tick = 0u64;
    while ticks.recv().is_ok() && !stop.load(Ordering::Relaxed) {
        tick += 1;
        if tick.is_multiple_of(cfg.heartbeat_check_every) {
            clients.check();
        }
        if clients.live().is_empty() {
            continue;
        }
        if tick.is_multiple_of(cfg.error_report_every) {
            clients.broadcast(udp, &app.error_report());
        }
        for payload in app.reports(tick.is_multiple_of(cfg.slow_report_every)) {
            clients.broadcast(udp, &payload);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_check_drops_the_silent_clients_and_all_of_them_when_nobody_sent_anything() {
        let c = Clients::default();
        let (a, b): (SocketAddr, SocketAddr) =
            ("10.0.0.1:1".parse().unwrap(), "10.0.0.2:2".parse().unwrap());
        let like = Frame::json(0, "");
        c.activity();
        c.heartbeat(a, &like);
        c.heartbeat(b, &like);
        c.check();
        assert_eq!(c.live().len(), 2);
        // only a's heartbeat since: b is dropped
        c.activity();
        c.heartbeat(a, &like);
        c.check();
        assert_eq!(c.live(), [a]);
        // no frame at all since the last check: all dropped
        c.check();
        assert!(c.live().is_empty());
    }
}
