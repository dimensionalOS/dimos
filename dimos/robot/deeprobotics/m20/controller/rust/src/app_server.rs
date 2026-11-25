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

//! The vendor phone app's server in place of basic_server's ([app_server] in m20_controller.toml): the UDP and TCP
//! listeners (UdpServer / TcpServer), heratBeat's client list, each command into the operator's command path
//! (crate::app_commands), and UploadThread's reports (crate::app_reports) pushed to every live client. Its threads
//! only read what the 1 kHz loop hands over (Snapshots, written there with try_lock) and write the operator's slots.
//! Not served: basic_server's HTTP command port (30002).

use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use tracing::{info, warn};

use crate::app_commands::{App, Links};
use crate::app_frame::{decode_datagram, Frame, Stream};
use crate::config::AppServerConfig;

/// A TCP connection's writing half: replies and reports take turns on it, so frames never interleave.
pub type TcpWriter = Arc<Mutex<TcpStream>>;

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
    tcp: HashMap<SocketAddr, TcpWriter>,
    /// the last frame from anyone
    last_frame: Option<Instant>,
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
    pub fn activity(&self, now: Instant) {
        let mut l = self.lock();
        l.last_frame = Some(now);
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

    /// heartbeatCheckThread: with a frame from anyone within `period`, drop the clients with no heartbeat since the
    /// last check; with none, drop them all.
    pub fn check(&self, now: Instant, period: Duration) {
        let mut l = self.lock();
        let heard = l
            .last_frame
            .is_some_and(|t| now.saturating_duration_since(t) <= period);
        if heard {
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

    fn connected(&self, from: SocketAddr, writer: &TcpWriter) {
        self.lock().tcp.insert(from, writer.clone());
    }

    fn disconnected(&self, from: SocketAddr) {
        self.lock().tcp.remove(&from);
    }

    /// netSend: each client its report, on its TCP connection if it has one, else from the UDP socket; a client
    /// whose send fails is dropped (DeleteClient).
    fn broadcast(&self, udp: &UdpSocket, payload: &[u8]) {
        let mut l = self.lock();
        let ClientList { clients, tcp, .. } = &mut *l;
        clients.retain(|from, c| {
            let bytes = Frame::answering(&c.like, 0, payload).encode();
            match tcp.get(from) {
                Some(stream) => stream
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .write_all(&bytes)
                    .is_ok(),
                None => udp.send_to(&bytes, from).is_ok(),
            }
        });
    }
}

/// The running server: its threads stop when the flag is set (each looks at it once per poll_s).
pub struct AppServer {
    stop: Arc<AtomicBool>,
    threads: Vec<std::thread::JoinHandle<()>>,
    pub udp_addr: SocketAddr,
    pub tcp_addr: SocketAddr,
    pub clients: Arc<Clients>,
}

impl AppServer {
    /// Bind both listeners and start serving.
    pub fn start(cfg: &AppServerConfig, links: Links) -> Result<AppServer> {
        let udp = UdpSocket::bind((cfg.bind.as_str(), cfg.udp_port))
            .with_context(|| format!("binding UDP {}:{}", cfg.bind, cfg.udp_port))?;
        let tcp = TcpListener::bind((cfg.bind.as_str(), cfg.tcp_port))
            .with_context(|| format!("binding TCP {}:{}", cfg.bind, cfg.tcp_port))?;
        let poll = Duration::from_secs_f64(cfg.poll_s);
        udp.set_read_timeout(Some(poll))?;
        tcp.set_nonblocking(true)?;
        let (udp_addr, tcp_addr) = (udp.local_addr()?, tcp.local_addr()?);
        let udp = Arc::new(udp);
        let stop = Arc::new(AtomicBool::new(false));
        let clients = Arc::new(Clients::default());
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
                Box::new(move || serve_tcp(&tcp, &stop, &clients, &app, poll)),
            )?);
        }
        {
            let (stop, clients, app) = (stop.clone(), clients.clone(), app.clone());
            let cfg = cfg.clone();
            threads.push(spawn(
                "m20-app-reports",
                Box::new(move || report_loop(&cfg, &udp, &stop, &clients, &app)),
            )?);
        }
        info!(%udp_addr, %tcp_addr, "vendor app server listening");
        Ok(AppServer {
            stop,
            threads,
            udp_addr,
            tcp_addr,
            clients,
        })
    }

    pub fn stop(mut self) {
        self.stop.store(true, Ordering::Relaxed);
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
    }
}

/// Each datagram's frame, answered to its sender from this socket.
fn serve_udp(udp: &UdpSocket, stop: &AtomicBool, clients: &Clients, app: &App) {
    let mut buf = vec![0u8; 65536];
    while !stop.load(Ordering::Relaxed) {
        // a timeout (or EINTR) only ends this wait
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
    poll: Duration,
) {
    let mut connections = Vec::new();
    while !stop.load(Ordering::Relaxed) {
        match listener.accept() {
            Ok((stream, from)) => {
                let (stop, clients, app) = (stop.clone(), clients.clone(), app.clone());
                let spawned = std::thread::Builder::new()
                    .name("m20-app-tcp-client".into())
                    .spawn(move || serve_connection(stream, from, &stop, &clients, &app, poll));
                match spawned {
                    Ok(t) => connections.push(t),
                    Err(error) => warn!(%error, "cannot serve an app connection"),
                }
            }
            Err(_) => std::thread::sleep(poll),
        }
        connections.retain(|t| !t.is_finished());
    }
    for t in connections {
        let _ = t.join();
    }
}

fn serve_connection(
    mut stream: TcpStream,
    from: SocketAddr,
    stop: &AtomicBool,
    clients: &Clients,
    app: &App,
    poll: Duration,
) {
    let _ = stream.set_nonblocking(false);
    let _ = stream.set_read_timeout(Some(poll));
    let _ = stream.set_nodelay(true);
    // a client that stops reading costs one poll per send, then is dropped
    let _ = stream.set_write_timeout(Some(poll));
    let Ok(writer) = stream.try_clone().map(|s| Arc::new(Mutex::new(s))) else {
        return;
    };
    info!(client = %from, "app TCP connection");
    clients.connected(from, &writer);
    let mut frames = Stream::default();
    let mut buf = vec![0u8; 65536];
    'read: while !stop.load(Ordering::Relaxed) {
        match stream.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => {
                for frame in frames.push(&buf[..n]) {
                    let Some(reply) = app.handle(&frame, from, clients) else {
                        continue;
                    };
                    let sent = writer
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .write_all(&reply.encode());
                    if sent.is_err() {
                        break 'read;
                    }
                }
            }
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock
                        | std::io::ErrorKind::TimedOut
                        | std::io::ErrorKind::Interrupted
                ) => {}
            Err(_) => break,
        }
    }
    clients.disconnected(from);
}

/// UploadThread: each period with a client the motion report, every slow_report_every-th the device and basic ones;
/// ExceptionHandle's error list each error_report_period_s; heartbeatCheckThread's check each heartbeat_timeout_s.
fn report_loop(
    cfg: &AppServerConfig,
    udp: &UdpSocket,
    stop: &AtomicBool,
    clients: &Clients,
    app: &App,
) {
    let period = Duration::from_secs_f64(cfg.report_period_s);
    let check = Duration::from_secs_f64(cfg.heartbeat_timeout_s);
    let errors = Duration::from_secs_f64(cfg.error_report_period_s);
    let start = Instant::now();
    let (mut next, mut next_check, mut next_errors) = (start, start + check, start + errors);
    let mut tick = 0u64;
    while !stop.load(Ordering::Relaxed) {
        next += period;
        let now = Instant::now();
        match next.checked_duration_since(now) {
            Some(wait) => std::thread::sleep(wait),
            None => next = now,
        }
        let now = Instant::now();
        if now >= next_check {
            next_check += check;
            clients.check(now, check);
        }
        if now >= next_errors {
            next_errors += errors;
            clients.broadcast(udp, &app.error_report());
        }
        if clients.live().is_empty() {
            continue;
        }
        let slow = tick.is_multiple_of(cfg.slow_report_every);
        for payload in app.reports(slow) {
            clients.broadcast(udp, &payload);
        }
        tick += 1;
    }
}
