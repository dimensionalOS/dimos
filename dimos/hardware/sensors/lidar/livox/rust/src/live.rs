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

//! Live UDP source: configures a Mid-360 over the control plane and receives
//! its point/IMU streams.
//!
//! The lidar address comes from config, so there is no passive discovery
//! phase: the handshake commands `lidar_ip:cmd_port` directly with retries,
//! the same information the SDK2 search step would produce. Works unchanged
//! against a real sensor on its subnet and against virtual_mid360 on loopback.

use crate::pipeline::PacketSource;
use crate::wire::{
    self, build_param_set_body, host_ip_config_value, AsyncControlAck, ControlFrame, KeyValue,
};
use std::io;
use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::time::Duration;

/// The SDK2 port pair set, defaulting to the standard Mid-360 assignment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ports {
    pub cmd_data: u16,
    pub push_msg: u16,
    pub point_data: u16,
    pub imu_data: u16,
    pub log_data: u16,
    pub host_cmd_data: u16,
    pub host_push_msg: u16,
    pub host_point_data: u16,
    pub host_imu_data: u16,
    pub host_log_data: u16,
}

impl Default for Ports {
    fn default() -> Self {
        Ports {
            cmd_data: wire::LIDAR_CMD_PORT,
            push_msg: wire::LIDAR_PUSH_MSG_PORT,
            point_data: wire::LIDAR_POINT_PORT,
            imu_data: wire::LIDAR_IMU_PORT,
            log_data: wire::LIDAR_LOG_PORT,
            host_cmd_data: wire::HOST_CMD_PORT,
            host_push_msg: wire::HOST_PUSH_MSG_PORT,
            host_point_data: wire::HOST_POINT_PORT,
            host_imu_data: wire::HOST_IMU_PORT,
            host_log_data: wire::HOST_LOG_PORT,
        }
    }
}

#[derive(Debug, Clone)]
pub struct LiveConfig {
    pub host_ip: Ipv4Addr,
    pub lidar_ip: Ipv4Addr,
    /// Multicast group the device streams data to. `None` receives unicast
    /// only, the loopback/virtual arrangement.
    pub multicast_ip: Option<Ipv4Addr>,
    pub enable_imu: bool,
    pub ports: Ports,
}

const HANDSHAKE_RETRY: Duration = Duration::from_millis(500);
const HANDSHAKE_TIMEOUT: Duration = Duration::from_secs(60);
const HANDSHAKE_ATTEMPTS: u32 =
    (HANDSHAKE_TIMEOUT.as_millis() / HANDSHAKE_RETRY.as_millis()) as u32;
const RECV_POLL: Duration = Duration::from_millis(200);

/// Receives the point and IMU streams after driving the config handshake.
pub struct LiveSource {
    rx: mpsc::Receiver<Vec<u8>>,
    stop: Arc<AtomicBool>,
}

impl LiveSource {
    pub fn start(config: LiveConfig) -> io::Result<LiveSource> {
        let stop = Arc::new(AtomicBool::new(false));
        let (tx, rx) = mpsc::channel::<Vec<u8>>();

        let point = data_socket(&config, config.ports.host_point_data)?;
        spawn_reader("point", point, tx.clone(), stop.clone());
        if config.enable_imu {
            let imu = data_socket(&config, config.ports.host_imu_data)?;
            spawn_reader("imu", imu, tx, stop.clone());
        }

        let cmd = UdpSocket::bind(SocketAddrV4::new(
            config.host_ip,
            config.ports.host_cmd_data,
        ))?;
        cmd.set_read_timeout(Some(HANDSHAKE_RETRY))?;
        let handshake_stop = stop.clone();
        std::thread::spawn(move || run_handshake(&config, &cmd, &handshake_stop));

        Ok(LiveSource { rx, stop })
    }

    /// Shared flag that ends `recv` when set, e.g. from a shutdown signal.
    pub fn stop_flag(&self) -> Arc<AtomicBool> {
        self.stop.clone()
    }
}

impl PacketSource for LiveSource {
    fn recv(&mut self, buf: &mut [u8]) -> Option<usize> {
        loop {
            if self.stop.load(Ordering::Relaxed) {
                return None;
            }
            match self.rx.recv_timeout(RECV_POLL) {
                Ok(packet) => {
                    let len = packet.len().min(buf.len());
                    buf[..len].copy_from_slice(&packet[..len]);
                    return Some(len);
                }
                Err(mpsc::RecvTimeoutError::Timeout) => continue,
                Err(mpsc::RecvTimeoutError::Disconnected) => return None,
            }
        }
    }
}

impl Drop for LiveSource {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

/// Bind a data-plane receive socket, joining the multicast group when the
/// device streams to one.
fn data_socket(config: &LiveConfig, port: u16) -> io::Result<UdpSocket> {
    let socket = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, port))?;
    if let Some(group) = config.multicast_ip {
        socket.join_multicast_v4(&group, &config.host_ip)?;
    }
    socket.set_read_timeout(Some(RECV_POLL))?;
    Ok(socket)
}

fn spawn_reader(
    label: &'static str,
    socket: UdpSocket,
    tx: mpsc::Sender<Vec<u8>>,
    stop: Arc<AtomicBool>,
) {
    std::thread::spawn(move || {
        let mut buf = [0u8; 4096];
        while !stop.load(Ordering::Relaxed) {
            match socket.recv_from(&mut buf) {
                Ok((len, _)) => {
                    if tx.send(buf[..len].to_vec()).is_err() {
                        return;
                    }
                }
                Err(err)
                    if err.kind() == io::ErrorKind::WouldBlock
                        || err.kind() == io::ErrorKind::TimedOut =>
                {
                    continue
                }
                Err(err) => {
                    tracing::warn!("{label} socket recv failed: {err}");
                    return;
                }
            }
        }
    });
}

/// One param-set step of the handshake: what to send and how to log it.
struct Step {
    label: &'static str,
    key: u16,
    value: Vec<u8>,
}

fn handshake_steps(config: &LiveConfig) -> Vec<Step> {
    let mut steps = vec![
        Step {
            label: "point host cfg",
            key: wire::param_key::POINT_DATA_HOST_IP_CFG,
            value: host_ip_config_value(
                config.host_ip,
                config.ports.host_point_data,
                config.ports.point_data,
            )
            .to_vec(),
        },
        Step {
            label: "imu host cfg",
            key: wire::param_key::IMU_HOST_IP_CFG,
            value: host_ip_config_value(
                config.host_ip,
                config.ports.host_imu_data,
                config.ports.imu_data,
            )
            .to_vec(),
        },
    ];
    if config.enable_imu {
        steps.push(Step {
            label: "imu enable",
            key: wire::param_key::IMU_DATA_EN,
            value: vec![1],
        });
    }
    steps.push(Step {
        label: "work mode normal",
        key: wire::param_key::WORK_MODE,
        value: vec![wire::WORK_MODE_NORMAL],
    });
    steps
}

/// Send each config step until the device ACKs it, matching the SDK's
/// one-key-per-request behavior. The work-mode step last starts streaming.
fn run_handshake(config: &LiveConfig, cmd: &UdpSocket, stop: &AtomicBool) {
    let device = SocketAddrV4::new(config.lidar_ip, config.ports.cmd_data);
    let mut seq: u32 = 0;
    for step in handshake_steps(config) {
        let mut acked = false;
        for _ in 0..HANDSHAKE_ATTEMPTS {
            if stop.load(Ordering::Relaxed) {
                return;
            }
            seq = seq.wrapping_add(1);
            let body = build_param_set_body(&[KeyValue {
                key: step.key,
                value: &step.value,
            }]);
            let request = wire::build_control(
                seq,
                wire::cmd_id::PARAM_SET,
                wire::CMD_TYPE_REQUEST,
                wire::SENDER_HOST,
                &body,
            );
            if let Err(err) = cmd.send_to(&request, device) {
                tracing::warn!("control send to {device} failed: {err}");
                continue;
            }
            if wait_for_ack(cmd, seq) {
                tracing::info!(step = step.label, "handshake step acked");
                acked = true;
                break;
            }
        }
        if !acked {
            tracing::error!(
                step = step.label,
                lidar = %config.lidar_ip,
                "device did not ack within {HANDSHAKE_ATTEMPTS} attempts"
            );
            return;
        }
    }
    tracing::info!(lidar = %config.lidar_ip, "mid360 configured and streaming");
}

/// Wait one retry interval for the ACK matching `seq`.
fn wait_for_ack(cmd: &UdpSocket, seq: u32) -> bool {
    let mut buf = [0u8; 2048];
    while let Ok((len, _)) = cmd.recv_from(&mut buf) {
        let Ok(frame) = ControlFrame::parse(&buf[..len]) else {
            continue;
        };
        if frame.cmd_type != wire::CMD_TYPE_ACK || frame.seq != seq {
            continue;
        }
        match AsyncControlAck::parse(frame.data) {
            Ok(ack) if ack.ret_code == 0 => return true,
            Ok(ack) => {
                tracing::warn!(ret_code = ack.ret_code, "device rejected config step");
                return false;
            }
            Err(_) => continue,
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::{build_imu_samples, build_points_high, DataPacket, DataType, ImuSample};
    use std::collections::HashSet;

    fn test_ports(base: u16) -> Ports {
        Ports {
            cmd_data: base,
            push_msg: base + 1,
            point_data: base + 2,
            imu_data: base + 3,
            log_data: base + 4,
            host_cmd_data: base + 5,
            host_push_msg: base + 6,
            host_point_data: base + 7,
            host_imu_data: base + 8,
            host_log_data: base + 9,
        }
    }

    /// A minimal in-test device: ACK every param-set, then stream one point
    /// packet and one IMU packet once work mode is set.
    fn spawn_fake_device(ports: Ports) -> std::thread::JoinHandle<Vec<u16>> {
        std::thread::spawn(move || {
            let loopback = Ipv4Addr::LOCALHOST;
            let cmd = UdpSocket::bind(SocketAddrV4::new(loopback, ports.cmd_data)).unwrap();
            cmd.set_read_timeout(Some(Duration::from_secs(10))).unwrap();
            let mut keys_seen = Vec::new();
            let mut buf = [0u8; 2048];
            loop {
                let (len, from) = cmd.recv_from(&mut buf).unwrap();
                let frame = ControlFrame::parse(&buf[..len]).unwrap();
                let params = wire::parse_param_set_body(frame.data).unwrap();
                let key = params[0].key;
                if keys_seen.last() != Some(&key) {
                    keys_seen.push(key);
                }
                let ack_body = AsyncControlAck {
                    ret_code: 0,
                    error_key: 0,
                }
                .build();
                let ack = wire::build_control(
                    frame.seq,
                    frame.cmd_id,
                    wire::CMD_TYPE_ACK,
                    wire::SENDER_LIDAR,
                    &ack_body,
                );
                cmd.send_to(&ack, from).unwrap();
                if key == wire::param_key::WORK_MODE {
                    break;
                }
            }

            let point_payload = build_points_high(&[crate::wire::PointHigh {
                x_mm: 1000,
                y_mm: 0,
                z_mm: 0,
                reflectivity: 128,
                tag: 0,
            }]);
            let point_packet = DataPacket {
                time_interval: 100,
                dot_num: 1,
                udp_cnt: 0,
                frame_cnt: 0,
                data_type: DataType::CartesianHigh,
                time_type: 0,
                timestamp_ns: 1_000,
                payload: &point_payload,
            }
            .build();
            let point_socket =
                UdpSocket::bind(SocketAddrV4::new(loopback, ports.point_data)).unwrap();
            point_socket
                .send_to(
                    &point_packet,
                    SocketAddrV4::new(loopback, ports.host_point_data),
                )
                .unwrap();

            let imu_payload = build_imu_samples(&[ImuSample {
                gyro: [0.0; 3],
                acc_g: [0.0, 0.0, 1.0],
            }]);
            let imu_packet = DataPacket {
                time_interval: 0,
                dot_num: 1,
                udp_cnt: 0,
                frame_cnt: 0,
                data_type: DataType::Imu,
                time_type: 0,
                timestamp_ns: 2_000,
                payload: &imu_payload,
            }
            .build();
            let imu_socket = UdpSocket::bind(SocketAddrV4::new(loopback, ports.imu_data)).unwrap();
            imu_socket
                .send_to(
                    &imu_packet,
                    SocketAddrV4::new(loopback, ports.host_imu_data),
                )
                .unwrap();

            keys_seen
        })
    }

    #[test]
    fn handshake_and_stream_over_loopback() {
        let ports = test_ports(47600);
        let device = spawn_fake_device(ports);

        let mut source = LiveSource::start(LiveConfig {
            host_ip: Ipv4Addr::LOCALHOST,
            lidar_ip: Ipv4Addr::LOCALHOST,
            multicast_ip: None,
            enable_imu: true,
            ports,
        })
        .unwrap();

        let mut buf = [0u8; 4096];
        let mut types_seen = HashSet::new();
        for _ in 0..2 {
            let len = source.recv(&mut buf).expect("packet before shutdown");
            let packet = DataPacket::parse(&buf[..len]).unwrap();
            types_seen.insert(packet.data_type);
        }
        assert!(types_seen.contains(&DataType::CartesianHigh));
        assert!(types_seen.contains(&DataType::Imu));

        let keys = device.join().unwrap();
        assert_eq!(
            keys,
            vec![
                wire::param_key::POINT_DATA_HOST_IP_CFG,
                wire::param_key::IMU_HOST_IP_CFG,
                wire::param_key::IMU_DATA_EN,
                wire::param_key::WORK_MODE,
            ]
        );
    }

    #[test]
    fn recv_ends_on_stop() {
        let ports = test_ports(47700);
        let mut source = LiveSource::start(LiveConfig {
            host_ip: Ipv4Addr::LOCALHOST,
            lidar_ip: Ipv4Addr::LOCALHOST,
            multicast_ip: None,
            enable_imu: false,
            ports,
        })
        .unwrap();
        source.stop_flag().store(true, Ordering::Relaxed);
        let mut buf = [0u8; 16];
        assert_eq!(source.recv(&mut buf), None);
    }
}
