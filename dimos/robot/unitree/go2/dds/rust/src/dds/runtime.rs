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

//! Participant + reader/writer construction for the Go2.

use cyclonedds_rs::{
    dds_history_kind, dds_reliability_kind, DdsParticipant, DdsQos, DdsReader, DdsTopic, DdsWriter,
    TopicType,
};
use std::time::{Duration, Instant};

/// A boot-time interface race is worth retrying through; an off-robot run is not.
const PARTICIPANT_RETRY_FOR: Duration = Duration::from_secs(30);

/// The participant, pinned to `iface` unless `CYCLONEDDS_URI` is already set.
pub fn participant(iface: &str, domain_id: u32) -> DdsParticipant {
    if std::env::var_os("CYCLONEDDS_URI").is_none() {
        let xml = format!(
            "<CycloneDDS xmlns=\"https://cdds.io/config\"><Domain Id=\"any\"><General>\
             <Interfaces><NetworkInterface name=\"{iface}\"/></Interfaces>\
             <AllowMulticast>default</AllowMulticast></General></Domain></CycloneDDS>"
        );
        std::env::set_var("CYCLONEDDS_URI", xml);
    }
    let deadline = Instant::now() + PARTICIPANT_RETRY_FOR;
    let mut logged = false;
    loop {
        match DdsParticipant::create(Some(domain_id), None, None) {
            Ok(p) => return p,
            Err(e) if Instant::now() >= deadline => {
                panic!("create DDS participant on domain {domain_id}: {e:?}")
            }
            Err(e) => {
                if !logged {
                    tracing::warn!(?e, iface, "participant create failed, retrying");
                    logged = true;
                }
                std::thread::sleep(Duration::from_millis(500));
            }
        }
    }
}

/// Best-effort + KEEP_LAST: matches both the reliable and best-effort writers on the Go2.
fn reader_qos(depth: i32) -> DdsQos {
    let mut q = DdsQos::create().expect("qos");
    q.set_history(dds_history_kind::DDS_HISTORY_KEEP_LAST, depth);
    q
}

/// Reliable + KEEP_LAST, what the Go2's request readers expect.
fn writer_qos(depth: i32) -> DdsQos {
    let mut q = DdsQos::create().expect("qos");
    q.set_history(dds_history_kind::DDS_HISTORY_KEEP_LAST, depth);
    q.set_reliability(
        dds_reliability_kind::DDS_RELIABILITY_RELIABLE,
        Duration::from_millis(100),
    );
    q
}

pub fn make_reader<T: TopicType + 'static>(p: &DdsParticipant, topic_name: &str) -> DdsReader<T> {
    let topic = DdsTopic::<T>::create(p, topic_name, None, None).expect("create topic (reader)");
    DdsReader::create(p, topic, Some(reader_qos(64)), None).expect("create reader")
}

pub fn make_writer<T: TopicType + 'static>(p: &DdsParticipant, topic_name: &str) -> DdsWriter<T> {
    let topic = DdsTopic::<T>::create(p, topic_name, None, None).expect("create topic (writer)");
    DdsWriter::create(p, topic, Some(writer_qos(16)), None).expect("create writer")
}
