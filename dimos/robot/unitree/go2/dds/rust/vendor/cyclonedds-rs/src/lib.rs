/*
    Copyright 2020 Sojan James

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

//! Safe Rust bindings to cyclonedds
//!
//!

pub mod alloc;
mod common;
pub mod dds_api;
pub mod dds_domain;
pub mod dds_listener;
pub mod dds_participant;
pub mod dds_publisher;
pub mod dds_qos;
pub mod dds_reader;
pub mod dds_subscriber;
pub mod dds_topic;
mod dds_waitset;
pub mod dds_writer;
pub mod error;
pub mod serdes;

pub use common::{DdsReadable, DdsWritable, Entity};
pub use dds_api::*;
pub use dds_listener::{DdsListener, DdsListenerBuilder};
pub use dds_participant::{DdsParticipant, ParticipantBuilder};
pub use dds_publisher::{DdsPublisher, PublisherBuilder};
pub use dds_qos::*;
pub use dds_reader::{DdsReadCondition, DdsReader, ReaderBuilder};
pub use dds_subscriber::{DdsSubscriber, SubscriberBuilder};
pub use dds_topic::{DdsTopic, TopicBuilder};
pub use dds_waitset::DdsWaitset;
pub use dds_writer::{DdsWriter, WriterBuilder};
pub use serdes::{Sample, SampleBuffer, TopicType};

pub use cdr;
pub use cyclonedds_sys::dds_error::DDSError;

pub use serde_derive::{Deserialize, Serialize};
