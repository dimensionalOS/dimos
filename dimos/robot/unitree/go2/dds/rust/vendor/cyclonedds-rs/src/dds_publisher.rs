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

use crate::{DdsListener, DdsParticipant, DdsQos, DdsWritable};
pub use cyclonedds_sys::{DDSError, DdsDomainId, DdsEntity};
use std::convert::From;

pub struct PublisherBuilder {
    maybe_qos: Option<DdsQos>,
    maybe_listener: Option<DdsListener>,
}

impl PublisherBuilder {
    pub fn new() -> Self {
        Self {
            maybe_qos: None,
            maybe_listener: None,
        }
    }

    pub fn with_qos(mut self, qos: DdsQos) -> Self {
        self.maybe_qos = Some(qos);
        self
    }

    pub fn with_listener(mut self, listener: DdsListener) -> Self {
        self.maybe_listener = Some(listener);
        self
    }

    pub fn create(self, participant: &DdsParticipant) -> Result<DdsPublisher, DDSError> {
        DdsPublisher::create(participant, self.maybe_qos, self.maybe_listener)
    }
}

#[derive(Clone)]
pub struct DdsPublisher(DdsEntity, Option<DdsListener>);

impl<'a> DdsPublisher {
    pub fn create(
        participant: &DdsParticipant,
        maybe_qos: Option<DdsQos>,
        maybe_listener: Option<DdsListener>,
    ) -> Result<Self, DDSError> {
        unsafe {
            let p = cyclonedds_sys::dds_create_publisher(
                participant.entity().entity(),
                maybe_qos.map_or(std::ptr::null(), |d| d.into()),
                maybe_listener
                    .as_ref()
                    .map_or(std::ptr::null(), |l| l.into()),
            );
            if p > 0 {
                Ok(DdsPublisher(DdsEntity::new(p), maybe_listener))
            } else {
                Err(DDSError::from(p))
            }
        }
    }
}

impl<'a> DdsWritable for DdsPublisher {
    fn entity(&self) -> &DdsEntity {
        &self.0
    }
}
