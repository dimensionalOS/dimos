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

#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use bitmask::bitmask;

include!(concat!(env!("OUT_DIR"), "/generated.rs"));

pub mod dds_error;
pub use dds_error::DDSError;

//// some macros we need to use in Rust
pub const DDS_FREE_KEY_BIT: u32 = 0x01;
pub const DDS_FREE_CONTENTS_BIT: u32 = 0x02;
pub const DDS_FREE_ALL_BIT: u32 = 0x04;

#[derive(Clone, PartialEq)]
pub struct DdsEntity(dds_entity_t);

impl DdsEntity {
    pub unsafe fn new(entity: dds_entity_t) -> Self {
        DdsEntity(entity)
    }
    pub unsafe fn entity(&self) -> dds_entity_t {
        self.0
    }
}

pub mod builtin_entity {
    use crate::DdsEntity;
    pub const BUILTIN_TOPIC_DCPSPARTICIPANT_ENTITY: DdsEntity =
        DdsEntity(crate::BUILTIN_TOPIC_DCPSPARTICIPANT);
    pub const BUILTIN_TOPIC_DCPSTOPIC_ENTITY: DdsEntity = DdsEntity(crate::BUILTIN_TOPIC_DCPSTOPIC);
    pub const BUILTIN_TOPIC_DCPSPUBLICATION_ENTITY: DdsEntity =
        DdsEntity(crate::BUILTIN_TOPIC_DCPSPUBLICATION);
    pub const BUILTIN_TOPIC_DCPSSUBSCRIPTION: DdsEntity =
        DdsEntity(crate::BUILTIN_TOPIC_DCPSSUBSCRIPTION);
}

pub type DdsDomainId = dds_domainid_t;
pub type DdsTopicDescriptor = dds_topic_descriptor_t;

bitmask! {
    pub mask StateMask : u32 where flags State {
        DdsReadSampleState = 0x1,
        DdsNotReadSampleState = 0x2,
        DdsAnySampleState = 0x1 | 0x2,
        DdsNewViewState = 0x4,
        DdsNotNewViewState = 0x8,
        DdsAnyViewState = 0x4 | 0x8,
        DdsAliveInstanceState = 16,
        DdsNotAliveDisposedInstanceState = 32,
        DdsNotAliveNoWritersInstanceState = 64,
        DdsAnyInstanceState = 16 | 32 | 64,
        DdsAnyState =  1 | 2  | 4 | 8 | 16 | 32 | 64,
    }
}
