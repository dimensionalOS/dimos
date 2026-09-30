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

use std::convert::From;

use crate::common::Entity;
pub use cyclonedds_sys::dds_error::DDSError;
use cyclonedds_sys::DdsEntity;

//use crate::dds_writer::DdsWriter;
pub use cyclonedds_sys::dds_status_id;
pub use cyclonedds_sys::{dds_attach_t, dds_duration_t};

// re-export constants
pub use cyclonedds_sys::dds_status_id_DDS_DATA_AVAILABLE_STATUS_ID as DDS_DATA_AVAILABLE_STATUS_ID;
pub use cyclonedds_sys::dds_status_id_DDS_DATA_ON_READERS_STATUS_ID as DDS_DATA_ON_READERS_STATUS_ID;
pub use cyclonedds_sys::dds_status_id_DDS_INCONSISTENT_TOPIC_STATUS_ID as DDS_INCONSISTENT_TOPIC_STATUS_ID;
pub use cyclonedds_sys::dds_status_id_DDS_LIVELINESS_CHANGED_STATUS_ID as DDS_LIVELINESS_CHANGED_STATUS_ID;
pub use cyclonedds_sys::dds_status_id_DDS_LIVELINESS_LOST_STATUS_ID as DDS_LIVELINESS_LOST_STATUS_ID;
pub use cyclonedds_sys::dds_status_id_DDS_OFFERED_DEADLINE_MISSED_STATUS_ID as DDS_OFFERED_DEADLINE_MISSED_STATUS_ID;
pub use cyclonedds_sys::dds_status_id_DDS_OFFERED_INCOMPATIBLE_QOS_STATUS_ID as DDS_OFFERED_INCOMPATIBLE_QOS_STATUS_ID;
pub use cyclonedds_sys::dds_status_id_DDS_PUBLICATION_MATCHED_STATUS_ID as DDS_PUBLICATION_MATCHED_STATUS_ID;
pub use cyclonedds_sys::dds_status_id_DDS_REQUESTED_DEADLINE_MISSED_STATUS_ID as DDS_REQUESTED_DEADLINE_MISSED_STATUS_ID;
pub use cyclonedds_sys::dds_status_id_DDS_REQUESTED_INCOMPATIBLE_QOS_STATUS_ID as DDS_REQUESTED_INCOMPATIBLE_QOS_STATUS_ID;
pub use cyclonedds_sys::dds_status_id_DDS_SAMPLE_LOST_STATUS_ID as DDS_SAMPLE_LOST_STATUS_ID;
pub use cyclonedds_sys::dds_status_id_DDS_SAMPLE_REJECTED_STATUS_ID as DDS_SAMPLE_REJECTED_STATUS_ID;
pub use cyclonedds_sys::dds_status_id_DDS_SUBSCRIPTION_MATCHED_STATUS_ID as DDS_SUBSCRIPTION_MATCHED_STATUS_ID;
pub use cyclonedds_sys::State;
pub use cyclonedds_sys::StateMask;

#[derive(Default)]
pub struct DdsStatus(u32);

impl DdsStatus {
    pub fn set(mut self, id: dds_status_id) -> Self {
        let mask = 1 << id;
        self.0 |= mask;
        self
    }

    pub fn is_set(&self, id: dds_status_id) -> bool {
        let mask = 1 << id;
        mask & self.0 != 0
    }
}

impl From<DdsStatus> for u32 {
    fn from(status: DdsStatus) -> Self {
        status.0
    }
}

pub fn dds_set_status_mask(entity: &DdsEntity, status_mask: DdsStatus) -> Result<(), DDSError> {
    unsafe {
        let err = cyclonedds_sys::dds_set_status_mask(entity.entity(), status_mask.into());

        if err < 0 {
            Err(DDSError::from(err))
        } else {
            Ok(())
        }
    }
}

pub fn dds_get_status_changes(entity: &DdsEntity) -> Result<DdsStatus, DDSError> {
    unsafe {
        let mut status = DdsStatus::default();
        let err = cyclonedds_sys::dds_get_status_changes(entity.entity(), &mut status.0);

        if err < 0 {
            Err(DDSError::from(err))
        } else {
            Ok(status)
        }
    }
}

pub fn dds_triggered(entity: &dyn Entity) -> Result<(), DDSError> {
    unsafe {
        let err = cyclonedds_sys::dds_triggered(entity.entity().entity());

        if err < 0 {
            Err(DDSError::from(err))
        } else {
            Ok(())
        }
    }
}

#[cfg(test)]
mod dds_qos_tests {
    use super::*;
    #[test]
    fn test_dds_status() {
        let status = DdsStatus::default();
        assert_eq!(false, status.is_set(0));
        let status = status
            .set(DDS_INCONSISTENT_TOPIC_STATUS_ID)
            .set(DDS_OFFERED_DEADLINE_MISSED_STATUS_ID)
            .set(DDS_SUBSCRIPTION_MATCHED_STATUS_ID);

        assert_eq!(true, status.is_set(DDS_INCONSISTENT_TOPIC_STATUS_ID));
        assert_eq!(true, status.is_set(DDS_OFFERED_DEADLINE_MISSED_STATUS_ID));
        assert_eq!(true, status.is_set(DDS_SUBSCRIPTION_MATCHED_STATUS_ID));
        assert_eq!(false, status.is_set(DDS_SAMPLE_REJECTED_STATUS_ID));
    }
}
