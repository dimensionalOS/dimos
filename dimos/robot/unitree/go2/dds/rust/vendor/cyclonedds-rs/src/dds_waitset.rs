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

use crate::{DdsParticipant, Entity};
use cyclonedds_sys::size_t;
pub use cyclonedds_sys::{DDSError, DdsDomainId, DdsEntity};
use std::convert::From;
use std::marker::PhantomData;

pub struct DdsWaitset<T>(DdsEntity, PhantomData<T>);

impl<'a, T> DdsWaitset<T> {
    pub fn create(participant: &DdsParticipant) -> Result<Self, DDSError> {
        unsafe {
            let p = cyclonedds_sys::dds_create_waitset(participant.entity().entity());
            if p >= 0 {
                Ok(DdsWaitset(DdsEntity::new(p), PhantomData))
            } else {
                Err(DDSError::from(p))
            }
        }
    }

    pub fn attach(&mut self, entity: &dyn Entity, x: &'a T) -> Result<(), DDSError> {
        unsafe {
            let p = cyclonedds_sys::dds_waitset_attach(
                self.0.entity(),
                entity.entity().entity(),
                x as *const T as isize,
            );
            if p == 0 {
                Ok(())
            } else {
                Err(DDSError::from(p))
            }
        }
    }
    pub fn detach(&mut self, entity: &dyn Entity) -> Result<(), DDSError> {
        unsafe {
            let p = cyclonedds_sys::dds_waitset_detach(self.0.entity(), entity.entity().entity());
            if p == 0 {
                Ok(())
            } else {
                Err(DDSError::from(p))
            }
        }
    }
    pub fn set_trigger(&mut self, trigger: bool) -> Result<(), DDSError> {
        unsafe {
            let p = cyclonedds_sys::dds_waitset_set_trigger(self.0.entity(), trigger);
            if p == 0 {
                Ok(())
            } else {
                Err(DDSError::from(p))
            }
        }
    }
    pub fn wait<'b>(
        &mut self,
        xs: &'b mut Vec<&'b T>,
        timeout_us: i64,
    ) -> Result<&'b [&'b T], DDSError> {
        let capacity = xs.capacity();
        unsafe {
            let p = cyclonedds_sys::dds_waitset_wait(
                self.0.entity(),
                xs.as_mut_ptr() as *mut isize,
                capacity as size_t,
                timeout_us,
            );
            if p == 0 {
                // timeout, empty slice back
                Ok(&xs[0..0])
            } else if p > 0 {
                let p = p as usize;
                xs.set_len(p);
                Ok(&xs[0..p])
            } else {
                Err(DDSError::from(p))
            }
        }
    }
}

impl<T> Entity for DdsWaitset<T> {
    fn entity(&self) -> &DdsEntity {
        &self.0
    }
}

impl<T> Drop for DdsWaitset<T> {
    fn drop(&mut self) {
        unsafe {
            let _ret: DDSError = cyclonedds_sys::dds_delete(self.0.entity()).into();
            //if DDSError::DdsOk != ret {
            //    //we ignore the error here as the waitset may be deleted by cyclone
            //    //panic!("cannot delete DdsWaitset: {}", ret);
            //} else {
            //
            //}
        }
    }
}
