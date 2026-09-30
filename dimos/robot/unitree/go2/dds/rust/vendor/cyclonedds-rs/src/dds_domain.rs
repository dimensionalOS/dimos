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

use cyclonedds_sys::{dds_error::DDSError, DdsDomainId, DdsEntity};
use std::convert::From;
use std::ffi::CString;

pub struct DdsDomain(DdsEntity);

impl DdsDomain {
    ///Create a domain with a specified domain id
    pub fn create(domain: DdsDomainId, config: Option<&str>) -> Result<Self, DDSError> {
        unsafe {
            if let Some(cfg) = config {
                let domain_name = CString::new(cfg).expect("Unable to create new config string");
                let d = cyclonedds_sys::dds_create_domain(domain, domain_name.as_ptr());
                // negative return value signify an error
                if d > 0 {
                    Ok(DdsDomain(DdsEntity::new(d)))
                } else {
                    Err(DDSError::from(d))
                }
            } else {
                let d = cyclonedds_sys::dds_create_domain(domain, std::ptr::null());

                if d > 0 {
                    Ok(DdsDomain(DdsEntity::new(d)))
                } else {
                    Err(DDSError::from(d))
                }
            }
        }
    }
}

impl PartialEq for DdsDomain {
    fn eq(&self, other: &Self) -> bool {
        unsafe { self.0.entity() == other.0.entity() }
    }
}

impl Eq for DdsDomain {}

impl Drop for DdsDomain {
    fn drop(&mut self) {
        unsafe {
            let ret: DDSError = cyclonedds_sys::dds_delete(self.0.entity()).into();
            if DDSError::DdsOk != ret {
                panic!("cannot delete domain: {}", ret);
            }
        }
    }
}

#[cfg(test)]
mod dds_domain_tests {
    use crate::dds_domain::DdsDomain;
    use cyclonedds_sys::DDSError;

    #[test]
    fn test_create_domain_with_bad_config() {
        assert!(Err(DDSError::DdsOk) != DdsDomain::create(0, Some("blah")));
    }
}
