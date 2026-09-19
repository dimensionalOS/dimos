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

use cyclonedds_sys::*;
use std::convert::From;
use std::ffi::c_void;
use std::ptr::NonNull;

use crate::SampleBuffer;
pub use cyclonedds_sys::DdsEntity;
use std::marker::PhantomData;

use crate::serdes::{Sample, TopicType};
use crate::{dds_listener::DdsListener, dds_qos::DdsQos, dds_topic::DdsTopic, DdsWritable, Entity};

pub struct WriterBuilder<T: TopicType> {
    maybe_qos: Option<DdsQos>,
    maybe_listener: Option<DdsListener>,
    phantom: PhantomData<T>,
}

impl<T> WriterBuilder<T>
where
    T: TopicType,
{
    pub fn new() -> Self {
        Self {
            maybe_qos: None,
            maybe_listener: None,
            phantom: PhantomData,
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

    pub fn create(
        self,
        entity: &dyn DdsWritable,
        topic: DdsTopic<T>,
    ) -> Result<DdsWriter<T>, DDSError> {
        DdsWriter::create(entity, topic, self.maybe_qos, self.maybe_listener)
    }
}

pub enum LoanedInner<T: Sized + TopicType> {
    Uninitialized(NonNull<T>, DdsEntity),
    Initialized(NonNull<T>, DdsEntity),
    Empty,
}

pub struct Loaned<T: Sized + TopicType> {
    inner: LoanedInner<T>,
}

impl<T> Loaned<T>
where
    T: Sized + TopicType,
{
    pub fn as_mut_ptr(&mut self) -> Option<*mut T> {
        match self.inner {
            LoanedInner::Uninitialized(p, _) => Some(p.as_ptr()),
            LoanedInner::Initialized(p, _) => Some(p.as_ptr()),
            LoanedInner::Empty => None,
        }
    }

    pub fn assume_init(mut self) -> Self {
        match &mut self.inner {
            LoanedInner::Uninitialized(p, e) => Self {
                inner: LoanedInner::Initialized(*p, e.clone()),
            },
            LoanedInner::Initialized(p, e) => Self {
                inner: LoanedInner::Initialized(*p, e.clone()),
            },
            LoanedInner::Empty => Self {
                inner: LoanedInner::Empty,
            },
        }
    }
}

impl<T> Drop for Loaned<T>
where
    T: Sized + TopicType,
{
    fn drop(&mut self) {
        let (mut p_sample, entity) = match &mut self.inner {
            LoanedInner::Uninitialized(p, entity) => (p.as_ptr(), Some(entity)),
            LoanedInner::Initialized(p, entity) => (p.as_ptr(), Some(entity)),
            LoanedInner::Empty => (std::ptr::null_mut(), None),
        };

        if let Some(entity) = entity {
            let voidpp: *mut *mut T = &mut p_sample;
            let voidpp = voidpp as *mut *mut c_void;
            unsafe { dds_return_loan(entity.entity(), voidpp, 1) };
        }
    }
}

#[derive(Clone)]
pub struct DdsWriter<T: Sized + TopicType>(DdsEntity, Option<DdsListener>, PhantomData<T>);

impl<'a, T> DdsWriter<T>
where
    T: Sized + TopicType,
{
    pub fn create(
        entity: &dyn DdsWritable,
        topic: DdsTopic<T>,
        maybe_qos: Option<DdsQos>,
        maybe_listener: Option<DdsListener>,
    ) -> Result<Self, DDSError> {
        unsafe {
            let w = dds_create_writer(
                entity.entity().entity(),
                topic.entity().entity(),
                maybe_qos.map_or(std::ptr::null(), |q| q.into()),
                maybe_listener
                    .as_ref()
                    .map_or(std::ptr::null(), |l| l.into()),
            );

            if w >= 0 {
                Ok(DdsWriter(DdsEntity::new(w), maybe_listener, PhantomData))
            } else {
                Err(DDSError::from(w))
            }
        }
    }

    pub fn write_to_entity(entity: &DdsEntity, msg: std::sync::Arc<T>) -> Result<(), DDSError> {
        unsafe {
            let sample = Sample::<T>::from(msg);
            let sample = &sample as *const Sample<T>;
            let sample = sample as *const ::std::os::raw::c_void;
            let ret = dds_write(entity.entity(), sample);
            if ret >= 0 {
                Ok(())
            } else {
                Err(DDSError::from(ret))
            }
        }
    }

    pub fn write(&mut self, msg: std::sync::Arc<T>) -> Result<(), DDSError> {
        Self::write_to_entity(&self.0, msg)
    }

    // Loan memory buffers for zero copy operation. Only supported for fixed size types
    #[cfg(feature = "shm")]
    pub fn loan(&mut self) -> Result<Loaned<T>, DDSError> {
        if !T::is_fixed_size() {
            // Loaning is not supported for types that are not fixed size
            return Err(DDSError::Unsupported);
        }

        let mut p_sample: *mut T = std::ptr::null_mut();
        let voidpp: *mut *mut T = &mut p_sample;
        let voidpp = voidpp as *mut *mut c_void;
        let res = unsafe { dds_loan_sample(self.0.entity(), voidpp) };
        if res == 0 {
            Ok(Loaned {
                inner: LoanedInner::Uninitialized(
                    NonNull::new(p_sample).unwrap(),
                    self.entity().clone(),
                ),
            })
        } else {
            Err(DDSError::from(res))
        }
    }

    // Return the loaned buffer.  If the buffer was initialized, then write the data to be published
    pub fn return_loan(&mut self, mut buffer: Loaned<T>) -> Result<(), DDSError> {
        let res = match &mut buffer.inner {
            LoanedInner::Uninitialized(p, entity) => {
                let mut p_sample = p.as_ptr();
                let voidpp: *mut *mut T = &mut p_sample;
                let voidpp = voidpp as *mut *mut c_void;
                unsafe { dds_return_loan(entity.entity(), voidpp, 1) }
            }
            LoanedInner::Initialized(p, entity) => {
                let p_sample = p.as_ptr();
                unsafe { dds_write(entity.entity(), p_sample as *const c_void) }
            }
            LoanedInner::Empty => 0,
        };

        if res == 0 {
            Ok(())
        } else {
            Err(DDSError::from(res))
        }
    }

    pub fn set_listener(&mut self, listener: DdsListener) -> Result<(), DDSError> {
        unsafe {
            let refl = &listener;
            let rc = dds_set_listener(self.0.entity(), refl.into());
            if rc == 0 {
                self.1 = Some(listener);
                Ok(())
            } else {
                Err(DDSError::from(rc))
            }
        }
    }
}

impl<'a, T> Entity for DdsWriter<T>
where
    T: std::marker::Sized + TopicType,
{
    fn entity(&self) -> &DdsEntity {
        &self.0
    }
}

impl<'a, T> Drop for DdsWriter<T>
where
    T: std::marker::Sized + TopicType,
{
    fn drop(&mut self) {
        unsafe {
            let ret: DDSError = cyclonedds_sys::dds_delete(self.0.entity()).into();
            if DDSError::DdsOk != ret && DDSError::AlreadyDeleted != ret {
                //panic!("cannot delete Writer: {}", ret);
            }
        }
    }
}

#[cfg(test)]
mod test {
    use core::panic;
    use std::{ops::Deref, sync::Arc, time::Duration};

    use super::*;
    use crate::{DdsParticipant, DdsReader, DdsSubscriber};
    use crate::{DdsPublisher, DdsWriter};

    use cdds_derive::{Topic, TopicFixedSize};
    use serde_derive::{Deserialize, Serialize};
    use tokio::runtime::Runtime;

    const cyclone_shm_config: &str = r###"<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS xmlns="https://cdds.io/config"
                xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/iceoryx/etc/cyclonedds.xsd">
        <Domain id="any">
            <SharedMemory>
                <Enable>true</Enable>
                <LogLevel>info</LogLevel>
            </SharedMemory>
        </Domain>
    </CycloneDDS>"###;

    #[repr(C)]
    #[derive(Serialize, Deserialize, Debug, PartialEq, Clone)]
    enum Position {
        Front,
        Back,
    }

    impl Default for Position {
        fn default() -> Self {
            Self::Front
        }
    }

    #[derive(Serialize, Deserialize, TopicFixedSize, Debug, PartialEq)]
    struct TestTopic {
        a: u32,
        b: u16,
        c: [u8; 10],
        d: [u8; 15],
        #[topic_key]
        e: u32,
        #[topic_key_enum]
        pos: Position,
    }

    impl Default for TestTopic {
        fn default() -> Self {
            Self {
                a: 10,
                b: 20,
                c: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                d: [1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5],
                e: 0,
                pos: Position::default(),
            }
        }
    }

    #[derive(Serialize, Deserialize, Topic, Debug, PartialEq)]
    struct AnotherTopic {
        pub value: u32,
        pub name: String,
        pub arr: [String; 2],
        pub vec: Vec<String>,
        #[topic_key]
        pub key: u32,
    }

    impl Default for AnotherTopic {
        fn default() -> Self {
            assert!(Self::has_key());
            Self {
                value: 42,
                name: "the answer".to_owned(),
                arr: ["one".to_owned(), "two".to_owned()],
                vec: vec!["Hello".to_owned(), "world".to_owned()],
                key: 0,
            }
        }
    }

    //#[test]
    fn test_loan() {
        // Make sure iox-roudi is running
        std::env::set_var("CYCLONEDDS_URI", cyclone_shm_config);

        let participant = DdsParticipant::create(None, None, None).unwrap();

        let topic = TestTopic::create_topic(&participant, Some("test_topic"), None, None).unwrap();
        let another_topic = AnotherTopic::create_topic(&participant, None, None, None).unwrap();

        let publisher = DdsPublisher::create(&participant, None, None).unwrap();

        let mut writer = DdsWriter::create(&publisher, topic.clone(), None, None).unwrap();
        let mut another_writer =
            DdsWriter::create(&publisher, another_topic.clone(), None, None).unwrap();

        // this writer does not have a fixed size. Loan should fail

        if let Ok(r) = another_writer.loan() {
            panic!("This must fail");
        }

        let subscriber = DdsSubscriber::create(&participant, None, None).unwrap();
        let reader = DdsReader::create_async(&subscriber, topic, None).unwrap();
        let another_reader = DdsReader::create_async(&subscriber, another_topic, None).unwrap();

        let rt = Runtime::new().unwrap();

        let _result = rt.block_on(async {
            let _another_task = tokio::spawn(async move {
                let mut samples = TestTopic::create_sample_buffer(5);
                if let Ok(t) = reader.take(&mut samples).await {
                    assert_eq!(t, 1);
                    for s in samples.iter() {
                        println!("Got sample {:?}", s);
                    }
                } else {
                    panic!("reader get failed");
                }
            });

            // add a delay to make sure the data is not ready immediately
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;

            let mut loaned = writer.loan().unwrap();

            let ptr = loaned.as_mut_ptr().unwrap();
            let topic = TestTopic::default();

            unsafe { ptr.write(topic) };
            let loaned = loaned.assume_init();
            writer.return_loan(loaned).unwrap();

            tokio::time::sleep(std::time::Duration::from_millis(300)).await;
        });
    }
}
