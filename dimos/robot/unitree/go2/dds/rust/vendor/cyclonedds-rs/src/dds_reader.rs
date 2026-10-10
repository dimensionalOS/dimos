/*
    Copyright 2021 Sojan James

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
use std::future::Future;
use std::os::raw::c_void;
use std::pin::Pin;
use std::sync::{Arc, Mutex};
use std::task::{Context, Poll, Waker};
//use std::convert::TryInto;

pub use cyclonedds_sys::{DdsDomainId, DdsEntity};

use std::marker::PhantomData;

use crate::dds_listener::DdsListenerBuilder;
use crate::error::ReaderError;
use crate::serdes::{SampleBuffer, TopicType};
use crate::{dds_listener::DdsListener, dds_qos::DdsQos, dds_topic::DdsTopic, DdsReadable, Entity};

/// Builder structure for reader
pub struct ReaderBuilder<T: TopicType> {
    maybe_qos: Option<DdsQos>,
    maybe_listener: Option<DdsListener>,
    is_async: bool,
    phantom: PhantomData<T>,
}

impl<T> ReaderBuilder<T>
where
    T: TopicType,
{
    pub fn new() -> Self {
        Self {
            maybe_qos: None,
            maybe_listener: None,
            is_async: false,
            phantom: PhantomData,
        }
    }
    /// Create a reader with async support.  If this is enabled,
    /// the builder creates listeners internally. Any listener
    /// passed separately via the `with_listener` api will be
    /// ignored.
    pub fn as_async(mut self) -> Self {
        self.is_async = true;
        self
    }

    /// Create a reader with the specified Qos
    pub fn with_qos(mut self, qos: DdsQos) -> Self {
        self.maybe_qos = Some(qos);
        self
    }

    /// Created a reader with the specified listener.
    /// Note that this is ignored if an async reader
    /// is created.
    pub fn with_listener(mut self, listener: DdsListener) -> Self {
        self.maybe_listener = Some(listener);
        self
    }

    pub fn create(
        self,
        entity: &dyn DdsReadable,
        topic: DdsTopic<T>,
    ) -> Result<DdsReader<T>, DDSError> {
        if self.is_async {
            DdsReader::create_async(entity, topic, self.maybe_qos)
        } else {
            DdsReader::create_sync_or_async(
                entity,
                topic,
                self.maybe_qos,
                self.maybe_listener,
                ReaderType::Sync,
            )
        }
    }
}

enum ReaderType {
    Async(Arc<Mutex<(Option<Waker>, Result<(), crate::error::ReaderError>)>>),
    Sync,
}

struct Inner<T: Sized + TopicType> {
    entity: DdsEntity,
    _listener: Option<DdsListener>,
    reader_type: ReaderType,
    _phantom: PhantomData<T>,
    // The callback closures that can be attached to a reader
}

pub struct DdsReader<T: Sized + TopicType> {
    inner: Arc<Inner<T>>,
}

impl<'a, T> DdsReader<T>
where
    T: Sized + TopicType,
{
    pub fn create(
        entity: &dyn DdsReadable,
        topic: DdsTopic<T>,
        maybe_qos: Option<DdsQos>,
        maybe_listener: Option<DdsListener>,
    ) -> Result<Self, DDSError> {
        Self::create_sync_or_async(entity, topic, maybe_qos, maybe_listener, ReaderType::Sync)
    }

    fn create_sync_or_async(
        entity: &dyn DdsReadable,
        topic: DdsTopic<T>,
        maybe_qos: Option<DdsQos>,
        maybe_listener: Option<DdsListener>,
        reader_type: ReaderType,
    ) -> Result<Self, DDSError> {
        unsafe {
            let w = dds_create_reader(
                entity.entity().entity(),
                topic.entity().entity(),
                maybe_qos.map_or(std::ptr::null(), |q| q.into()),
                maybe_listener
                    .as_ref()
                    .map_or(std::ptr::null(), |l| l.into()),
            );

            if w >= 0 {
                Ok(DdsReader {
                    inner: Arc::new(Inner {
                        entity: DdsEntity::new(w),
                        _listener: maybe_listener,
                        reader_type,
                        _phantom: PhantomData,
                    }),
                })
            } else {
                Err(DDSError::from(w))
            }
        }
    }

    /// Create an async reader. This constructor must be used if using any of the async functions.
    pub fn create_async(
        entity: &dyn DdsReadable,
        topic: DdsTopic<T>,
        maybe_qos: Option<DdsQos>,
    ) -> Result<Self, DDSError> {
        let waker = Arc::new(<Mutex<(
            Option<Waker>,
            Result<(), crate::error::ReaderError>,
        )>>::new((None, Ok(()))));
        let waker_cb = waker.clone();
        let requested_deadline_waker = waker.clone();

        let listener = DdsListenerBuilder::new()
            .on_data_available(move |_entity| {
                //println!("Data available ");
                let mut maybe_waker = waker_cb.lock().unwrap();
                if let Some(waker) = maybe_waker.0.take() {
                    waker.wake();
                }
            })
            .on_requested_deadline_missed(move |entity, status| {
                println!(
                    "Deadline missed: Entity:{:?} Status:{:?}",
                    unsafe { entity.entity() },
                    status
                );
                let mut maybe_waker = requested_deadline_waker.lock().unwrap();
                maybe_waker.1 = Err(ReaderError::RequestedDeadLineMissed);
                if let Some(waker) = maybe_waker.0.take() {
                    waker.wake();
                }
            })
            .build();

        match Self::create_sync_or_async(
            entity,
            topic,
            maybe_qos,
            Some(listener),
            ReaderType::Async(waker),
        ) {
            Ok(reader) => Ok(reader),
            Err(e) => Err(e),
        }
    }

    /// read synchronously
    pub fn read_now(&self, buf: &mut SampleBuffer<T>) -> Result<usize, DDSError> {
        Self::readn_from_entity_now(self.entity(), buf, false)
    }

    /// take synchronously
    pub fn take_now(&self, buf: &mut SampleBuffer<T>) -> Result<usize, DDSError> {
        Self::readn_from_entity_now(self.entity(), buf, true)
    }

    /// Read multiple samples from the reader synchronously. The buffer for the sampes must be passed in.
    /// On success, returns the number of samples read.
    pub fn readn_from_entity_now(
        entity: &DdsEntity,
        buf: &mut SampleBuffer<T>,
        take: bool,
    ) -> Result<usize, DDSError> {
        let (voidp, info_ptr) = unsafe { buf.as_mut_ptr() };
        let voidpp = voidp as *mut *mut c_void;
        //println!("Infoptr:{:?}",info_ptr);

        let ret = unsafe {
            if take {
                dds_take(
                    entity.entity(),
                    voidpp,
                    info_ptr as *mut _,
                    buf.len() as size_t,
                    buf.len() as u32,
                )
            } else {
                dds_read(
                    entity.entity(),
                    voidpp,
                    info_ptr as *mut _,
                    buf.len() as size_t,
                    buf.len() as u32,
                )
            }
        };
        if ret > 0 {
            // If first sample is value we assume all are
            if buf.is_valid_sample(0) {
                Ok(ret as usize)
            } else {
                Err(DDSError::NoData)
            }
        } else {
            Err(DDSError::OutOfResources)
        }
    }

    /// Read samples asynchronously. The number of samples actually read is returned.
    pub async fn read(&self, samples: &mut SampleBuffer<T>) -> Result<usize, ReaderError> {
        if let ReaderType::Async(waker) = &self.inner.reader_type {
            let future_sample = SampleArrayFuture::new(
                self.inner.entity.clone(),
                waker.clone(),
                samples,
                FutureType::Read,
            );
            future_sample.await
        } else {
            Err(ReaderError::ReaderNotAsync)
        }
    }

    /// Get samples asynchronously. The number of samples actually read is returned.
    pub async fn take(&self, samples: &mut SampleBuffer<T>) -> Result<usize, ReaderError> {
        if let ReaderType::Async(waker) = &self.inner.reader_type {
            let future_sample = SampleArrayFuture::new(
                self.inner.entity.clone(),
                waker.clone(),
                samples,
                FutureType::Take,
            );
            future_sample.await
        } else {
            Err(ReaderError::ReaderNotAsync)
        }
    }

    pub fn create_readcondition(
        &'a mut self,
        mask: StateMask,
    ) -> Result<DdsReadCondition<T>, DDSError> {
        DdsReadCondition::create(self, mask)
    }
}

impl<'a, T> Entity for DdsReader<T>
where
    T: std::marker::Sized + TopicType,
{
    fn entity(&self) -> &DdsEntity {
        &self.inner.entity
    }
}

impl<'a, T> Drop for DdsReader<T>
where
    T: Sized + TopicType,
{
    fn drop(&mut self) {
        unsafe {
            //println!("Drop reader:{:?}", self.entity().entity());
            let ret: DDSError = cyclonedds_sys::dds_delete(self.inner.entity.entity()).into();
            if DDSError::DdsOk != ret {
                //panic!("cannot delete Reader: {}", ret);
                println!("Ignoring dds_delete failure for DdsReader");
            } else {
                //println!("Reader dropped");
            }
        }
    }
}

pub struct DdsReadCondition<'a, T: Sized + TopicType>(DdsEntity, &'a DdsReader<T>);

impl<'a, T> DdsReadCondition<'a, T>
where
    T: Sized + TopicType,
{
    fn create(reader: &'a DdsReader<T>, mask: StateMask) -> Result<Self, DDSError> {
        unsafe {
            let mask: u32 = *mask;
            let p = cyclonedds_sys::dds_create_readcondition(reader.entity().entity(), mask);
            if p > 0 {
                Ok(DdsReadCondition(DdsEntity::new(p), reader))
            } else {
                Err(DDSError::from(p))
            }
        }
    }
}

impl<'a, T> Entity for DdsReadCondition<'a, T>
where
    T: std::marker::Sized + TopicType,
{
    fn entity(&self) -> &DdsEntity {
        &self.0
    }
}

enum FutureType {
    Take,
    Read,
}

impl FutureType {
    fn is_take(&self) -> bool {
        match self {
            FutureType::Take => true,
            FutureType::Read => false,
        }
    }
}

struct SampleArrayFuture<'a, T> {
    entity: DdsEntity,
    waker: Arc<Mutex<(Option<Waker>, Result<(), crate::error::ReaderError>)>>,
    take_or_read: FutureType,
    buffer: &'a mut SampleBuffer<T>,
}

impl<'a, T> SampleArrayFuture<'a, T> {
    fn new(
        entity: DdsEntity,
        waker: Arc<Mutex<(Option<Waker>, Result<(), crate::error::ReaderError>)>>,
        buffer: &'a mut SampleBuffer<T>,
        ty: FutureType,
    ) -> Self {
        Self {
            entity,
            waker,
            take_or_read: ty,
            buffer,
        }
    }
}

impl<'a, T> Future for SampleArrayFuture<'a, T>
where
    T: TopicType,
{
    type Output = Result<usize, ReaderError>;

    fn poll(mut self: Pin<&mut Self>, ctx: &mut Context) -> Poll<Self::Output> {
        // Lock the waker first in case a callback for read complete happens and we miss it
        // clone to avoid the lifetime problem with self
        let waker = self.waker.clone();
        let mut waker = waker.lock().unwrap();
        let is_take = self.take_or_read.is_take();
        let entity = self.entity.clone();

        // check if we have an error from any of the callbacks
        if let Err(e) = &waker.1 {
            return Poll::Ready(Err(e.clone()));
        }

        match DdsReader::<T>::readn_from_entity_now(&entity, self.buffer, is_take) {
            Ok(len) => Poll::Ready(Ok(len)),
            Err(DDSError::NoData) | Err(DDSError::OutOfResources) => {
                let _ = waker.0.replace(ctx.waker().clone());
                Poll::Pending
            }
            Err(e) => {
                //println!("Error:{}",e);
                // Some other error happened
                Poll::Ready(Err(ReaderError::DdsError(e)))
            }
        }
    }
}

#[cfg(test)]
mod test {
    use core::panic;
    use std::time::Duration;

    use super::*;
    use crate::{DdsParticipant, DdsSubscriber};
    use crate::{DdsPublisher, DdsWriter};

    use cdds_derive::Topic;
    use serde_derive::{Deserialize, Serialize};
    use tokio::runtime::Runtime;

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

    #[derive(Serialize, Deserialize, Topic, Debug, PartialEq)]
    struct TestTopic {
        a: u32,
        b: u16,
        c: String,
        d: Vec<u8>,
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
                c: "TestTopic".to_owned(),
                d: vec![1, 2, 3, 4, 5],
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

    #[test]
    fn test_reader_async() {
        let participant = DdsParticipant::create(None, None, None).unwrap();

        let topic = TestTopic::create_topic(&participant, Some("test_topic"), None, None).unwrap();
        let another_topic = AnotherTopic::create_topic(&participant, None, None, None).unwrap();

        let publisher = DdsPublisher::create(&participant, None, None).unwrap();

        let mut writer = DdsWriter::create(&publisher, topic.clone(), None, None).unwrap();
        let mut another_writer =
            DdsWriter::create(&publisher, another_topic.clone(), None, None).unwrap();

        let subscriber = DdsSubscriber::create(&participant, None, None).unwrap();
        let reader = DdsReader::create_async(&subscriber, topic, None).unwrap();
        let another_reader = DdsReader::create_async(&subscriber, another_topic, None).unwrap();

        let rt = Runtime::new().unwrap();

        let _result = rt.block_on(async {
            let _task = tokio::spawn(async move {
                let mut samplebuffer = SampleBuffer::new(1);
                if let Ok(t) = reader.take(&mut samplebuffer).await {
                    let sample = samplebuffer.iter().take(1).next().unwrap();
                    assert!(*sample == TestTopic::default());
                } else {
                    panic!("reader get failed");
                }
            });

            let _another_task = tokio::spawn(async move {
                let mut samples = AnotherTopic::create_sample_buffer(5);
                if let Ok(t) = another_reader.read(&mut samples).await {
                    assert_eq!(t, 1);
                    for s in samples.iter() {
                        println!("Got sample {}", s.key);
                    }
                } else {
                    panic!("reader get failed");
                }
            });

            // add a delay to make sure the data is not ready immediately
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
            let data = Arc::new(TestTopic::default());
            writer.write(data).unwrap();

            another_writer
                .write(Arc::new(AnotherTopic::default()))
                .unwrap();

            tokio::time::sleep(std::time::Duration::from_millis(300)).await;
        });
    }
    /*
        #[test]
        fn test_requested_deadline_miss() {
            let participant = DdsParticipant::create(None, None, None).unwrap();
            let topic = TestTopic::create_topic(&participant, Some("test_topic"), None, None).unwrap();
            let publisher = DdsPublisher::create(&participant, None, None).unwrap();

            let writer_qos = DdsQos::create().unwrap().set_deadline(std::time::Duration::from_millis(50));
            let mut writer = DdsWriter::create(&publisher, topic.clone(), Some(writer_qos), None).unwrap();




            let reader_qos = DdsQos::create().unwrap().set_deadline(std::time::Duration::from_millis(500));
            let subscriber = DdsSubscriber::create(&participant, None, None).unwrap();
            let reader = DdsReader::create_async(&subscriber, topic, None).unwrap();

            let rt = Runtime::new().unwrap();

            let _result = rt.block_on(async {


                let t = tokio::spawn(async move {

                    loop {
                        let d = writer.write(Arc::new(TestTopic::default())).unwrap();
                        tokio::time::sleep(Duration::from_millis(1000)).await;
                    }

                });

                loop {

                    let d = reader.read1().await;
                    tokio::time::sleep(Duration::from_millis(700)).await;
                    println!("reader returned:{:?}",d);
                }



            });

        }
    */
}
