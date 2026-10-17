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

use crate::{dds_listener::DdsListener, dds_participant::DdsParticipant, dds_qos::DdsQos, Entity};

use std::convert::From;
use std::ffi::CString;
use std::marker::PhantomData;

use crate::serdes::{SerType, TopicType};
pub use cyclonedds_sys::{ddsi_sertype, DDSError, DdsEntity};

pub struct TopicBuilder<T: TopicType> {
    maybe_qos: Option<DdsQos>,
    maybe_listener: Option<DdsListener>,
    topic_name: String,
    phantom: PhantomData<T>,
}

impl<T> TopicBuilder<T>
where
    T: TopicType,
{
    pub fn new() -> Self {
        Self {
            maybe_qos: None,
            maybe_listener: None,
            topic_name: T::topic_name(None),
            phantom: PhantomData,
        }
    }

    pub fn with_name(mut self, name: String) -> Self {
        self.topic_name = name;
        self
    }

    pub fn with_name_prefix(mut self, mut prefix_name: String) -> Self {
        prefix_name.push_str(self.topic_name.as_str());
        self.topic_name = prefix_name;
        self
    }

    pub fn with_qos(mut self, qos: DdsQos) -> Self {
        self.maybe_qos = Some(qos);
        self
    }

    pub fn with_listener(mut self, listener: DdsListener) -> Self {
        self.maybe_listener = Some(listener);
        self
    }

    pub fn create(self, participant: &DdsParticipant) -> Result<DdsTopic<T>, DDSError> {
        DdsTopic::<T>::create(
            participant,
            self.topic_name.as_str(),
            self.maybe_qos,
            self.maybe_listener,
        )
    }
}

pub struct DdsTopic<T: Sized + TopicType>(DdsEntity, PhantomData<T>, Option<DdsListener>);

impl<T> DdsTopic<T>
where
    T: std::marker::Sized + TopicType,
{
    pub fn create(
        participant: &DdsParticipant,
        name: &str,
        maybe_qos: Option<DdsQos>,
        maybe_listener: Option<DdsListener>,
    ) -> Result<Self, DDSError> {
        let t = SerType::<T>::new();
        let mut t = SerType::into_sertype(t);
        let tt = &mut t as *mut *mut ddsi_sertype;

        unsafe {
            let strname = CString::new(name).expect("CString::new failed");
            let topic = cyclonedds_sys::dds_create_topic_sertype(
                participant.entity().entity(),
                strname.as_ptr(),
                tt,
                maybe_qos.map_or(std::ptr::null(), |q| q.into()),
                maybe_listener
                    .as_ref()
                    .map_or(std::ptr::null(), |l| l.into()),
                std::ptr::null_mut(),
            );

            if topic >= 0 {
                Ok(DdsTopic(DdsEntity::new(topic), PhantomData, maybe_listener))
            } else {
                Err(DDSError::from(topic))
            }
        }
    }
}

impl<T> Entity for DdsTopic<T>
where
    T: std::marker::Sized + TopicType,
{
    fn entity(&self) -> &DdsEntity {
        &self.0
    }
}

impl<T> Clone for DdsTopic<T>
where
    T: std::marker::Sized + TopicType,
{
    fn clone(&self) -> Self {
        Self(self.0.clone(), PhantomData, self.2.clone())
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::SampleBuffer;
    use crate::{DdsPublisher, DdsWriter};
    use cdds_derive::Topic;
    use serde_derive::{Deserialize, Serialize};
    use std::sync::Arc;
    #[test]
    fn test_topic_creation() {
        #[derive(Default, Deserialize, Serialize, Topic)]
        struct MyTopic {
            #[topic_key]
            a: u32,
            b: u32,
            c: String,
            d: u32,
        }

        assert_eq!(
            MyTopic::topic_name(None),
            String::from("/dds_topic/test/test_topic_creation/MyTopic")
        );
        assert_eq!(
            MyTopic::topic_name(Some("prefix")),
            String::from("prefix/dds_topic/test/test_topic_creation/MyTopic")
        );

        let participant = DdsParticipant::create(None, None, None).unwrap();
        let topic = MyTopic::create_topic(&participant, None, None, None).unwrap();
        let publisher =
            DdsPublisher::create(&participant, None, None).expect("Unable to create publisher");
        let mut writer = DdsWriter::create(&publisher, topic, None, None).unwrap();

        // MyTopic::create_writer()

        let data = Arc::new(MyTopic {
            a: 1,
            b: 32,
            c: "my_data_sample".to_owned(),
            d: 546,
        });

        writer.write(data).unwrap();
    }
}
