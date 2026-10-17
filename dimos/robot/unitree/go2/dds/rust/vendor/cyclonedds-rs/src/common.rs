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

use cyclonedds_sys::DdsEntity;

/// An entity on which you can attach a DdsWriter
pub trait DdsWritable {
    fn entity(&self) -> &DdsEntity;
}

/// An entity on which you can attach a DdsReader
pub trait DdsReadable {
    fn entity(&self) -> &DdsEntity;
}

pub trait Entity {
    fn entity(&self) -> &DdsEntity;
}
