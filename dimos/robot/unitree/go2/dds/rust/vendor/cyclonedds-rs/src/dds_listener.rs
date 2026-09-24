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

//! A Listner can be attached to different types of entities. The callbacks that
//! are supported depends on the type of entity. There is no checking for whether
//! an entity supports the callback.
//! # Example
//! ```
//! use cyclonedds_rs::DdsListener;
//! let listener = DdsListener::new()
//!   .on_subscription_matched(|a,b| {
//!     println!("Subscription matched!");
//! }).on_publication_matched(|a,b|{
//!     println!("Publication matched");
//! }).
//! hook(); // The hook call will finalize the listener. No more callbacks can be attached after this.
//! ```

use cyclonedds_sys::dds_listener_t;
use cyclonedds_sys::*;
use std::convert::From;

/*
 Each listener has its own set of callbacks.
*/

/// The callbacks are in a different structure that is always
/// heap allocated.
#[derive(Default)]
struct Callbacks {
    // Callbacks for readers
    on_sample_lost: Option<Box<dyn FnMut(DdsEntity, dds_sample_lost_status_t) + 'static>>,
    on_data_available: Option<Box<dyn FnMut(DdsEntity) + 'static>>,
    on_sample_rejected: Option<Box<dyn FnMut(DdsEntity, dds_sample_rejected_status_t) + 'static>>,
    on_liveliness_changed:
        Option<Box<dyn FnMut(DdsEntity, dds_liveliness_changed_status_t) + 'static>>,
    on_requested_deadline_missed:
        Option<Box<dyn FnMut(DdsEntity, dds_requested_deadline_missed_status_t) + 'static>>,
    on_requested_incompatible_qos:
        Option<Box<dyn FnMut(DdsEntity, dds_requested_incompatible_qos_status_t) + 'static>>,
    on_subscription_matched:
        Option<Box<dyn FnMut(DdsEntity, dds_subscription_matched_status_t) + 'static>>,

    //callbacks for writers
    on_liveliness_lost: Option<Box<dyn FnMut(DdsEntity, dds_liveliness_lost_status_t) + 'static>>,
    on_offered_deadline_missed:
        Option<Box<dyn FnMut(DdsEntity, dds_offered_deadline_missed_status_t) + 'static>>,
    on_offered_incompatible_qos:
        Option<Box<dyn FnMut(DdsEntity, dds_offered_incompatible_qos_status_t) + 'static>>,
    on_publication_matched:
        Option<Box<dyn FnMut(DdsEntity, dds_publication_matched_status_t) + 'static>>,

    on_inconsistent_topic:
        Option<Box<dyn FnMut(DdsEntity, dds_inconsistent_topic_status_t) + 'static>>,
    on_data_on_readers: Option<Box<dyn FnMut(DdsEntity) + 'static>>,
}

unsafe impl Send for Inner {}
struct Inner {
    listener: Option<*mut dds_listener_t>,
    callbacks: Option<Box<Callbacks>>,
    raw_ptr: Option<*mut Callbacks>,
}

#[derive(Clone)]
pub struct DdsListener {
    inner: std::sync::Arc<std::sync::Mutex<Inner>>,
}

impl<'a> DdsListener {
    pub fn new() -> Self {
        Self {
            inner: std::sync::Arc::new(std::sync::Mutex::new(Inner {
                listener: None,
                callbacks: Some(Box::default()),
                raw_ptr: None,
            })),
        }
    }
}

impl<'a> Default for DdsListener {
    fn default() -> Self {
        DdsListener::new()
    }
}

impl<'a> From<&DdsListener> for *const dds_listener_t {
    fn from(listener: &DdsListener) -> Self {
        if let Some(listener) = listener.inner.lock().unwrap().listener {
            listener
        } else {
            panic!("Attempt to convert from unitialized &listener");
        }
    }
}

impl<'a> DdsListener {
    // take ownership as we're going to do some bad stuff here.
    pub fn hook(self) -> Self {
        // we're going to grab the Boxed callbacks and keep them separately as
        // we will send a pointer to the callback array into C. We convert the
        // pointer back to a box in the Drop function.

        // free the previous pointer if present
        if let Some(raw) = self.inner.lock().unwrap().raw_ptr.take() {
            unsafe {
                // take ownership and free when out of scope
                Box::from_raw(raw);
            }
        }

        let inner = &self.inner;

        {
            let mut inner = inner.lock().unwrap();
            if let Some(b) = inner.callbacks.take() {
                let raw = Box::into_raw(b);
                unsafe {
                    let l = dds_create_listener(raw as *mut std::ffi::c_void);
                    if !l.is_null() {
                        let callbacks_ptr = raw as *mut Callbacks;
                        let callbacks = &*callbacks_ptr;
                        self.register_callbacks(l, callbacks);
                        inner.raw_ptr = Some(raw);
                        inner.listener = Some(l);
                    } else {
                        panic!("Error creating listener");
                    }
                }
            } else {
                println!("No callbacks to take");
            }
        }
        self
    }

    /// register the callbacks for the closures that have been set.DdsListener
    unsafe fn register_callbacks(&self, listener: *mut dds_listener_t, callbacks: &Callbacks) {
        if callbacks.on_data_available.is_some() {
            //println!("Listener hooked for data available");
            dds_lset_data_available(listener, Some(Self::call_data_available_closure));
        }
        if callbacks.on_sample_lost.is_some() {
            dds_lset_sample_lost(listener, Some(Self::call_sample_lost_closure));
        }

        if callbacks.on_sample_rejected.is_some() {
            dds_lset_sample_rejected(listener, Some(Self::call_sample_rejected_closure));
        }

        if callbacks.on_liveliness_changed.is_some() {
            dds_lset_liveliness_changed(listener, Some(Self::call_liveliness_changed_closure));
        }

        if callbacks.on_requested_deadline_missed.is_some() {
            dds_lset_requested_deadline_missed(
                listener,
                Some(Self::call_requested_deadline_missed_closure),
            );
        }

        if callbacks.on_requested_incompatible_qos.is_some() {
            dds_lset_requested_incompatible_qos(
                listener,
                Some(Self::call_requested_incompatible_qos_closure),
            );
        }

        if callbacks.on_subscription_matched.is_some() {
            dds_lset_subscription_matched(listener, Some(Self::call_subscription_matched_closure));
        }
        if callbacks.on_liveliness_lost.is_some() {
            dds_lset_liveliness_lost(listener, Some(Self::call_liveliness_lost_closure));
        }
        if callbacks.on_offered_deadline_missed.is_some() {
            dds_lset_offered_deadline_missed(
                listener,
                Some(Self::call_offered_deadline_missed_closure),
            );
        }
        if callbacks.on_offered_incompatible_qos.is_some() {
            dds_lset_offered_incompatible_qos(
                listener,
                Some(Self::call_offered_incompatible_qos_closure),
            );
        }
        if callbacks.on_publication_matched.is_some() {
            dds_lset_publication_matched(listener, Some(Self::call_publication_matched_closure));
        }
        if callbacks.on_inconsistent_topic.is_some() {
            dds_lset_inconsistent_topic(listener, Some(Self::call_inconsistent_topic_closure));
        }
        if callbacks.on_data_on_readers.is_some() {
            dds_lset_data_on_readers(listener, Some(Self::call_data_on_readers_closure));
        }
    }
}

//////
impl DdsListener {
    #[deprecated]
    pub fn on_data_available<F>(self, callback: F) -> Self
    where
        F: FnMut(DdsEntity) + 'static,
    {
        if let Some(callbacks) = &mut self.inner.lock().unwrap().callbacks {
            callbacks.on_data_available = Some(Box::new(callback));
        }

        self
    }

    unsafe extern "C" fn call_data_available_closure(
        reader: dds_entity_t,
        data: *mut std::ffi::c_void,
    ) {
        let callbacks_ptr = data as *mut Callbacks;
        let callbacks = &mut *callbacks_ptr;
        //        println!("C Callback!");
        if let Some(avail) = &mut callbacks.on_data_available {
            avail(DdsEntity::new(reader));
        }
    }
}

impl<'a> DdsListener {
    /////
    #[deprecated]
    pub fn on_sample_lost<F>(self, callback: F) -> Self
    where
        F: FnMut(DdsEntity, dds_sample_lost_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self.inner.lock().unwrap().callbacks {
            callbacks.on_sample_lost = Some(Box::new(callback));
        }
        self
    }

    unsafe extern "C" fn call_sample_lost_closure(
        reader: dds_entity_t,
        status: dds_sample_lost_status_t,
        data: *mut std::ffi::c_void,
    ) {
        let callbacks_ptr = data as *mut Callbacks;
        let callbacks = &mut *callbacks_ptr;
        //println!("C Callback - sample lost");
        if let Some(lost) = &mut callbacks.on_sample_lost {
            lost(DdsEntity::new(reader), status);
        }
    }
}

impl<'a> DdsListener {
    //////
    #[deprecated]
    pub fn on_sample_rejected<F>(self, callback: F) -> Self
    where
        F: FnMut(DdsEntity, dds_sample_rejected_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self.inner.lock().unwrap().callbacks {
            callbacks.on_sample_rejected = Some(Box::new(callback));
        }
        self
    }

    unsafe extern "C" fn call_sample_rejected_closure(
        reader: dds_entity_t,
        status: dds_sample_rejected_status_t,
        data: *mut std::ffi::c_void,
    ) {
        let callbacks_ptr = data as *mut Callbacks;
        let callbacks = &mut *callbacks_ptr;
        //println!("C Callback - sample rejected");
        if let Some(rejected) = &mut callbacks.on_sample_rejected {
            rejected(DdsEntity::new(reader), status);
        }
    }
}

// Liveliness changed
impl<'a> DdsListener {
    #[deprecated]
    pub fn on_liveliness_changed<F>(self, callback: F) -> Self
    where
        F: FnMut(DdsEntity, dds_liveliness_changed_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self.inner.lock().unwrap().callbacks {
            callbacks.on_liveliness_changed = Some(Box::new(callback));
        }
        self
    }

    unsafe extern "C" fn call_liveliness_changed_closure(
        entity: dds_entity_t,
        status: dds_liveliness_changed_status_t,
        data: *mut std::ffi::c_void,
    ) {
        let callbacks_ptr = data as *mut Callbacks;
        let callbacks = &mut *callbacks_ptr;
        //println!("C Callback - Liveliness changed");
        if let Some(changed) = &mut callbacks.on_liveliness_changed {
            changed(DdsEntity::new(entity), status);
        }
    }
}

impl<'a> DdsListener {
    #[deprecated]
    pub fn on_requested_deadline_missed<F>(self, callback: F) -> Self
    where
        F: FnMut(DdsEntity, dds_requested_deadline_missed_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self.inner.lock().unwrap().callbacks {
            callbacks.on_requested_deadline_missed = Some(Box::new(callback));
        }
        self
    }

    unsafe extern "C" fn call_requested_deadline_missed_closure(
        entity: dds_entity_t,
        status: dds_requested_deadline_missed_status_t,
        data: *mut std::ffi::c_void,
    ) {
        let callbacks_ptr = data as *mut Callbacks;
        let callbacks = &mut *callbacks_ptr;
        //println!("C Callback - requested deadline missed");
        if let Some(missed) = &mut callbacks.on_requested_deadline_missed {
            missed(DdsEntity::new(entity), status);
        }
    }
}

impl<'a> DdsListener {
    #[deprecated]
    pub fn on_requested_incompatible_qos<F>(self, callback: F) -> Self
    where
        F: FnMut(DdsEntity, dds_requested_incompatible_qos_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self.inner.lock().unwrap().callbacks {
            callbacks.on_requested_incompatible_qos = Some(Box::new(callback));
        }
        self
    }

    unsafe extern "C" fn call_requested_incompatible_qos_closure(
        entity: dds_entity_t,
        status: dds_requested_incompatible_qos_status_t,
        data: *mut std::ffi::c_void,
    ) {
        let callbacks_ptr = data as *mut Callbacks;
        let callbacks = &mut *callbacks_ptr;
        //println!("C Callback - requested incompatible QOS");
        if let Some(incompatible_qos) = &mut callbacks.on_requested_incompatible_qos {
            incompatible_qos(DdsEntity::new(entity), status);
        }
    }
}

impl<'a> DdsListener {
    #[deprecated]
    pub fn on_subscription_matched<F>(self, callback: F) -> Self
    where
        F: FnMut(DdsEntity, dds_subscription_matched_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self.inner.lock().unwrap().callbacks {
            callbacks.on_subscription_matched = Some(Box::new(callback));
        }
        self
    }

    unsafe extern "C" fn call_subscription_matched_closure(
        entity: dds_entity_t,
        status: dds_subscription_matched_status_t,
        data: *mut std::ffi::c_void,
    ) {
        let callbacks_ptr = data as *mut Callbacks;
        let callbacks = &mut *callbacks_ptr;
        //println!("C Callback - subscription matched");
        if let Some(matched) = &mut callbacks.on_subscription_matched {
            matched(DdsEntity::new(entity), status);
        }
    }
}

impl<'a> DdsListener {
    #[deprecated]
    pub fn on_liveliness_lost<F>(self, callback: F) -> Self
    where
        F: FnMut(DdsEntity, dds_liveliness_lost_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self.inner.lock().unwrap().callbacks {
            callbacks.on_liveliness_lost = Some(Box::new(callback));
        }
        self
    }

    unsafe extern "C" fn call_liveliness_lost_closure(
        entity: dds_entity_t,
        status: dds_liveliness_lost_status_t,
        data: *mut std::ffi::c_void,
    ) {
        let callbacks_ptr = data as *mut Callbacks;
        let callbacks = &mut *callbacks_ptr;
        //println!("C Callback - liveliness lost");
        if let Some(lost) = &mut callbacks.on_liveliness_lost {
            lost(DdsEntity::new(entity), status);
        }
    }
}

impl<'a> DdsListener {
    #[deprecated]
    pub fn on_offered_deadline_missed<F>(self, callback: F) -> Self
    where
        F: FnMut(DdsEntity, dds_offered_deadline_missed_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self.inner.lock().unwrap().callbacks {
            callbacks.on_offered_deadline_missed = Some(Box::new(callback));
        }
        self
    }

    unsafe extern "C" fn call_offered_deadline_missed_closure(
        entity: dds_entity_t,
        status: dds_offered_deadline_missed_status_t,
        data: *mut std::ffi::c_void,
    ) {
        let callbacks_ptr = data as *mut Callbacks;
        let callbacks = &mut *callbacks_ptr;
        //println!("C Callback - offered deadline missed");
        if let Some(missed) = &mut callbacks.on_offered_deadline_missed {
            missed(DdsEntity::new(entity), status);
        }
    }
}

impl<'a> DdsListener {
    #[deprecated]
    pub fn on_offered_incompatible_qos<F>(self, callback: F) -> Self
    where
        F: FnMut(DdsEntity, dds_offered_incompatible_qos_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self.inner.lock().unwrap().callbacks {
            callbacks.on_offered_incompatible_qos = Some(Box::new(callback));
        }
        self
    }

    unsafe extern "C" fn call_offered_incompatible_qos_closure(
        entity: dds_entity_t,
        status: dds_offered_incompatible_qos_status_t,
        data: *mut std::ffi::c_void,
    ) {
        let callbacks_ptr = data as *mut Callbacks;
        let callbacks = &mut *callbacks_ptr;
        //println!("C Callback - offered incompatible QOS");
        if let Some(incompatible) = &mut callbacks.on_offered_incompatible_qos {
            incompatible(DdsEntity::new(entity), status);
        }
    }
}

impl<'a> DdsListener {
    #[deprecated]
    pub fn on_publication_matched<F>(self, callback: F) -> Self
    where
        F: FnMut(DdsEntity, dds_publication_matched_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self.inner.lock().unwrap().callbacks {
            callbacks.on_publication_matched = Some(Box::new(callback));
        }
        self
    }

    unsafe extern "C" fn call_publication_matched_closure(
        entity: dds_entity_t,
        status: dds_publication_matched_status_t,
        data: *mut std::ffi::c_void,
    ) {
        let callbacks_ptr = data as *mut Callbacks;
        let callbacks = &mut *callbacks_ptr;
        //println!("C Callback - publication matched");
        if let Some(matched) = &mut callbacks.on_publication_matched {
            matched(DdsEntity::new(entity), status);
        }
    }
}

impl<'a> DdsListener {
    #[deprecated]
    pub fn on_inconsistent_topic<F>(self, callback: F) -> Self
    where
        F: FnMut(DdsEntity, dds_inconsistent_topic_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self.inner.lock().unwrap().callbacks {
            callbacks.on_inconsistent_topic = Some(Box::new(callback));
        }
        self
    }

    unsafe extern "C" fn call_inconsistent_topic_closure(
        entity: dds_entity_t,
        status: dds_inconsistent_topic_status_t,
        data: *mut std::ffi::c_void,
    ) {
        let callbacks_ptr = data as *mut Callbacks;
        let callbacks = &mut *callbacks_ptr;
        //println!("C Callback - inconsistent topic");
        if let Some(inconsistant) = &mut callbacks.on_inconsistent_topic {
            inconsistant(DdsEntity::new(entity), status);
        }
    }
}

impl<'a> DdsListener {
    #[deprecated]
    pub fn on_data_on_readers<F>(self, callback: F) -> Self
    where
        F: FnMut(DdsEntity) + 'static,
    {
        if let Some(callbacks) = &mut self.inner.lock().unwrap().callbacks {
            callbacks.on_data_on_readers = Some(Box::new(callback));
        }
        self
    }

    unsafe extern "C" fn call_data_on_readers_closure(
        entity: dds_entity_t,
        data: *mut std::ffi::c_void,
    ) {
        let callbacks_ptr = data as *mut Callbacks;
        let callbacks = &mut *callbacks_ptr;
        //println!("C Callback - data on readers");
        if let Some(data) = &mut callbacks.on_data_on_readers {
            data(DdsEntity::new(entity));
        }
    }
}

impl<'a> Drop for DdsListener {
    fn drop(&mut self) {
        // delete the listener so we are sure of not
        // getting any callbacks
        if let Some(listener) = &self.inner.lock().unwrap().listener {
            unsafe {
                dds_reset_listener(*listener);
                dds_delete_listener(*listener);
            }
        }
        // gain back control of the Callback structure
        if let Some(raw) = self.inner.lock().unwrap().raw_ptr.take() {
            unsafe {
                // take ownership and free when out of scope
                let _ = Box::from_raw(raw);
            }
        }
    }
}

#[derive(Default)]
pub struct DdsListenerBuilder {
    listener: Option<DdsListener>,
}

impl DdsListenerBuilder {
    pub fn new() -> Self {
        Self {
            listener: Some(DdsListener::new()),
        }
    }

    pub fn build(&mut self) -> DdsListener {
        self.listener.take().unwrap().hook()
    }

    pub fn on_data_available<F>(&mut self, callback: F) -> &mut Self
    where
        F: FnMut(DdsEntity) + 'static,
    {
        if let Some(callbacks) = &mut self
            .listener
            .as_ref()
            .unwrap()
            .inner
            .lock()
            .unwrap()
            .callbacks
        {
            callbacks.on_data_available = Some(Box::new(callback));
        }

        self
    }

    /////
    pub fn on_sample_lost<F>(&mut self, callback: F) -> &mut Self
    where
        F: FnMut(DdsEntity, dds_sample_lost_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self
            .listener
            .as_ref()
            .unwrap()
            .inner
            .lock()
            .unwrap()
            .callbacks
        {
            callbacks.on_sample_lost = Some(Box::new(callback));
        }
        self
    }

    //////
    pub fn on_sample_rejected<F>(&mut self, callback: F) -> &mut Self
    where
        F: FnMut(DdsEntity, dds_sample_rejected_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self
            .listener
            .as_ref()
            .unwrap()
            .inner
            .lock()
            .unwrap()
            .callbacks
        {
            callbacks.on_sample_rejected = Some(Box::new(callback));
        }
        self
    }

    // Liveliness changed
    pub fn on_liveliness_changed<F>(&mut self, callback: F) -> &mut Self
    where
        F: FnMut(DdsEntity, dds_liveliness_changed_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self
            .listener
            .as_ref()
            .unwrap()
            .inner
            .lock()
            .unwrap()
            .callbacks
        {
            callbacks.on_liveliness_changed = Some(Box::new(callback));
        }
        self
    }

    pub fn on_requested_deadline_missed<F>(&mut self, callback: F) -> &mut Self
    where
        F: FnMut(DdsEntity, dds_requested_deadline_missed_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self
            .listener
            .as_ref()
            .unwrap()
            .inner
            .lock()
            .unwrap()
            .callbacks
        {
            callbacks.on_requested_deadline_missed = Some(Box::new(callback));
        }
        self
    }

    pub fn on_requested_incompatible_qos<F>(&mut self, callback: F) -> &mut Self
    where
        F: FnMut(DdsEntity, dds_requested_incompatible_qos_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self
            .listener
            .as_ref()
            .unwrap()
            .inner
            .lock()
            .unwrap()
            .callbacks
        {
            callbacks.on_requested_incompatible_qos = Some(Box::new(callback));
        }
        self
    }

    pub fn on_subscription_matched<F>(&mut self, callback: F) -> &mut Self
    where
        F: FnMut(DdsEntity, dds_subscription_matched_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self
            .listener
            .as_ref()
            .unwrap()
            .inner
            .lock()
            .unwrap()
            .callbacks
        {
            callbacks.on_subscription_matched = Some(Box::new(callback));
        }
        self
    }

    pub fn on_liveliness_lost<F>(&mut self, callback: F) -> &mut Self
    where
        F: FnMut(DdsEntity, dds_liveliness_lost_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self
            .listener
            .as_ref()
            .unwrap()
            .inner
            .lock()
            .unwrap()
            .callbacks
        {
            callbacks.on_liveliness_lost = Some(Box::new(callback));
        }
        self
    }

    pub fn on_offered_deadline_missed<F>(&mut self, callback: F) -> &mut Self
    where
        F: FnMut(DdsEntity, dds_offered_deadline_missed_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self
            .listener
            .as_ref()
            .unwrap()
            .inner
            .lock()
            .unwrap()
            .callbacks
        {
            callbacks.on_offered_deadline_missed = Some(Box::new(callback));
        }
        self
    }

    pub fn on_offered_incompatible_qos<F>(&mut self, callback: F) -> &mut Self
    where
        F: FnMut(DdsEntity, dds_offered_incompatible_qos_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self
            .listener
            .as_ref()
            .unwrap()
            .inner
            .lock()
            .unwrap()
            .callbacks
        {
            callbacks.on_offered_incompatible_qos = Some(Box::new(callback));
        }
        self
    }

    pub fn on_publication_matched<F>(&mut self, callback: F) -> &mut Self
    where
        F: FnMut(DdsEntity, dds_publication_matched_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self
            .listener
            .as_ref()
            .unwrap()
            .inner
            .lock()
            .unwrap()
            .callbacks
        {
            callbacks.on_publication_matched = Some(Box::new(callback));
        }
        self
    }

    pub fn on_inconsistent_topic<F>(&mut self, callback: F) -> &mut Self
    where
        F: FnMut(DdsEntity, dds_inconsistent_topic_status_t) + 'static,
    {
        if let Some(callbacks) = &mut self
            .listener
            .as_ref()
            .unwrap()
            .inner
            .lock()
            .unwrap()
            .callbacks
        {
            callbacks.on_inconsistent_topic = Some(Box::new(callback));
        }
        self
    }

    pub fn on_data_on_readers<F>(&mut self, callback: F) -> &mut Self
    where
        F: FnMut(DdsEntity) + 'static,
    {
        if let Some(callbacks) = &mut self
            .listener
            .as_ref()
            .unwrap()
            .inner
            .lock()
            .unwrap()
            .callbacks
        {
            callbacks.on_data_on_readers = Some(Box::new(callback));
        }
        self
    }
}
