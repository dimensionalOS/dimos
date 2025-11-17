use std::sync::mpsc::{Receiver, Sender, SyncSender, channel, sync_channel};

// use pyo3::prelude::*;
// use pyo3::types::PyBytes;

pub type MsgT<T> = T;
pub type TopicT<T> = T;
pub type SubscriptionFn = Box<dyn FnOnce() + Send>;
pub type CallbackFn<TopicT, MsgT> = Box<dyn Fn(MsgT, TopicT) + Send>;

pub struct Subscription {
    unsubscribe_fn: Option<SubscriptionFn>,
}

impl Subscription {
    pub fn new(f: SubscriptionFn) -> Self {
        Self {
            unsubscribe_fn: Some(Box::new(f)),
        }
    }

    /// Explicitly unsubscribe now. Safe to call at most once.
    pub fn unsubscribe(&mut self) {
        if let Some(f) = self.unsubscribe_fn.take() {
            f();
        }
    }
}

impl Drop for Subscription {
    fn drop(&mut self) {
        if let Some(f) = self.unsubscribe_fn.take() {
            f();
        }
    }
}

pub trait PubSub<TopicT, MsgT> {
    /// Publish a message to a topic.
    fn publish(&self, topic: TopicT, message: MsgT);

    /// Subscribe to a topic with a callback. returns unsubscribe function
    fn subscribe(&self, topic: TopicT, callback: CallbackFn<TopicT, MsgT>) -> Subscription;

    /// Unsubscribe from a topic.
    fn sub(&self, topic: TopicT, callback: CallbackFn<TopicT, MsgT>) -> Subscription {
        self.subscribe(topic, callback)
    }
}
