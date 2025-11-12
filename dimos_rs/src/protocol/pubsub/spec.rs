// use std::sync::mpsc::{Receiver, Sender, channel};

// use pyo3::prelude::*;
// use pyo3::types::PyBytes;

pub type MsgT<T> = T;
pub type TopicT<T> = T;
pub type SubscriptionFn<TopicT, MsgT> = Box<dyn FnOnce(MsgT, TopicT) + Send>;

pub struct Subscription {
    unsubscribe_fn: Option<Box<dyn FnOnce() + Send>>,
}

impl Subscription {
    pub fn new<F>(f: F) -> Self
    where
        F: FnOnce() + Send + 'static,
    {
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
    fn subscribe<F>(&self, topic: TopicT, callback: F) -> Subscription
    where
        F: Fn(MsgT, TopicT) + Send + 'static;

    /// Unsubscribe from a topic.
    fn sub(&self, topic: TopicT, callback: SubscriptionFn<TopicT, 'static, MsgT>) -> Subscription {
        self.subscribe(topic, |msg, topic| callback(msg, topic))
    }
}
