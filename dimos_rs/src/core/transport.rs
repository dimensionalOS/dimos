use colored::Colorize;
use pyo3::prelude::*;
use pyo3::types::{PyAny, PyDict};
use std::ffi::CStr;

#[pyclass(subclass)]
pub struct PubSubTransport {
    #[pyo3(get, set)]
    topic: Py<PyAny>,
}

#[pymethods]
impl PubSubTransport {
    #[new]
    fn new(topic: Py<PyAny>) -> Self {
        PubSubTransport { topic }
    }

    fn __str__(&self, py: Python<'_>) -> PyResult<String> {
        let topic_str = self.topic.bind(py).repr()?.to_string();
        Ok(format!(
            "{}{}{}",
            "PubSubTransport(".green(),
            topic_str.blue(),
            ")".green()
        ))
    }

    fn __repr__(&self, py: Python<'_>) -> PyResult<String> {
        self.__str__(py)
    }
}

#[pyclass(extends=PubSubTransport)]
pub struct LCMTransport {
    #[pyo3(get, set)]
    _started: bool,
    lcm: Py<PyAny>,
}

#[pymethods]
impl LCMTransport {
    #[new]
    #[pyo3(signature = (topic, ty, **kwargs))]
    fn new(
        py: Python<'_>,
        topic: &str,
        ty: Py<PyAny>,
        kwargs: Option<&Bound<PyDict>>,
    ) -> PyResult<(Self, PubSubTransport)> {
        // Import LCMTopic and LCM from Python
    let lcmpubsub_module = py.import("dimos.protocol.pubsub.lcmpubsub")?;
        let lcm_topic_class = lcmpubsub_module.getattr("Topic")?;
        let lcm_class = lcmpubsub_module.getattr("LCM")?;

        // Create LCMTopic(topic, type)
        let lcm_topic = lcm_topic_class.call1((topic, ty))?;

        // Create LCM instance with kwargs
        let lcm_instance = if let Some(kwargs) = kwargs {
            lcm_class.call((), Some(kwargs))?
        } else {
            lcm_class.call0()?
        };

        let base = PubSubTransport::new(lcm_topic.into());
        Ok((
            LCMTransport {
                _started: false,
                lcm: lcm_instance.into(),
            },
            base,
        ))
    }

    fn __reduce__(&self, py: Python<'_>, slf: PyRef<'_, Self>) -> PyResult<Py<PyAny>> {
        // Get the topic from the base class
        let base: &PubSubTransport = slf.as_ref();
        let topic = base.topic.bind(py);

        // Extract topic.topic and topic.lcm_type
        let topic_str = topic.getattr("topic")?;
        let lcm_type = topic.getattr("lcm_type")?;

        // Return (LCMTransport, (topic.topic, topic.lcm_type))
        let builtins = py.import("builtins")?;
        let tuple = builtins.getattr("tuple")?;
        let args = tuple.call1((topic_str, lcm_type))?;

        // Get the class type
        let lcm_transport_class = py.get_type::<Self>();
        let reduce_result = tuple.call1((lcm_transport_class, args))?;

        Ok(reduce_result.into())
    }

    fn broadcast(
        &mut self,
        py: Python<'_>,
        _selfstream: Py<PyAny>,
        msg: Py<PyAny>,
        slf: PyRefMut<'_, Self>,
    ) -> PyResult<()> {
        if !self._started {
            self.lcm.bind(py).call_method0("start")?;
            self._started = true;
        }

        // Get the topic from the base class
        let base: &PubSubTransport = slf.as_ref();
        let topic = base.topic.bind(py);

        // Call lcm.publish(self.topic, msg)
        self.lcm.bind(py).call_method1("publish", (topic, msg))?;
        Ok(())
    }

    fn subscribe(
        &mut self,
        py: Python<'_>,
        callback: Py<PyAny>,
        _selfstream: Option<Py<PyAny>>,
        slf: PyRefMut<'_, Self>,
    ) -> PyResult<Py<PyAny>> {
        if !self._started {
            self.lcm.bind(py).call_method0("start")?;
            self._started = true;
        }

        // Get the topic from the base class
        let base: &PubSubTransport = slf.as_ref();
        let topic = base.topic.bind(py);

        // Create a lambda that wraps the callback: lambda msg, topic: callback(msg)
        let lambda_code = py.eval(
            CStr::from_bytes_with_nul(b"lambda callback: lambda msg, topic: callback(msg)\0")
                .unwrap(),
            None,
            None,
        )?;
        let wrapper = lambda_code.call1((callback,))?;

        // Call lcm.subscribe(self.topic, wrapper)
        let unsubscribe_fn = self
            .lcm
            .bind(py)
            .call_method1("subscribe", (topic, wrapper))?;
        Ok(unsubscribe_fn.into())
    }
}
