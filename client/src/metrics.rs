use std::{
    collections::HashMap,
    sync::{Arc, Mutex, RwLock},
    time::Duration,
};

use hdrhistogram::Histogram;
use tracing::info;

pub fn init() -> Arc<Recorder> {
    let recorder = Arc::new(Recorder {
        histograms: RwLock::new(HashMap::new()),
    });
    metrics::set_boxed_recorder(Box::new(ArcRecorder(recorder.clone()))).unwrap();
    recorder
}

pub struct Recorder {
    histograms: RwLock<HashMap<metrics::Key, Mutex<Histogram<u64>>>>,
}

impl Recorder {
    pub fn report(&self) {
        // metrics crate documentation assures us that Key's interior mutability does not affect the hash code.
        #[allow(clippy::mutable_key_type)]
        let histograms = &*self.histograms.read().unwrap();
        for (key, histogram) in histograms {
            let histogram = histogram.lock().unwrap();
            info!(
                key = %key.name(),
                percentile_25 = ?Duration::from_nanos(histogram.value_at_quantile(0.25)),
                percentile_50 = ?Duration::from_nanos(histogram.value_at_quantile(0.50)),
                percentile_75 = ?Duration::from_nanos(histogram.value_at_quantile(0.75)),
                max = ?Duration::from_nanos(histogram.value_at_quantile(1.0)),
                "metric"
            );
        }
    }
}

struct ArcRecorder(Arc<Recorder>);

impl metrics::Recorder for ArcRecorder {
    fn describe_counter(
        &self,
        _key: metrics::KeyName,
        _unit: Option<metrics::Unit>,
        _description: metrics::SharedString,
    ) {
        todo!()
    }

    fn describe_gauge(
        &self,
        _key: metrics::KeyName,
        _unit: Option<metrics::Unit>,
        _description: metrics::SharedString,
    ) {
        todo!()
    }

    fn describe_histogram(
        &self,
        _key: metrics::KeyName,
        _unit: Option<metrics::Unit>,
        _description: metrics::SharedString,
    ) {
        todo!()
    }

    fn register_counter(&self, _key: &metrics::Key) -> metrics::Counter {
        todo!()
    }

    fn register_gauge(&self, _key: &metrics::Key) -> metrics::Gauge {
        todo!()
    }

    fn register_histogram(&self, key: &metrics::Key) -> metrics::Histogram {
        metrics::Histogram::from_arc(Arc::new(Handle {
            recorder: self.0.clone(),
            key: key.clone(),
        }))
    }
}

struct Handle {
    recorder: Arc<Recorder>,
    key: metrics::Key,
}

impl metrics::HistogramFn for Handle {
    fn record(&self, value: f64) {
        let mut histograms = self.recorder.histograms.read().unwrap();
        let mut histogram = match histograms.get(&self.key) {
            Some(x) => x.lock().unwrap(),
            None => {
                drop(histograms);
                self.recorder
                    .histograms
                    .write()
                    .unwrap()
                    .insert(self.key.clone(), Mutex::new(Histogram::new(3).unwrap()));
                histograms = self.recorder.histograms.read().unwrap();
                histograms.get(&self.key).unwrap().lock().unwrap()
            }
        };
        histogram.record((value * 1e9) as u64).unwrap();
    }
}
