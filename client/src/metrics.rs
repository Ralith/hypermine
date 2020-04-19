use std::{
    collections::HashMap,
    sync::{Arc, Mutex, RwLock},
    time::Duration,
};

use hdrhistogram::Histogram;
use metrics_core::Key;
use tracing::info;

pub fn init() -> Arc<Recorder> {
    let recorder = Arc::new(Recorder {
        histograms: RwLock::new(HashMap::new()),
    });
    metrics::set_boxed_recorder(Box::new(ArcRecorder(recorder.clone()))).unwrap();
    recorder
}

pub struct Recorder {
    histograms: RwLock<HashMap<Key, Mutex<Histogram<u64>>>>,
}

impl Recorder {
    pub fn report(&self) {
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
    fn increment_counter(&self, _key: Key, _value: u64) {
        todo!()
    }

    fn update_gauge(&self, _key: Key, _value: i64) {
        todo!()
    }

    fn record_histogram(&self, key: Key, value: u64) {
        let mut histograms = self.0.histograms.read().unwrap();
        let mut histogram = match histograms.get(&key) {
            Some(x) => x.lock().unwrap(),
            None => {
                drop(histograms);
                self.0
                    .histograms
                    .write()
                    .unwrap()
                    .insert(key.clone(), Mutex::new(Histogram::new(3).unwrap()));
                histograms = self.0.histograms.read().unwrap();
                histograms.get(&key).unwrap().lock().unwrap()
            }
        };
        histogram.record(value).unwrap();
    }
}
