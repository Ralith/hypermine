use super::Base;

#[test]
fn init_base() {
    let _guard = common::tracing_guard();
    Base::headless();
}
