use super::Base;

#[test]
#[ignore]
fn init_base() {
    let _guard = common::tracing_guard();
    Base::headless();
}
