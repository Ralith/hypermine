use super::Base;

#[test]
#[ignore]
fn init_base() {
    let _ = tracing_subscriber::fmt::try_init();
    Base::headless();
}
