fn main() {
    // Set up logging
    tracing_subscriber::fmt::init();

    if let Err(e) = server::run() {
        eprintln!("{:#}", e);
        std::process::exit(1);
    }
}
