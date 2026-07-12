//! `tpt-gcs` console entry point.
//!
//! Runs the dependency-free UDP console GCS. Usage:
//! ```text
//! tpt-gcs [bind] [remote]
//!   bind   local UDP address   (default 0.0.0.0:14551)
//!   remote vehicle UDP address (default 127.0.0.1:14550)
//! ```

use std::env;
use tpt_gcs::ConsoleGcs;

fn main() -> std::io::Result<()> {
    let args: Vec<String> = env::args().collect();
    let bind = args.get(1).map(String::as_str).unwrap_or("0.0.0.0:14551");
    let remote = args.get(2).map(String::as_str).unwrap_or("127.0.0.1:14550");

    let mut gcs = ConsoleGcs::new(bind, remote)?;
    gcs.run()
}
