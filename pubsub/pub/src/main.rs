// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

use safe_drive::{context::Context, error::DynError, logger::Logger, pr_info};
use std::time::Duration;

fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;

    let node = ctx.create_node("pub", None, Default::default())?;

    let publisher = node.create_publisher::<std_msgs::msg::String>("pubsub", None)?;

    let logger = Logger::new("pub");

    let mut cnt = 0;
    let mut msg = std_msgs::msg::String::new().unwrap();
    loop {
        let data = format!("Hello, World!: cnt = {cnt}");
        msg.data.assign(&data);

        pr_info!(logger, "send: {}", msg.data);

        publisher.send(&msg)?;

        cnt += 1;
        std::thread::sleep(Duration::from_secs(1));
    }
}
