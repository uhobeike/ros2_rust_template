use safe_drive::{context::Context, error::DynError, logger::Logger, pr_info};

fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;

    let node = ctx.create_node("sub", None, Default::default())?;

    let subscriber = node.create_subscriber::<std_msgs::msg::String>("pubsub", None)?;

    let logger = Logger::new("sub");

    let mut selector = ctx.create_selector()?;

    selector.add_subscriber(
        subscriber,
        Box::new(move |msg| {
            pr_info!(logger, "receive: {}", msg.data);
        }),
    );

    loop {
        selector.wait()?;
    }
}
