use safe_drive::{
    context::Context, error::DynError, logger::Logger, pr_info, topic::subscriber::Subscriber,
};

#[async_std::main]
async fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;
    let node = ctx.create_node("multi_thread_sub", None, Default::default())?;

    let subscriber1 =
        node.create_subscriber::<std_msgs::msg::String>("multi_thread_pubsub_1", None)?;
    let subscriber2 =
        node.create_subscriber::<std_msgs::msg::String>("multi_thread_pubsub_2", None)?;

    let task1 = async_std::task::spawn(receiver(subscriber1));
    let task2 = async_std::task::spawn(receiver(subscriber2));

    task1.await?;
    task2.await?;

    Ok(())
}

async fn receiver(mut subscriber: Subscriber<std_msgs::msg::String>) -> Result<(), DynError> {
    let logger = Logger::new(subscriber.get_topic_name());

    loop {
        let msg = subscriber.recv().await?;
        pr_info!(logger, "{}", msg.data);
    }
}
