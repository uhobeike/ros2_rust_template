use safe_drive::{context::Context, error::DynError};
use std::time::Duration;

#[async_std::main]
async fn main() -> Result<(), DynError> {
  let ctx = Context::new()?;
  let node = ctx.create_node("multi_thread_pub", None, Default::default())?;

  let publisher1 = node.create_publisher::<std_msgs::msg::String>("multi_thread_pubsub_1", None)?;
  let publisher2 = node.create_publisher::<std_msgs::msg::String>("multi_thread_pubsub_2", None)?;

  let task1 = async_std::task::spawn(async move {
    let mut cnt = 0;
    let mut msg = std_msgs::msg::String::new().unwrap();
    for _ in 0..50 {
      let data = format!("Hello, World!: cnt = {cnt}");
      msg.data.assign(&data);
      publisher1.send(&msg).unwrap();
      cnt += 1;
      async_std::task::sleep(Duration::from_millis(500)).await;
    }
  });

  let task2 = async_std::task::spawn(async move {
    let mut cnt = 0;
    let mut msg = std_msgs::msg::String::new().unwrap();
    for _ in 0..100 {
      let data = format!("Hello, Universe!: cnt = {cnt}");
      msg.data.assign(&data);
      publisher2.send(&msg).unwrap();
      cnt += 1;
      async_std::task::sleep(Duration::from_millis(250)).await;
    }
  });

  task1.await;
  task2.await;

  Ok(())
}