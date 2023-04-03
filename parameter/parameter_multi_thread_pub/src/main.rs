use safe_drive::{context::Context, error::DynError, logger::Logger, parameter::Value, pr_info};
use std::time::Duration;
use std::sync::{Arc, Mutex};

#[allow(unreachable_code)]
#[async_std::main]
async fn main() -> Result<(), DynError> {
  let ctx = Context::new()?;
  let node = ctx.create_node("parameter_multi_thread_pub", None, Default::default())?;

  let publisher1 = node.create_publisher::<std_msgs::msg::String>("task_1", None)?;
  let publisher2 = node.create_publisher::<std_msgs::msg::String>("task_2", None)?;

  let publish_string = Arc::new(Mutex::new("パラメータでここを書き換えれるよ".to_string()));
  let publish_hz = Arc::new(Mutex::new(1.0));

  let task1_publish_string = Arc::clone(&publish_string);
  let task1_publish_hz = Arc::clone(&publish_hz);
  let _task1 = async_std::task::spawn(async move {
    let mut cnt = 0;
    let mut msg = std_msgs::msg::String::new().unwrap();
    loop {
      let mut hz_outside= f64::default();
      {
        let hz_inside = task1_publish_hz.lock().unwrap();
        let str = task1_publish_string.lock().unwrap();
        let data = format!("俺はtask1だ！🥸 {str}: cnt = {cnt}");
        msg.data.assign(&data);
        publisher1.send(&msg).unwrap();
        cnt += 1;
        hz_outside = *hz_inside;
      }
      async_std::task::sleep(Duration::from_millis(((1.0/hz_outside)*1000.0) as u64)).await;
    }
  });

  let task2_publish_string = Arc::clone(&publish_string);
  let task2_publish_hz = Arc::clone(&publish_hz);
  let _task2 = async_std::task::spawn(async move {
    let mut cnt = 0;
    let mut msg = std_msgs::msg::String::new().unwrap();
    loop {
      let mut hz_outside= f64::default();
      {
        let hz_inside = task2_publish_hz.lock().unwrap();
        let str = task2_publish_string.lock().unwrap();
        let data = format!("俺はtask2だ！😈 {str}: cnt = {cnt}");
        msg.data.assign(&data);
        publisher2.send(&msg).unwrap();
        cnt += 1;
        hz_outside = *hz_inside;
      }
      async_std::task::sleep(Duration::from_millis(((1.0/hz_outside)*1000.0) as u64)).await;
    }
  });

  let mut param_server = node.create_parameter_server()?;
  {
    let mut params = param_server.params.write();

    params.set_dynamically_typed_parameter(
        "publish_string".to_string(),
        Value::String("こんにちは！世界".to_string()),
        false,
        Some("publish_string description".to_string()),
    )?;

    params.set_dynamically_typed_parameter(
        "publish_hz".to_string(),
        Value::F64(1.0),
        false,
        Some("publish_hz description".to_string()),
    )?;
  }

  let logger = Logger::new("parameter_multi_thread_pub");
  let param_server_publish_string = Arc::clone(&publish_string);
  let param_server_publish_hz = Arc::clone(&publish_hz);
  loop {
    let updated = param_server.wait().await?;

    let params = param_server.params.read(); // Read lock
    
    let mut string = param_server_publish_string.lock().unwrap();
    let mut hz = param_server_publish_hz.lock().unwrap();
    for key in updated.iter() {
      let value = &params.get_parameter(key).unwrap().value;
      match &key[..] {
          "publish_hz" => {
            *hz = format!("{}", value).parse::<f64>().unwrap();
            pr_info!(logger, "updated parameters[ name:{key} value:{hz}[Hz] ]");
          },
          "publish_string" => {
            *string = format!("{}", value).to_string();
            pr_info!(logger, "updated parameters[ name:{key} value:{string} ]");
          },
          _ => println!("not parameter[ name:{key} ]"),
      }
    }
  }

  _task1.await;
  _task2.await;
}