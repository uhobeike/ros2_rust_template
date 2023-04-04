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

  let publish_string = Arc::new(Mutex::new(String::default()));
  let publish_hz = Arc::new(Mutex::new(1.0));
  let mut param_server = Arc::new(Mutex::new(node.create_parameter_server()?));

  let param = Arc::clone(&param_server);
  {
    let mut param_server = param.lock().unwrap();
    let mut params = param_server.params.write();

    params.set_dynamically_typed_parameter(
        "publish_string".to_string(),
        Value::String("パラメータでここを書き換えれるよ".to_string()),
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

  let param = Arc::clone(&param_server);
  let param_server_publish_string = Arc::clone(&publish_string);
  let param_server_publish_hz = Arc::clone(&publish_hz);
  let get_param = async_std::task::spawn(async move {
        let mut param_server = param.lock().unwrap();

        let params = param_server.params.read();

        let mut string = param_server_publish_string.lock().unwrap();
        let value = &params.get_parameter("publish_string").unwrap().value;
        *string = format!("{}", value).to_string();
        

        let mut hz = param_server_publish_hz.lock().unwrap();
        let value = &params.get_parameter("publish_hz").unwrap().value;
        *hz = format!("{}", value).parse::<f64>().unwrap();
  });


  _task1.await;
  _task2.await;
  get_param.await;

  Ok(())

    // loop {
    //   async_std::task::sleep(Duration::from_millis(1000)).await;
    // }
}