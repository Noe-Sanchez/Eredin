#![deny(warnings)]
use probe_rs::probe::list::Lister;
use probe_rs::Permissions;
use probe_rs::rtt::Rtt;
use std::thread;
use std::time::Duration;
use rclrs::*;
use std::sync::{Arc, Mutex};
use anyhow::Error;

fn eredin_pack_odom(counter: u32, odom: &nav_msgs::msg::Odometry) -> [u8; 35] {
    let mut data: [u8; 35] = [0; 35];
    
    data[0] = b'@'; // Header
    data[1..5].copy_from_slice(&counter.to_le_bytes()); // Message counter
    data[5] = b'$'; // Separator
    //data[6..10].copy_from_slice( &odom.pose.pose.position.x.to_le_bytes()); // pos_x
    //data[10..14].copy_from_slice(&odom.pose.pose.position.y.to_le_bytes()); // pos_y
    //data[14..18].copy_from_slice(&odom.pose.pose.position.z.to_le_bytes()); // pos_z
    // Convert each position component to f32 and copy to data
    let pos_x_f32 = odom.pose.pose.position.x as f32;
    let pos_y_f32 = odom.pose.pose.position.y as f32;
    let pos_z_f32 = odom.pose.pose.position.z as f32;
    data[6..10].copy_from_slice(&pos_x_f32.to_le_bytes());
    data[10..14].copy_from_slice(&pos_y_f32.to_le_bytes());
    data[14..18].copy_from_slice(&pos_z_f32.to_le_bytes());
    data[18] = b'$'; // Separator
    let quat_x_f32 = odom.pose.pose.orientation.x as f32;
    let quat_y_f32 = odom.pose.pose.orientation.y as f32;
    let quat_z_f32 = odom.pose.pose.orientation.z as f32;
    let quat_w_f32 = odom.pose.pose.orientation.w as f32;
    data[19..23].copy_from_slice(&quat_x_f32.to_le_bytes()); // quat_x
    data[23..27].copy_from_slice(&quat_y_f32.to_le_bytes());
    data[27..31].copy_from_slice(&quat_z_f32.to_le_bytes()); // quat_z
    data[31..35].copy_from_slice(&quat_w_f32.to_le_bytes());
    //data[19..23].copy_from_slice(&odom.pose.pose.orientation.x.to_le_bytes()); // quat_x
    //data[23..27].copy_from_slice(&odom.pose.pose.orientation.y.to_le_bytes()); // quat_y
    //data[27..31].copy_from_slice(&odom.pose.pose.orientation.z.to_le_bytes()); // quat_z
    //data[31..35].copy_from_slice(&odom.pose.pose.orientation.w.to_le_bytes()); // quat_w
                                                                               
    return data;
}

// Well imitate frame matching as done by mavlink
// expected_counter is the expected message counter, data is the reception buffer, stx is the current buffer index, return Actuators message if valid, else None
fn eredin_unpack_control(expected_counter: u32, data: &[u8], stx: usize) -> Option<actuator_msgs::msg::Actuators> {
  // Framing
  if data[stx] == b'@' && data[stx + 5] == b'$' {
    // Potential valid message found, parse it
    let recv_counter = u32::from_le_bytes([
      data[stx + 1],
      data[stx + 2],
      data[stx + 3],
      data[stx + 4],
    ]);

    // Check if counter matches
    if recv_counter == expected_counter {
      // Parse actuator commands
      let mut actuators = actuator_msgs::msg::Actuators::default();
      let mut actuator_values: [f32; 4] = [0.0; 4];
      for i in 0..4 {
        // Grab 4 bytes for each actuator command, starting from stx + 6
        actuator_values[i] = f32::from_le_bytes([
          data[stx + 6 + i*4],
          data[stx + 7 + i*4],
          data[stx + 8 + i*4],
          data[stx + 9 + i*4],
        ]);
      }
      // Resize velocity field
      actuators.velocity.resize(4, 0.0);
      for i in 0..4 {
        actuators.velocity[i] = actuator_values[i] as f64;
      }

      return Some(actuators);
    } else {
      println!("Received counter {} does not match expected counter {}", recv_counter, expected_counter);
      return None;
    }
  }else {
    return None;
  }
}

fn main() -> Result<(), Error> {
  // 1. Obtain a probe-rs session
  let lister = Lister::new();
  let probes = lister.list_all();
  
  if probes.is_empty() {
    return Err(Error::msg("No probes found"));
  }

  // Weve got a probe, we can init ros2
  let ctx          = Context::default_from_env()?;
  let mut executor = ctx.create_basic_executor();
  let node         = executor.create_node("eredin_bridge_node")?;

  let control_pub  = node.create_publisher::<actuator_msgs::msg::Actuators>("/x500_1/command/motor_speedpija")?;

  // Make Arc and Mutex for odom, since well access on spin and via thread
  let current_odom    = nav_msgs::msg::Odometry::default();
  let current_odom    = Arc::new(Mutex::new(current_odom));
  let odom_clone_ros  = Arc::clone(&current_odom); // Clone for ros spin
  let odom_clone_main = Arc::clone(&current_odom); // Clone for rtt

  // Register our odom subscriber
  let ros_worker       = node.create_worker::<usize>(0);
  let _odom_subscriber = ros_worker.create_subscription::<nav_msgs::msg::Odometry, _>("/model/x500_1/odometry",
      move |msg: nav_msgs::msg::Odometry| {
          // Update current_odom with the received message
          let mut odom_lock = odom_clone_ros.lock().expect("Failed to lock odom mutex");
          *odom_lock = msg;
          //println!("Received odometry message: position=({:.2}, {:.2}, {:.2}), orientation=({:.2}, {:.2}, {:.2}, {:.2})",
          //         odom_lock.pose.pose.position.x,    odom_lock.pose.pose.position.y,    odom_lock.pose.pose.position.z,
          //         odom_lock.pose.pose.orientation.x, odom_lock.pose.pose.orientation.y, odom_lock.pose.pose.orientation.z, odom_lock.pose.pose.orientation.w);
      }
  )?;

  // Spawn ros thread now
  thread::spawn(move || {
    println!("ROS spin thread started");
    executor.spin(SpinOptions::default()).first_error().unwrap();
  });

  // Single core VC1 probe
  let probe = probes[0].open()?;
  let mut session = probe.attach("STM32H743ZI", Permissions::default())?;
  let mut core = session.core(0)?;

  // RTT on chan 0, use later, on thread
  let mut rtt = Rtt::attach(&mut core)?;

  // Rtt spin here and make ros spin on thread later
  if rtt.down_channels().len() > 0 { // CHANGE TO CHECK FOR SPECIFIC CHANNEL
    println!("Connected to RTT down channel");

    // Data to send to the target
    // Allow for unused assignment, since rrt write does not trigger the borrow checker to recognize the data as used
    #[allow(unused_assignments)]
    let mut data_to_send: [u8; 35] = [0; 35]; // 28 for 7 f32 values, 3 for header and 2 separators, 4 for msg_counter
    
    let mut msg_counter: u32 = 1;
    let mut received_flag: bool;

    let mut reception_buff: [u8; 100] = [0; 100]; //Packet is 1 header + 4 for counter + 1 separator + 16 for actuator commands 

    loop {
      
      {
        let odom_lock = odom_clone_main.lock().unwrap();
        data_to_send = eredin_pack_odom(msg_counter, &odom_lock);
      }

      // Well loop until we get ack 
      received_flag = false;
      while !received_flag {
        // First write data, to ensure target as a way to read
        // Write to down channel 0
        let down_channel = rtt.down_channel(0).unwrap();
        match down_channel.write(&mut core, &data_to_send) {
          Ok(bytes_written) => {
            println!("Wrote {} bytes to RTT down channel 0", bytes_written);
            println!("Sent odometry data with message counter {}", msg_counter);
            if bytes_written < data_to_send.len() {
              println!("Warning: Only wrote {} of {} bytes (buffer might be full)", bytes_written, data_to_send.len());
            }
          }
          Err(e) => {
            eprintln!("Error writing to RTT: {}", e);
          }
        }

        // Sleep a bit to let target process
        thread::sleep(Duration::from_millis(10));

        let up_channel   = rtt.up_channel(1).unwrap();
        match up_channel.read(&mut core, &mut reception_buff) {
          Ok(bytes_read) => {
            // Parse buffer in search for valid message
            let mut index: usize = 0;
            while index + 22 <= bytes_read {
              // Read with func
              if let Some(actuators) = eredin_unpack_control(msg_counter, &reception_buff, index) { 
                // Valid message received
                println!("Received valid control message with counter {}", msg_counter);
                //control_pub.publish(&actuators)?;
                // Check if ros ctx is ok
                if ctx.ok() {
                  //control_pub.publish(&actuators)?;
                  control_pub.publish(&actuators).expect("Failed to publish actuator command");
                } else {
                  println!("ROS context not ok, skipping publish");
                }
                received_flag = true;
                break; // Exit parsing loop
              }else {
                index += 1;
              }
            }
          }
          Err(e) => {
            eprintln!("Error reading from RTT: {}", e);
          }
        }
      }

      // Increment message counter
      msg_counter = msg_counter.wrapping_add(1);

      // Sleep 10 ms
      thread::sleep(Duration::from_millis(10));
    }
  } else {
    eprintln!("RTT down channel 0 not found. Check target firmware configuration.");
    return Err(Error::msg("RTT down channel 0 not available"));
  }

  // Allow unreachable code since we loop infinitely
  #[allow(unreachable_code)]
  return Ok(());

}
