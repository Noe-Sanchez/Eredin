use crate::*;
use rtic::Mutex;
use rtt_target::rprintln;
use crate::app::basic_led;
use crate::app::task_rtt_receive;

pub async fn basic_led(con: basic_led::Context<'static>) {
  let mut led = con.shared.led_r;
  loop{
    led.lock(|led| {
      led.toggle();
    });

    Mono::delay(1000.millis()).await;
  }
}

pub async fn task_rtt_receive(con: task_rtt_receive::Context<'static>) {
  let chan_down = con.local.rtt_down_channel;
  let chan_up   = con.local.rtt_up_channel;
  let mut led  = con.shared.led_b;
  let _odom = con.shared.odometry;

  // Just asign channels, since task wont be scheduled if not in HITL mode
  Mono::delay(3000.millis()).await;
  rprintln!("RTT trying to initialize channel...");
  led.lock(|led| {
    led.set_low();
  });
  let channel_down = chan_down.as_mut().expect("RTT channel down not initialized");
  let channel_up   = chan_up.as_mut().expect("RTT channel up not initialized");
  led.lock(|led| {
    led.set_high();
  });
  rprintln!("RTT channel initialized.");

  loop {
   
    // THIS WORKS
    /*let mut buffer = [0u8; 64];
    led.lock(|led| {
      let read_bytes = channel_down.read(&mut buffer);
      if read_bytes > 0 {
        
        // Add ' World!' to received message and print (without string)
        for i in 0..read_bytes {
          //rprint!("{}", buffer[i] as char);
          // Write to up channel
          channel_up.write(&buffer[i..i+1]);
        }
        //rprint!(" World!\n");
        channel_up.write(b" World!\n");


        led.toggle();
      }
    });*/

    // Compute axis angle representation for now, send twice in a 6x1  
    // First read odometry, will come 7x1 with f32 bytes, pose and quaternion, with '@' header and '$' separator
    
    // Read from down channel
    let mut buffer = [0u8; 64];
    let read_bytes = channel_down.read(&mut buffer);
    // Check indexes for header
    if read_bytes > 0 {
      // Parse message
      for i in 0..read_bytes {
        if buffer[i] == b'@' {
          // Header found, parse quaternion bytes
          let mut quat_bytes = [0.0f32; 4];

          // Skip '@' and the 12 bytes of the linear pose
          let quat_start = i + 1 + 12;
          for j in 0..4 {
            let byte_index = quat_start + j * 4;
            if byte_index + 4 <= read_bytes {
              quat_bytes[j] = f32::from_le_bytes([
                buffer[byte_index],
                buffer[byte_index + 1],
                buffer[byte_index + 2],
                buffer[byte_index + 3],
              ]);
            }
          }

          // Here we would compute the axis-angle from the quaternion
          // For simplicity, just send back the quaternion as is for now
          for j in 0..4 {
            channel_up.write(&quat_bytes[j].to_le_bytes());
          }
          channel_up.write(b"\n");
          led.lock(|led| {
            led.toggle();
          });
        }
      }
    }  

    Mono::delay(500.millis()).await;
  }
}
