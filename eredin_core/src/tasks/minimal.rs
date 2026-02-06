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
  //Mono::delay(3000.millis()).await;
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
  
  // We should process messages sequentially, if message is the same, dont process or send
  let mut msg_counter: u32 = 0;

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

    // First read odometry, will come 7x1 with f32 bytes, pose and quaternion, with '@' header and '$' separator
    // Additionally, message will have another separator and a u32 counter at the beginning
    // Example: @ <counter> $ <px> <py> <pz> $ <qx> <qy> <qz> <qw> 
    // Total is 35 bytes
    
    // Read from down channel
    let mut buffer = [0u8; 120];
    let read_bytes = channel_down.read(&mut buffer);
    // Check indexes for header
    
    if read_bytes >= 35 { // Need at least a full message

      led.lock(|led| led.toggle());
      
      let mut i = 0;
      while i + 35 <= read_bytes {
        // Check for valid message header and separators
        if buffer[i] == b'@' && buffer[i + 5] == b'$' && buffer[i + 18] == b'$' {
          // Parse counter
          let counter = u32::from_le_bytes([
            buffer[i + 1],
            buffer[i + 2],
            buffer[i + 3],
            buffer[i + 4],
          ]);
           
          // Skip if already processed
          if counter <= msg_counter {
            i += 35;
            continue;
          }
                    
          msg_counter = counter;
                    
          // Parse quaternion (skip: @ + counter(4) + $(1) + position(12) + $(1) = 19)
          let quat_start = i + 19;
          let mut quat_bytes = [0.0f32; 4];
          for j in 0..4 {
              quat_bytes[j] = f32::from_le_bytes([
                  buffer[quat_start + j * 4],
                  buffer[quat_start + j * 4 + 1],
                  buffer[quat_start + j * 4 + 2],
                  buffer[quat_start + j * 4 + 3],
              ]);
          }
          
          // Send response
          let mut send_buffer = [0u8; 22];
          send_buffer[0] = b'@';
          send_buffer[1..5].copy_from_slice(&counter.to_le_bytes());
          send_buffer[5] = b'$';
          for j in 0..4 {
              send_buffer[6 + j * 4..10 + j * 4]
                  .copy_from_slice(&quat_bytes[j].to_le_bytes());
          }
          channel_up.write(&send_buffer); 

          i += 35; // Move past processed message
        } else {
          i += 1; // Invalid message, try next byte
        }
      }
    }

    Mono::delay(10.millis()).await;
  }
}
