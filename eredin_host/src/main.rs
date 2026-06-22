use probe_rs::probe::list::Lister;
use probe_rs::Permissions;
use probe_rs::rtt::Rtt;
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Obtain a probe-rs session
    let lister = Lister::new();
    let probes = lister.list_all();
    
    if probes.is_empty() {
        return Err("No probes found".into());
    }
    
    let probe = probes[0].open()?;
    
    // Replace "somechip" with your actual target chip name (e.g., "STM32F103C8", "nRF52840_xxAA")
    //let mut session = probe.attach("somechip", Permissions::default())?;
    let mut session = probe.attach("STM32H743ZI", Permissions::default())?;
    
    // 2. Select a core
    let mut core = session.core(0)?;
    
    // 3. Attach to RTT
    // Get memory map from the target
    ////let memory_map = session.target().memory_map.clone();
    //let mut rtt = Rtt::attach(&mut core, &memory_map)?;
    let mut rtt = Rtt::attach(&mut core)?;
    
    // 4. Get the desired down channel
    let channel_number = 0;
    
    // Check if down channel exists
    if rtt.down_channels().len() > channel_number {
        // <<<<<<<<<<<<<< Previous POC >>>>>>>>>>>>>
        /*// Data to send to the target
        let data_to_send = b"Hello?";
        
        println!("Connected to RTT down channel {}", channel_number);
        
        // Loop to send data periodically
        loop {
            // Read on 0 down and write on 0 up
            let down_channel = rtt.down_channel(channel_number).unwrap();
            match down_channel.write(&mut core, data_to_send) {
                Ok(bytes_written) => {
                    println!("Wrote {} bytes to RTT down channel {}", bytes_written, channel_number);
                    println!("Sent data: {}", String::from_utf8_lossy(data_to_send));
                    
                    if bytes_written < data_to_send.len() {
                        println!("Warning: Only wrote {} of {} bytes (buffer might be full)",
                                 bytes_written, data_to_send.len());
                    }
                }
                Err(e) => {
                    eprintln!("Error writing to RTT: {}", e);
                    // Optionally break on error
                    // break;
                }
            }
            // Read from up channel 0
            let up_channel = rtt.up_channel(1).unwrap();
            let mut buffer = [0u8; 64];
            match up_channel.read(&mut core, &mut buffer) {
                Ok(bytes_read) => {
                    if bytes_read > 0 {
                        let received_data = &buffer[..bytes_read];
                        let received_str = String::from_utf8_lossy(received_data);
                        println!("Received {} bytes from RTT up channel 1: {}", bytes_read, received_str
                        );
                    }
                }
                Err(e) => {
                    eprintln!("Error reading from RTT: {}", e);
                }
              }
            thread::sleep(Duration::from_secs(2)); 


        }*/
        // <<<<<<<<<<<<<< End Previous POC >>>>>>>>>>>>>

        // New poc, send 7 f32 values to down channel 0 and read from up channel 1
        // Data to send to the target
        let mut data_to_send: [u8; 35] = [0; 35]; // 28 for 7 f32 values, 3 for header and 2 separators, 4 for msg_counter
        let mut odometry_values: [f32; 7] = [1.1, 2.2, -1.1, 1.0, 0.0, 0.0, 0.0]; // Random linear pose, unit quaternion 
        // We should send 35 bytes, with header, counter, separator, position, separator, quaternion
        // Format: @ <counter> $ <pos_x> <pos_y> <pos_z> $ <quat_x> <quat_y> <quat_z> <quat_w>
        
        let mut msg_counter: u32 = 1;
        let mut received_flag: bool;
        
        println!("Connected to RTT down channel {}", channel_number);

        let time = std::time::Instant::now();

        // Loop to send data periodically
        loop {
          
            // Do a sine wave on quaternion z and w for testing
            let t = time.elapsed().as_secs_f32();
            odometry_values[3] = (2.0*3.14*t).sin(); // quat_z
            odometry_values[6] = (2.0*3.14*t).cos(); // quat_w
            
            // Print odometry values for debugging
            println!("Preparing to send odometry values: {:?}", odometry_values);

            // Prepare data to send
            data_to_send[0] = b'@'; // Header
            data_to_send[1..5].copy_from_slice(&msg_counter.to_le_bytes()); // Message counter
            data_to_send[5] = b'$'; // Separator
            data_to_send[6..10].copy_from_slice(&odometry_values[0].to_le_bytes()); // pos_x
            data_to_send[10..14].copy_from_slice(&odometry_values[1].to_le_bytes()); // pos_y
            data_to_send[14..18].copy_from_slice(&odometry_values[2].to_le_bytes()); // pos_z
            data_to_send[18] = b'$'; // Separator
            data_to_send[19..23].copy_from_slice(&odometry_values[3].to_le_bytes()); // quat_x
            data_to_send[23..27].copy_from_slice(&odometry_values[4].to_le_bytes()); // quat_y
            data_to_send[27..31].copy_from_slice(&odometry_values[5].to_le_bytes()); // quat_z
            data_to_send[31..35].copy_from_slice(&odometry_values[6].to_le_bytes()); // quat_w
            
            // Read from up channel 1, we should read the a quaternion back, but also the counter to acknowledge reception
            // The data has the same format, but without position
            
            // Well loop until we get ack 
            received_flag = false;
            while !received_flag {
                // First write data, to ensure target as a way to read
                // Write to down channel 0
                let down_channel = rtt.down_channel(channel_number).unwrap();
                match down_channel.write(&mut core, &data_to_send) {
                    Ok(bytes_written) => {
                        println!("Wrote {} bytes to RTT down channel {}", bytes_written, channel_number);
                        //println!("Sent odometry data: position=({:.2}, {:.2}, {:.2}), orientation=({:.2}, {:.2}, {:.2}, {:.2})",
                        //         odometry_values[0], odometry_values[1], odometry_values[2],
                        //         odometry_values[3], odometry_values[4], odometry_values[5], odometry_values[6]);
                        println!("Sent odometry data: {:?}, with message counter {}", &odometry_values, msg_counter);
                        if bytes_written < data_to_send.len() {
                            println!("Warning: Only wrote {} of {} bytes (buffer might be full)",
                                     bytes_written, data_to_send.len());
                        }
                    }
                    Err(e) => {
                        eprintln!("Error writing to RTT: {}", e);
                        // Optionally break on error
                        // break;
                    }
                }

                // Sleep a bit to let target process
                thread::sleep(Duration::from_millis(10));

                let up_channel = rtt.up_channel(1).unwrap();
                let mut reception_buff: [u8; 100] = [0; 100]; //Packet is 1 header + 4 for counter + 1 separator + 16 for actuators 
                match up_channel.read(&mut core, &mut reception_buff) {
                    Ok(bytes_read) => {
                      // Parse buffer in search for valid message
                      let mut index = 0;
                      while index + 22 <= bytes_read { 
                          if reception_buff[index] == b'@' && reception_buff[index + 5] == b'$' {
                              // Potential valid message found, parse it
                              let recv_counter = u32::from_le_bytes([
                                  reception_buff[index + 1],
                                  reception_buff[index + 2],
                                  reception_buff[index + 3],
                                  reception_buff[index + 4],
                              ]);
                              
                              // Check if counter matches
                              // If matches, set received_flag to true
                              if recv_counter == msg_counter {
                                  received_flag = true;
                                  println!("Acknowledged reception of message counter {}", msg_counter);
                                  // Parse quaternion now
                                  let mut quat: [f32; 4] = [0.0; 4];
                                  for i in 0..4 {
                                      quat[i] = f32::from_le_bytes([
                                          reception_buff[index + 6 + i*4],
                                          reception_buff[index + 7 + i*4],
                                          reception_buff[index + 8 + i*4],
                                          reception_buff[index + 9 + i*4],
                                      ]);
                                  }
                                  println!("Received quaternion from target: ({:.4}, {:.4}, {:.4}, {:.4})",
                                           quat[0], quat[1], quat[2], quat[3]);
                              } else {
                                  //println!("Received counter {} does not match sent counter {}", recv_counter, msg_counter);
                                  // Sleep a bit before next read
                                  thread::sleep(Duration::from_millis(10));
                              }
                              // Move index forward
                              index += 22; 
                          } else {
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
        eprintln!("RTT down channel {} not found. Check target firmware configuration.", channel_number);
        return Err(format!("RTT down channel {} not available", channel_number).into());
    }
}
