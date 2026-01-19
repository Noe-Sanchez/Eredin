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
        let mut data_to_send: [u8; 30] = [0; 30]; // 7 f32 values
        let mut odometry_values: [f32; 7] = [1.1, 2.2, -1.1, 1.0, 0.0, 0.0, 0.0]; // Random linear pose, unit quaternion 
        // We should send 28 bytes, but start with '@' as a header, and '$' as separator between position and orientation
        // So the final data format is: [ '@', pos_x, pos_y, pos_z, '$', quat_x, quat_y, quat_z, quat_w ]
        
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
            for i in 0..3 {
                let bytes = odometry_values[i].to_le_bytes();
                data_to_send[1 + i*4..1 + (i+1)*4].copy_from_slice(&bytes);
            }
            data_to_send[13] = b'$'; // Separator
            for i in 0..4 {
                let bytes = odometry_values[3 + i].to_le_bytes();
                data_to_send[14 + i*4..14 + (i+1)*4].copy_from_slice(&bytes);
            }

            
            // Write to down channel 0
            let down_channel = rtt.down_channel(channel_number).unwrap();
            match down_channel.write(&mut core, &data_to_send) {
                Ok(bytes_written) => {
                    println!("Wrote {} bytes to RTT down channel {}", bytes_written, channel_number);
                    //println!("Sent odometry data: position=({:.2}, {:.2}, {:.2}), orientation=({:.2}, {:.2}, {:.2}, {:.2})",
                    //         odometry_values[0], odometry_values[1], odometry_values[2],
                    //         odometry_values[3], odometry_values[4], odometry_values[5], odometry_values[6]);
                    println!("Sent odometry data: {:?}", odometry_values);
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


            // Read from up channel 1, we should read only the quaternion back
            let up_channel = rtt.up_channel(1).unwrap();
            let mut buffer = [0u8; 16]; // Buffer for 4 f32 values
            match up_channel.read(&mut core, &mut buffer) {
                Ok(bytes_read) => {
                    if bytes_read > 0 {
                        let mut received_values: [f32; 4] = [0.0; 4];
                        for i in 0..4 { 
                            let byte_slice = &buffer[i*4..(i+1)*4];
                            received_values[i] = f32::from_le_bytes(byte_slice.try_into().unwrap());
                        }
                        println!("Received {} bytes from RTT up channel 1: {:?}", bytes_read, received_values);
                    }
                }
                Err(e) => {
                    eprintln!("Error reading from RTT: {}", e);
                }
            }


            thread::sleep(Duration::from_secs(1));
        }
    } else {
        eprintln!("RTT down channel {} not found. Check target firmware configuration.", channel_number);
        return Err(format!("RTT down channel {} not available", channel_number).into());
    }
}
