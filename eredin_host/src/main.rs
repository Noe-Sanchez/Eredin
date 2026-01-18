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
        // Data to send to the target
        let data_to_send = b"Hello?";
        
        println!("Connected to RTT down channel {}", channel_number);
        
        // Loop to send data periodically
        loop {
            // 5. Write to the channel
            // Access the channel through down_channels()
            //let channel = &mut rtt.down_channels()[channel_number];
            //let channel = rtt.down_channels.as_mut().get_mut(channel_number).unwrap();  
            //let channel = Box::new(rtt.down_channels()).0.as_mut().unwrap();
            /*let channel = rtt.down_channel(0).unwrap();

            match channel.write(&mut core, data_to_send) {
                Ok(bytes_written) => {
                    println!("Wrote {} bytes to RTT down channel {}", bytes_written, channel_number);
                    
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
            
            thread::sleep(Duration::from_secs(2));*/    
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
            let up_channel = rtt.up_channel(0).unwrap();
            let mut buffer = [0u8; 64];
            match up_channel.read(&mut core, &mut buffer) {
                Ok(bytes_read) => {
                    if bytes_read > 0 {
                        let received_data = &buffer[..bytes_read];
                        let received_str = String::from_utf8_lossy(received_data);
                        println!("Received {} bytes from RTT up channel 0: {}", bytes_read, received_str
                        );
                    }
                }
                Err(e) => {
                    eprintln!("Error reading from RTT: {}", e);
                }
              }
            thread::sleep(Duration::from_secs(2)); 


        }
    } else {
        eprintln!("RTT down channel {} not found. Check target firmware configuration.", channel_number);
        return Err(format!("RTT down channel {} not available", channel_number).into());
    }
}
