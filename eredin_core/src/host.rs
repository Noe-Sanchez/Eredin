use probe_rs::probe::list::Lister;
use probe_rs::Permissions;
use probe_rs::rtt::Rtt;
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Obtain a probe-rs session (see probe-rs documentation for details on more specific attachment)
    let lister = Lister::new();
    let probes = lister.list_all();
    let probe = probes[0].open()?;
    let mut session = probe.attach("somechip", Permissions::default())?; // Replace "somechip" with your target chip name
    
    // 2. Select a core
    let mut core = session.core(0)?;

    // 3. Attach to RTT
    let mut rtt = Rtt::attach(&mut core)?;

    // 4. Get the desired down channel
    if let Some(mut output_channel) = rtt.down_channel(0) {
        // Data to send to the target
        let data_to_send = b"Hello from the host!\n";
        
        // Loop to send data periodically
        loop {
            // 5. Write to the channel
            // The write method handles non-blocking writes and returns the number of bytes written.
            let bytes_written = output_channel.write(&mut core, data_to_send)?;
            
            println!("Wrote {} bytes to RTT down channel 0", bytes_written);

            // In a real application, you might want a more sophisticated loop 
            // and data handling logic, potentially across threads.
            thread::sleep(Duration::from_secs(2));
        }
    } else {
        println!("RTT down channel 0 not found. Check target firmware configuration.");
    }

    Ok(())
}

