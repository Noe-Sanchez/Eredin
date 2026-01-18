use crate::*;
use rtic::Mutex;
use rtt_target::rprintln;
use rtt_target::rprint;
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
  let mut led = con.shared.led_b;
  let chan_opt = con.local.rtt_channel;
  // Just asign channels, since task wont be scheduled if not in HITL mode
  Mono::delay(3000.millis()).await;
  rprintln!("RTT trying to initialize channel...");
  led.lock(|led| {
    led.set_low();
  });
  let channel = chan_opt.as_mut().expect("RTT channel not initialized");
  led.lock(|led| {
    led.set_high();
  });
  rprintln!("RTT channel initialized.");

  loop {
    //led.lock(|led| {
    //  led.toggle();
    //  rprintln!("Toggled Blue LED");
    //});

    // Use read() func
    //let mut buffer = [0u8; 64];
    // Super simple blocking read
    //let read_bytes = channel.read(&mut buffer);
    //if read_bytes > 0 {
    //  rprintln!("RTT Received ({} bytes):", read_bytes); 
    //}
    
    let mut buffer = [0u8; 64];
    led.lock(|led| {
      let read_bytes = channel.read(&mut buffer);
      if read_bytes > 0 {
        //rprintln!("RTT Received ({} bytes):", read_bytes); 

        //for i in 0..read_bytes {
        //  rprint!("{}", buffer[i] as char);
        //}
        
        // Add ' World!' to received message and print (without string)
        for i in 0..read_bytes {
          rprint!("{}", buffer[i] as char);
        }
        rprint!(" World!\n");


        led.toggle();
      }
    });


    Mono::delay(500.millis()).await;
  }
}
