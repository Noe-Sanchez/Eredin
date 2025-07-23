#![deny(warnings)]
#![no_main]
#![no_std]

#[cfg(feature = "run-hitl")]
mod hitl_imports {
  use panic_rtt_target as _;
  //use rtt_target::{rprintln, rtt_init_print};
}
#[cfg(not(feature = "run-hitl"))]
use panic_halt as _;

pub mod eredin_types{
  pub struct Odometry {
    pub pose:     [f32; 7], // x, y, z, qw, qx, qy, qz
    pub velocity: [f32; 6], // vx, vy, vz, wx, wy, wz
  }
}


use rtic::app;
use rtic_monotonics::systick::prelude::*;

use core::fmt::Write;

use stm32h7xx_hal::{
                    prelude::*,
                    gpio::PA0,
                    gpio::PA1,
                    gpio::PA2,
                    gpio::Output,
                    gpio::PushPull,
                    stm32::USART3,
};


systick_monotonic!(Mono, 1000);

#[app(device = stm32h7xx_hal::pac, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
      led_r:    PA0<Output<PushPull>>,
      led_g:    PA1<Output<PushPull>>,
      led_b:    PA2<Output<PushPull>>,
      serial:   stm32h7xx_hal::serial::Serial<USART3>,
      dt:       u32, 
      odometry: eredin_types::Odometry, 
    }

    #[local]
    struct Local {
      read_data: [u8; 64],
      idx: u8,
      rtt_channel: Option<rtt_target::DownChannel>,
    }

    #[init]
    fn init(con: init::Context) -> (Shared, Local){
      
      #[cfg(feature = "run-hitl")]
      let rtt_channel: Option<rtt_target::DownChannel>; 
      #[cfg(not(feature = "run-hitl"))]
      let rtt_channel: Option<rtt_target::DownChannel> = None; 


      #[cfg(feature = "run-hitl")]
      {
        //rtt_target::rtt_init_print!(); 
        //rtt_target::rtt_init_default!(); // New macro for bidirectional RTT
        // Default init macro for buffer size cfg
        let rtt_channels = rtt_target::rtt_init! {
          up: {
            0: {
              size: 512, 
              name: "Terminal"
            }
          }
          down: {
            0: {
              size: 64, 
              mode: rtt_target::ChannelMode::BlockIfFull,
              name: "Terminal"
            }
          }
        };
        rtt_channel = Some(rtt_channels.down.0);

        //rtt_target::rprintln!("RTT> Running in HITL mode");
        rtt_target::set_print_channel(rtt_channels.up.0);
        rtt_target::rprintln!("RTT> Running in HITL mode");
      }
    
      let dp = con.device; 
      let cp = con.core;

      Mono::start(cp.SYST, 400_000_000);

      // Constrain and Freeze power
      let pwr = dp.PWR.constrain();
      let pwrcfg = pwr.freeze();

      // Constrain and Freeze clock
      let rcc = dp.RCC.constrain();
      let ccdr = rcc.sys_ck(400.MHz()).freeze(pwrcfg, &dp.SYSCFG);

      // Enable GPIOA/B clocks
      let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
      let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

      // Pins for LEDs
      let mut led_r = gpioa.pa0.into_push_pull_output();
      let mut led_g = gpioa.pa1.into_push_pull_output();
      let mut led_b = gpioa.pa2.into_push_pull_output();

      // Set LEDs to off initially
      led_r.set_high();
      led_g.set_high();
      led_b.set_high();

      // Pins for USART3
      let tx = gpiob.pb10.into_alternate();
      let rx = gpiob.pb11.into_alternate();

      // Configure the serial peripheral.
      let mut serial = dp
          .USART3
          .serial((tx, rx), 115_200.bps(), ccdr.peripheral.USART3, &ccdr.clocks)
          .unwrap();

      serial.listen(stm32h7xx_hal::serial::Event::Rxne);

      let dt = 0; // Initialize dt

      // Greet before spinning
      writeln!(serial, "Eredin> Starting Scheduler...\r").unwrap();

      // Schedule software tasks
      //task_blink_led1::spawn().ok();
      //task_blink_led2::spawn().ok();
      //task_blink_led3::spawn().ok();
      task_telemetry::spawn().ok();
      //task_telemetry2::spawn().ok();
      task_compute_control::spawn().ok(); // Actual task lol
      let read_data: [u8; 64] = [0; 64]; 
      let idx: u8 = 0;

      let odometry: eredin_types::Odometry = eredin_types::Odometry {
        pose: [0.0; 7], 
        velocity: [0.0; 6], 
      };

      // Software task for rtt demo
      #[cfg(feature = "run-hitl")]
      {
        rtt_target::rprintln!("RTT> Starting RTT task...");
        task_rtt_receive::spawn().ok();
      }

      // Resources for tasks
      (
        Shared {
          led_r,
          led_g,
          led_b,
          serial,
          dt,
          odometry,
        },
        Local {
          read_data,
          idx,
          rtt_channel,
        },
      )
  }
    
  #[task(shared = [led_b, dt, odometry], local = [rtt_channel])]
  async fn task_rtt_receive(con: task_rtt_receive::Context) {
    let mut count = 0;
    let mut led = con.shared.led_b;
    let dt = con.shared.dt;
    let odometry = con.shared.odometry;
    let chan_opt = con.local.rtt_channel;
    // Just asign channels, since task wont be scheduled if not in HITL mode
    let channel = chan_opt.as_mut().expect("RTT channel not initialized");

    // Locking tuple
    let mut bq_t = (dt, odometry);

    let mut rtt_buffer: [u8; 64] = [0; 64]; 

    loop {
      led.lock(|led| {
          led.toggle();
      });
      bq_t.lock(|dt, _odometry| { // Try blocking read from RTT
        // Loop until we read something
        let read_bytes = channel.read(&mut rtt_buffer);

        //rtt_target::rprintln!("RTT> Read {} bytes: {:?}", read_bytes, rtt_buffer);
        rtt_target::rprintln!("RTT> Read {} bytes: ", read_bytes);
        *dt += 1; // Increment dt
        rtt_target::rprintln!("RTT> Count: {}", count);
      });
      //rtt_target::rprintln!("RTT> Count: {}", count);
      count += 1;
      Mono::delay(200.millis()).await;
    }
  }

  #[task(shared = [led_g, dt, odometry])]
  async fn task_compute_control(con: task_compute_control::Context) {
    let mut led = con.shared.led_g;
    let dt = con.shared.dt;
    let odometry = con.shared.odometry;
    let mut outputs: [f32; 4] = [0.0; 4]; 
    // Locking tuple
    let mut bq_t = (dt, odometry);
    loop {
      led.lock(|led| {
          led.toggle();
      });

      // Timestep lock, no holding on deploy, holding by rtt receive in HITL
      //(dt, odometry).lock(|dt, odometry| {
      bq_t.lock(|dt, odometry| {
        outputs[0] = odometry.pose[0] + (*dt as f32) * 0.001; // Example computation
        outputs[1] = odometry.pose[1] + (*dt as f32) * 0.001; 
        outputs[2] = odometry.pose[2] + (*dt as f32) * 0.001;

        #[cfg(feature = "run-hitl")]
        {
          rtt_target::rprintln!("Control> dt: {}", dt); 
          rtt_target::rprintln!("Control> Outputs: {:?}", outputs);
        }

      });

      Mono::delay(100.millis()).await;
    }
  }

  #[task(shared = [serial, led_r, odometry])]
  async fn task_telemetry(con: task_telemetry::Context) {
    let serial_if = con.shared.serial; 
    let mut led = con.shared.led_r;
    let odometry = con.shared.odometry;
    let mut bq_t = (serial_if, odometry); // Locking tuple
    loop {
      led.lock(|led| {
          led.toggle();
      });
      bq_t.lock(|serial, odometry| {
          //writeln!(serial, "Task1> Hello from RTIC Task1!\r").unwrap();
          writeln!(serial, "Telemetry> Odometry: pose: {:?}, velocity: {:?}\r", 
                   odometry.pose, odometry.velocity).unwrap();
        
      });

      Mono::delay(1000.millis()).await;
    }
  }

  #[task(binds = USART3, shared = [serial], local = [read_data, idx])]
  fn task_receive(con: task_receive::Context) {
    let mut serial_if = con.shared.serial; 

    let byte = serial_if.lock(|serial| {
      serial.read()
    });
    let read_data = con.local.read_data;
    let idx = con.local.idx; 
    if let Ok(byte) = byte {
      read_data[*idx as usize] = byte;
      *idx += 1;
    }
    if *idx >= read_data.len() as u8 || read_data[*idx as usize - 1] == 13 {
      *idx = 0; // Reset index if it exceeds buffer size
      serial_if.lock(|serial| {
        let read_str = core::str::from_utf8(&read_data[..]).unwrap_or("Invalid UTF-8"); 
        writeln!(serial, "TaskReceive> Read data: {}", read_str).unwrap();
      });
      // Clear the buffer
      for i in 0..read_data.len() {
        read_data[i] = 0; // Clear the buffer
      }
    }
  }

  /*#[task(shared = [led_r])]
  async fn task_blink_led1(con: task_blink_led1::Context) {
    let mut led = con.shared.led_r;
    loop {
      led.lock(|led| {
          led.toggle();
      });

      Mono::delay(500.millis()).await;
    }
  }
  #[task(shared = [led_g])]
  async fn task_blink_led2(con: task_blink_led2::Context) {
    let mut led = con.shared.led_g;
    loop {
      led.lock(|led| {
          led.toggle();
      });

      Mono::delay(1000.millis()).await;
    }
  }
  #[task(shared = [led_b])]
  async fn task_blink_led3(con: task_blink_led3::Context) {
    let mut led = con.shared.led_b;
    loop {
      led.lock(|led| {
          led.toggle();
      });

      Mono::delay(2000.millis()).await;
    }
  }*/

  /*#[task(shared = [serial])]
  async fn task_telemetry2(con: task_telemetry2::Context) {
    let mut serial_if = con.shared.serial; 
    loop {
      serial_if.lock(|serial| {
          writeln!(serial, "Task2> Hello from RTIC Task2!\r").unwrap();
      });

      Mono::delay(500.millis()).await;
    }
  }*/


}

//#[panic_handler]
//fn panic(_info: &core::panic::PanicInfo) -> ! {
//    loop {}
//}
