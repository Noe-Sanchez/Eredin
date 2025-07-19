#![deny(warnings)]
#![no_main]
#![no_std]

#[cfg(feature = "run-hitl")]
mod hitl_imports {
  use panic_rtt_target as _;
  //use rtt_target::{rprintln, rtt_init_print};
}
#[cfg(feature = "run-deploy")]
use panic_halt as _;

use rtic::app;
use rtic_monotonics::systick::prelude::*;

use core::fmt::Write;

use stm32h7xx_hal::{
                    //pac, 
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
      led_r: PA0<Output<PushPull>>,
      led_g: PA1<Output<PushPull>>,
      led_b: PA2<Output<PushPull>>,
      serial: stm32h7xx_hal::serial::Serial<USART3>,
    }

    #[local]
    struct Local {
      read_data: [u8; 64],
      idx: u8,
    }

    #[init]
    fn init(con: init::Context) -> (Shared, Local){
      #[cfg(feature = "run-hitl")]{
        rtt_target::rtt_init_print!(); 
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
      let led_r = gpioa.pa0.into_push_pull_output();
      let led_g = gpioa.pa1.into_push_pull_output();
      let led_b = gpioa.pa2.into_push_pull_output();

      // Pins for USART3
      let tx = gpiob.pb10.into_alternate();
      let rx = gpiob.pb11.into_alternate();

      // Configure the serial peripheral.
      let mut serial = dp
          .USART3
          .serial((tx, rx), 115_200.bps(), ccdr.peripheral.USART3, &ccdr.clocks)
          .unwrap();

      serial.listen(stm32h7xx_hal::serial::Event::Rxne);

      // Greet before spinning
      writeln!(serial, "Eredin> Starting Scheduler...\r").unwrap();

      // Schedule software tasks
      task_blink_led1::spawn().ok();
      task_blink_led2::spawn().ok();
      task_blink_led3::spawn().ok();
      task_print::spawn().ok();
      task_print2::spawn().ok();
      let read_data: [u8; 64] = [0; 64]; 
      let idx: u8 = 0;

      // Software task for rtt demo
      #[cfg(feature = "run-hitl")]
      {
        rtt_target::rprintln!("RTT> Starting RTT task...");
        task_rtt_count::spawn().ok();
      }

      // Resources for tasks
      (
        Shared {
          led_r,
          led_g,
          led_b,
          serial,
        },
        Local {
          read_data,
          idx,
        },
      )
  }
    
  #[task]
  async fn task_rtt_count(_con: task_rtt_count::Context) {
    let mut count = 0;
    loop {
      rtt_target::rprintln!("RTT> Count: {}", count);
      count += 1;
      Mono::delay(1000.millis()).await;
    }
  }

  #[task(shared = [led_r])]
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
  }
  #[task(shared = [serial])]
  async fn task_print(con: task_print::Context) {
    let mut serial_if = con.shared.serial; 
    loop {
      serial_if.lock(|serial| {
          writeln!(serial, "Task1> Hello from RTIC Task1!\r").unwrap();
      });

      Mono::delay(1000.millis()).await;
    }
  }
  #[task(shared = [serial])]
  async fn task_print2(con: task_print2::Context) {
    let mut serial_if = con.shared.serial; 
    loop {
      serial_if.lock(|serial| {
          writeln!(serial, "Task2> Hello from RTIC Task2!\r").unwrap();
      });

      Mono::delay(500.millis()).await;
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


}

//#[panic_handler]
//fn panic(_info: &core::panic::PanicInfo) -> ! {
//    loop {}
//}
