#![deny(warnings)]
#![no_main]
#![no_std]

//use cortex_m_rt::entry;

//use stm32h7xx_hal::{pac, prelude::*};

//use core::fmt::Write;

//use stm32h7xx_hal::nb::block;

//#[entry]
//fn main() -> ! {

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
      //tx:    Tx<USART3>,
      //rx:    Rx<USART3>,
      //tx:    PB10<Alternate<7>>,
      //rx:    PB11<Alternate<7>>,
      serial: stm32h7xx_hal::serial::Serial<USART3>,
      //serial: stm32h7xx_hal::serial::Serial<pac::USART3, (PB10<Alternate>, PB11<Alternate>)>,
    }

    #[local]
    struct Local {}

    #[init]
    //fn init(con: init::Context) -> (Shared, Local, init::Monotonics){
    fn init(con: init::Context) -> (Shared, Local){
    
      //let dp = pac::Peripherals::take().unwrap();
      //let cp = cortex_m::Peripherals::take().unwrap();
      //let dp = con.device::Peripherals::take().unwrap();
      //let cp = cortex_m::Peripherals::take().unwrap();
      let dp = con.device; 
      let cp = con.core;

      //Mono::start(con.core.SYST, 400_000_000);
      Mono::start(cp.SYST, 400_000_000);

      // Constrain and Freeze power
      let pwr = dp.PWR.constrain();
      let pwrcfg = pwr.freeze();

      // Constrain and Freeze clock
      let rcc = dp.RCC.constrain();
      //let ccdr = rcc.sys_ck(160.MHz()).freeze(pwrcfg, &dp.SYSCFG);
      let ccdr = rcc.sys_ck(400.MHz()).freeze(pwrcfg, &dp.SYSCFG);

      // Acquire the GPIOC peripheral. This also enables the clock for
      // GPIOC in the RCC register.
      let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
      let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

      let led_r = gpioa.pa0.into_push_pull_output();
      let led_g = gpioa.pa1.into_push_pull_output();
      let led_b = gpioa.pa2.into_push_pull_output();

      let tx = gpiob.pb10.into_alternate();
      let rx = gpiob.pb11.into_alternate();


      //let mut delay = cp.SYST.delay(ccdr.clocks);

      // Configure the serial peripheral.
      let serial = dp
          .USART3
          .serial((tx, rx), 115_200.bps(), ccdr.peripheral.USART3, &ccdr.clocks)
          .unwrap();


      //let (mut tx, mut _rx) = serial.split();
      
      // Write a message to the serial port
      //tx.write(b'H').unwrap();

      

      // Scheduling

      /*task_blink_led1::spawn_after(200.millis()).unwrap();
      task_blink_led2::spawn_after(500.millis()).unwrap();
      task_blink_led3::spawn_after(1000.millis()).unwrap();*/
      task_blink_led1::spawn().ok();
      task_blink_led2::spawn().ok();
      task_blink_led3::spawn().ok();
      task_print::spawn().ok();


      (
        Shared {
          led_r,
          led_g,
          led_b,
          serial,
          //serial: serial,
        },
        Local {},
        //init::Monotonics(rr),
      )
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
          // Write a message to the serial port
          /*serial.write(b'H').unwrap();
          serial.write(b'e').unwrap();
          serial.write(b'l').unwrap();
          serial.write(b'l').unwrap();
          serial.write(b'o').unwrap();
          serial.write(b'\r').unwrap();
          serial.write(b'\n').unwrap();*/
          writeln!(serial, "Hello from RTIC!\r").unwrap();
      });

      Mono::delay(1000.millis()).await;
    }
  }


}

//#[panic_handler]
//fn panic(_info: &core::panic::PanicInfo) -> ! {
//    loop {}
//}
