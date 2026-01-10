use crate::*;
use rtic::Mutex;
use crate::app::basic_led;

pub async fn basic_led(con: basic_led::Context<'static>) {
  let mut led = con.shared.led_r;
  loop{
    led.lock(|led| {
      led.toggle();
    });

    Mono::delay(1000.millis()).await;
  }
}

