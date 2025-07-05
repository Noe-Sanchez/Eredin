#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;

use stm32h7xx_hal::{pac, prelude::*};

use core::fmt::Write;

//use stm32h7xx_hal::nb::block;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

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

    let tx = gpiob.pb10.into_alternate();
    let rx = gpiob.pb11.into_alternate();

    let mut delay = cp.SYST.delay(ccdr.clocks);

    // Configure the serial peripheral.
    let serial = dp
        .USART3
        .serial((tx, rx), 115_200.bps(), ccdr.peripheral.USART3, &ccdr.clocks)
        .unwrap();

    let (mut tx, mut _rx) = serial.split();

    // core::fmt::Write is implemented for tx.
    loop {
        writeln!(tx, "Hello, world! \r").unwrap();
        delay.delay_ms(125_u32);
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
