#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32h7::stm32h755cm7;

#[entry]
fn main() -> ! {
    // Get peripherals
    let p = stm32h755cm7::Peripherals::take().unwrap();
    
    // Enable GPIOB clock (LED is on PB0 for STM32H755-Nucleo)
    p.RCC.ahb4enr().modify(|_, w| w.gpioben().set_bit());

    // Enable GPIOE clock
    p.RCC.ahb4enr().modify(|_, w| w.gpioeen().set_bit());
    
    // Configure PB0 as output
    p.GPIOB.moder().modify(|_, w| w.moder0().output());
    p.GPIOB.otyper().modify(|_, w| w.ot0().push_pull());
    p.GPIOB.ospeedr().modify(|_, w| w.ospeedr0().low_speed());
    p.GPIOB.pupdr().modify(|_, w| w.pupdr0().floating());

    // Configure PB14 as output
    p.GPIOB.moder().modify(|_, w| w.moder14().output());
    p.GPIOB.otyper().modify(|_, w| w.ot14().push_pull());
    p.GPIOB.ospeedr().modify(|_, w| w.ospeedr14().low_speed());
    p.GPIOB.pupdr().modify(|_, w| w.pupdr14().floating());

    // Configure PE1 as output
    p.GPIOE.moder().modify(|_, w| w.moder1().output());
    p.GPIOE.otyper().modify(|_, w| w.ot1().push_pull());
    p.GPIOE.ospeedr().modify(|_, w| w.ospeedr1().low_speed());
    p.GPIOE.pupdr().modify(|_, w| w.pupdr1().floating());

    // Turn LED on
    p.GPIOB.bsrr().write(|w| w.bs0().set_bit());

    // Turn PB14 on
    p.GPIOB.bsrr().write(|w| w.bs14().set_bit());

    // Turn PE1 on
    p.GPIOE.bsrr().write(|w| w.bs1().set_bit());
    
    loop {
        // Turn LED on
        p.GPIOB.bsrr().write(|w| w.bs0().set_bit());
        p.GPIOB.bsrr().write(|w| w.br14().set_bit());
        p.GPIOE.bsrr().write(|w| w.br1().set_bit());
        
        // Simple delay
        for _ in 0..25_000 {
            cortex_m::asm::nop();
        }

        p.GPIOB.bsrr().write(|w| w.br0().set_bit());
        p.GPIOB.bsrr().write(|w| w.bs14().set_bit());
        p.GPIOE.bsrr().write(|w| w.br1().set_bit());
        
        // Simple delay
        for _ in 0..25_000 {
            cortex_m::asm::nop();
        }

        p.GPIOB.bsrr().write(|w| w.br0().set_bit());
        p.GPIOB.bsrr().write(|w| w.br14().set_bit());
        p.GPIOE.bsrr().write(|w| w.bs1().set_bit());

        // Simple delay
        for _ in 0..25_000 {
            cortex_m::asm::nop();
        }
    }
}
