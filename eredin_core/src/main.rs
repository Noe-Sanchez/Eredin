#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32h7::stm32h755cm7;

use rtic_monotonics

fn init_clocks(p: &stm32h755cm7::Peripherals) {
    // Enable HSE (assumes 25MHz crystal)
    p.RCC.cr().modify(|_, w| w.hseon().set_bit());
    while !p.RCC.cr().read().hserdy().bit_is_set() {}
    
    // Configure PLL1 for 400MHz (conservative for CM7)
    p.RCC.pllckselr().modify(|_, w| unsafe {
        w.pllsrc().hse()
         .divm1().bits(5)  // 25MHz / 5 = 5MHz
    });
    
    p.RCC.pll1divr().modify(|_, w| unsafe {
        w.divn1().bits(80 - 1)  // 5MHz * 80 = 400MHz
         .divp1().bits(2 - 1)   // 400MHz / 2 = 200MHz sys_ck
    });
    
    // Set flash wait states
    p.FLASH.acr().modify(|_, w| unsafe {w.latency().bits(2) });
    
    // Enable PLL and switch
    p.RCC.cr().modify(|_, w| w.pll1on().set_bit());
    while !p.RCC.cr().read().pll1rdy().bit_is_set() {}
    
    p.RCC.cfgr().modify(|_, w| w.sw().pll1());
    while p.RCC.cfgr().read().sws().bits() != 0b011 {}
}

fn init_uart(p: &stm32h755cm7::Peripherals) {
    // Enable USART3 and GPIOD clocks
    p.RCC.apb1lenr().modify(|_, w| w.usart3en().set_bit());
    p.RCC.ahb4enr().modify(|_, w| w.gpioden().set_bit());
    
    // Configure PD8 (TX) and PD9 (RX) for USART3
    p.GPIOD.moder().modify(|_, w| {
        w.moder8().alternate()
         .moder9().alternate()
    });
    
    p.GPIOD.afrh().modify(|_, w| {
        w.afr8().af7()  // USART3_TX
         .afr9().af7()  // USART3_RX
    });

    p.GPIOD.ospeedr().modify(|_, w| {
        w.ospeedr8().very_high_speed()
         .ospeedr9().very_high_speed()
    });

    p.USART3.cr1().modify(|_, w| {
        w.ue().clear_bit()  // Disable USART for configuration
    });
    
    // Configure USART3 (assumes 200MHz APB1 clock)
    //p.USART3.brr().write(|w| unsafe { w.bits(200_000_000 / 115200) });
    //p.USART3.brr().write(|w| unsafe { w.bits(100_000_000 / 115200) });
    p.USART3.brr().write(|w| unsafe { w.bits(555) });

    p.USART3.cr1().modify(|_, w| {
        w.m0().clear_bit()  // 8 data bits
         .pce().clear_bit()  // No parity
         .te().set_bit()    // Enable transmitter
         .re().set_bit()    // Enable receiver
    });

    p.USART3.cr2().modify(|_, w| unsafe {
        w.stop().bits(0)    // 1 stop bit
    });

    p.USART3.cr1().modify(|_, w| {
        w.ue().set_bit()    // Enable USART
    });

}

fn init_leds(p: &stm32h755cm7::Peripherals) {
    // Enable GPIOB and GPIOE clocks
    p.RCC.ahb4enr().modify(|_, w| w.gpioben().set_bit().gpioeen().set_bit());
    
    // Configure PB0 (LED) as output
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
}

// Write funtions for USART3
fn usart3_write(p: &stm32h755cm7::Peripherals, byte: u8) {
    // Wait until TXE (Transmit Data Register Empty) is set
    while p.USART3.isr().read().txe().bit_is_clear() {}
    
    // Write byte to transmit data register
    p.USART3.tdr().write(|w| unsafe { w.tdr().bits(byte as u16) });
}

fn usart3_print(p: &stm32h755cm7::Peripherals, message: &str) {
    for byte in message.bytes() {
        usart3_write(p, byte);
    }
}

// Blocking read function
fn usart3_bread_byte(p: &stm32h755cm7::Peripherals) -> u8 {
    // Wait until RXNE (Receive Data Register Not Empty) is set
    while p.USART3.isr().read().rxne().bit_is_clear() {}
    
    // Read byte from receive data register
    p.USART3.rdr().read().rdr().bits() as u8
}

fn usart3_bread_array(p: &stm32h755cm7::Peripherals, buffer: &mut [u8]) -> usize {
    let mut count = 0;
    for byte in buffer.iter_mut() {
        *byte = usart3_bread_byte(p);
        count += 1;
        if *byte == b'\n' || *byte == b'\r' {
            break; // Stop on newline or carriage return
        }
    }
    count
}

fn init_timers(p: &stm32h755cm7::Peripherals) {
    // Enable TIM2 clock
    p.RCC.apb1lenr().modify(|_, w| w.tim2en().set_bit());
    
    // All for now, will try RTIC
    
}

// Core funcs
// Create eredin namespace
mod eredin {
  pub mod core {

    pub fn delay(milliseconds: u32) {
      for _ in 0..milliseconds {
          for _ in 0..480 {
              cortex_m::asm::nop();
          }
        }
    }

  }
}
    

#[entry]
fn main() -> ! {
    // Get peripherals
    let p = stm32h755cm7::Peripherals::take().unwrap();
    
    init_clocks(&p); // Clock to 480 MHz
    init_leds(&p);
    init_uart(&p);
    init_timers(&p);
    

    // Turn LED on
    p.GPIOB.bsrr().write(|w| w.br0().set_bit());

    // Turn PB14 on
    p.GPIOB.bsrr().write(|w| w.br14().set_bit());

    // Turn PE1 on
    p.GPIOE.bsrr().write(|w| w.br1().set_bit());
    
    loop {
    }
}
