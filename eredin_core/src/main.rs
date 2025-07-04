#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32h7::stm32h743;

fn init_clocks(p: &stm32h743::Peripherals) {
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

fn init_uart(p: &stm32h743::Peripherals) {
    // Enable USART3 and GPIOB clocks
    p.RCC.apb1lenr().modify(|_, w| w.usart3en().set_bit());
    p.RCC.ahb4enr().modify(|_, w| w.gpioben().set_bit());
    
    // Configure PB10 (TX) and PB11 (RX) for USART3
    p.GPIOB.moder().modify(|_, w| {
        w.moder10().alternate()
         .moder11().alternate()
    });
    
    p.GPIOB.afrh().modify(|_, w| {
        w.afr10().af7()  // USART3_TX
         .afr11().af7()  // USART3_RX
    });

    p.GPIOB.ospeedr().modify(|_, w| {
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

fn init_leds(p: &stm32h743::Peripherals) {
    // Enable GPIOA
    p.RCC.ahb4enr().modify(|_, w| w.gpioaen().set_bit());
    
    // Configure PA0 as output 
    p.GPIOA.moder().modify(|_, w| w.moder0().output());
    p.GPIOA.otyper().modify(|_, w| w.ot0().push_pull());
    p.GPIOA.ospeedr().modify(|_, w| w.ospeedr0().low_speed());
    p.GPIOA.pupdr().modify(|_, w| w.pupdr0().floating());

    // Configure PA1 as output
    p.GPIOA.moder().modify(|_, w| w.moder1().output());
    p.GPIOA.otyper().modify(|_, w| w.ot1().push_pull());
    p.GPIOA.ospeedr().modify(|_, w| w.ospeedr1().low_speed());
    p.GPIOA.pupdr().modify(|_, w| w.pupdr1().floating());

    // Configure PA2 as output
    p.GPIOA.moder().modify(|_, w| w.moder2().output());
    p.GPIOA.otyper().modify(|_, w| w.ot2().push_pull());
    p.GPIOA.ospeedr().modify(|_, w| w.ospeedr2().low_speed());
    p.GPIOA.pupdr().modify(|_, w| w.pupdr2().floating());
}

// Write funtions for USART3
fn usart3_write(p: &stm32h743::Peripherals, byte: u8) {
    // Wait until TXE (Transmit Data Register Empty) is set
    while p.USART3.isr().read().txe().bit_is_clear() {}
    
    // Write byte to transmit data register
    p.USART3.tdr().write(|w| unsafe { w.tdr().bits(byte as u16) });
}

fn usart3_print(p: &stm32h743::Peripherals, message: &str) {
    for byte in message.bytes() {
        usart3_write(p, byte);
    }
}

// Blocking read function
fn usart3_bread_byte(p: &stm32h743::Peripherals) -> u8 {
    // Wait until RXNE (Receive Data Register Not Empty) is set
    while p.USART3.isr().read().rxne().bit_is_clear() {}
    
    // Read byte from receive data register
    p.USART3.rdr().read().rdr().bits() as u8
}

fn usart3_bread_array(p: &stm32h743::Peripherals, buffer: &mut [u8]) -> usize {
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

fn init_timers(p: &stm32h743::Peripherals) {
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
    let p = stm32h743::Peripherals::take().unwrap();
    
    //init_clocks(&p); // Clock to 480 MHz
    init_leds(&p);
    init_uart(&p);
    //init_timers(&p);

    p.GPIOA.bsrr().write(|w| w.bs0().set_bit());
    p.GPIOA.bsrr().write(|w| w.bs1().set_bit());
    p.GPIOA.bsrr().write(|w| w.bs2().set_bit());
    
    loop {
      usart3_print(&p, "Hello from STM32H743!\r\n");
      eredin::core::delay(1000);
      p.GPIOA.bsrr().write(|w| w.br0().set_bit());
      p.GPIOA.bsrr().write(|w| w.bs1().set_bit());
      p.GPIOA.bsrr().write(|w| w.bs2().set_bit());
      eredin::core::delay(1000);
      p.GPIOA.bsrr().write(|w| w.bs0().set_bit());
      p.GPIOA.bsrr().write(|w| w.br1().set_bit());
      p.GPIOA.bsrr().write(|w| w.bs2().set_bit());
      eredin::core::delay(1000);
      p.GPIOA.bsrr().write(|w| w.bs0().set_bit());
      p.GPIOA.bsrr().write(|w| w.bs1().set_bit());
      p.GPIOA.bsrr().write(|w| w.br2().set_bit());
    }
}
