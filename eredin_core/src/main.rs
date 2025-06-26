#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32h7::stm32h755cm7 as pac;
use core::fmt::Write;

struct Uart {
    usart: pac::USART3,
}

impl Uart {
    fn new(usart: pac::USART3) -> Self {
        Self { usart }
    }
    
    fn init(&mut self, rcc: &pac::RCC, gpiod: &pac::GPIOD) {
        // Enable clocks
        // Enable GPIOD clock
        rcc.ahb4enr().modify(|_, w| w.gpioden().set_bit());
        
        // Enable USART3 clock  
        rcc.apb1lenr().modify(|_, w| w.usart3en().set_bit());
        
        // Configure PD8 (TX) and PD9 (RX) as alternate function
        // PD8 - USART3_TX (AF7)
        gpiod.moder().modify(|_, w| w.moder8().alternate());
        gpiod.afrh().modify(|_, w| w.afr8().af7());
        gpiod.ospeedr().modify(|_, w| w.ospeedr8().very_high_speed());
        
        // PD9 - USART3_RX (AF7) 
        gpiod.moder().modify(|_, w| w.moder9().alternate());
        gpiod.afrh().modify(|_, w| w.afr9().af7());
        gpiod.ospeedr().modify(|_, w| w.ospeedr9().very_high_speed());
        
        // Configure USART3
        // Disable USART first
        self.usart.cr1().modify(|_, w| w.ue().clear_bit());
        
        // Set baud rate (assuming 64MHz APB1 clock, target 115200 baud)
        // BRR = fCK / baud_rate = 64000000 / 115200 ≈ 555 (0x22B)
        self.usart.brr().write(|w| unsafe { w.bits(555) });
        
        // Configure: 8 data bits, no parity, 1 stop bit
        self.usart.cr1().modify(|_, w| {
            w.m0().clear_bit()      // 8 data bits
             .pce().clear_bit()     // No parity
             .te().set_bit()        // Transmitter enable
             .re().set_bit()        // Receiver enable
        });
        
        self.usart.cr2().modify(|_, w| unsafe {
            w.stop().bits(0b00)     // 1 stop bit
        });
        
        // Enable USART
        self.usart.cr1().modify(|_, w| w.ue().set_bit());
    }
    
    fn write_byte(&mut self, byte: u8) {
        // Wait for transmit data register to be empty
        while self.usart.isr().read().txe().bit_is_clear() {}
        
        // Write data
        self.usart.tdr().write(|w| unsafe { w.tdr().bits(byte as u16) });
    }
    
    fn read_byte(&mut self) -> Option<u8> {
        if self.usart.isr().read().rxne().bit_is_set() {
            Some(self.usart.rdr().read().rdr().bits() as u8)
        } else {
            None
        }
    }
    
    fn write_str(&mut self, s: &str) {
        for byte in s.bytes() {
            self.write_byte(byte);
        }
    }
}

impl Write for Uart {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_str(s);
        Ok(())
    }
}

fn setup_system_clock(rcc: &pac::RCC) {
    // Basic clock setup - using HSI (64MHz) for simplicity
    // In production, you'd want to configure PLL for higher speeds
    
    // Ensure HSI is on
    rcc.cr().modify(|_, w| w.hsion().set_bit());
    while rcc.cr().read().hsirdy().bit_is_clear() {}
    
    // Select HSI as system clock
    rcc.cfgr().modify(|_, w| w.sw().hsi());
    while !rcc.cfgr().read().sws().is_hsi() {}
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();
    
    // Setup system clock
    setup_system_clock(&dp.RCC);
    
    // Initialize UART
    let mut uart = Uart::new(dp.USART3);
    uart.init(&dp.RCC, &dp.GPIOD);
    
    // Send initial message
    uart.write_str("STM32H755 UART Communication Started!\r\n");
   
    
    let mut counter = 0u32;
    
    loop {
        
        // Send periodic messages
        write!(uart, "Counter: {}\r\n", counter).ok();
        counter = counter.wrapping_add(1);
        
        // Check for received data and echo it back
        if let Some(received_byte) = uart.read_byte() {
            uart.write_str("Received: ");
            uart.write_byte(received_byte);
            uart.write_str("\r\n");
        }
        
        // Simple delay
        for _ in 0..1_000_000 {
            cortex_m::asm::nop();
        }
    }
}
