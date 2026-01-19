#![deny(warnings)]
#![no_main]
#![no_std]

#[cfg(feature = "run-hitl")]
mod hitl_imports {
  use panic_rtt_target as _;
}
#[cfg(not(feature = "run-hitl"))]
use panic_halt as _;

// TODO: Move to external module
pub mod eredin_types{
  pub struct Odometry {
    pub pose:     [f32; 7], // x, y, z, qw, qx, qy, qz
    pub velocity: [f32; 6], // vx, vy, vz, wx, wy, wz
  }
}


// RTIC imports
use rtic::app;
use rtic_monotonics::systick::prelude::*;

use core::fmt::Write;

use stm32h7xx_hal::{
  prelude::*,
  gpio::PA0,
  gpio::PA1,
  gpio::PA2,
  //gpio::PF13,
  gpio::PG0,
  gpio::PF15,
  gpio::Output,
  gpio::PushPull,
  stm32::USART3,
  stm32::UART4,
  spi,
  spi::SpiExt,
};


// Eredin imports
pub mod tasks;
use crate::tasks::minimal::basic_led;
use crate::tasks::minimal::task_rtt_receive;
use rtt_target::ChannelMode;

systick_monotonic!(Mono, 1000);

#[app(device = stm32h7xx_hal::pac, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
      led_r:    PA0<Output<PushPull>>,
      led_g:    PA1<Output<PushPull>>,
      led_b:    PA2<Output<PushPull>>,
      serial1:   stm32h7xx_hal::serial::Serial<USART3>,
      _serial2:   stm32h7xx_hal::serial::Serial<UART4>,
      spi:      stm32h7xx_hal::spi::Spi<stm32h7xx_hal::stm32::SPI1, stm32h7xx_hal::spi::Enabled, u8>,
      // alternate 
      //cs_baro:  PF13<Output<PushPull>>, // Chip select for barometer
      cs_baro:  PG0<Output<PushPull>>, // Chip select for barometer on alternate pin
      cs_gyro:  PF15<Output<PushPull>>, // Chip select for barometer on alternate pin
      odometry: eredin_types::Odometry,
    }
    #[local]
    struct Local {
      rtt_down_channel: Option<rtt_target::DownChannel>,
      rtt_up_channel:   Option<rtt_target::UpChannel>,
    }

    #[init]
    fn init(con: init::Context) -> (Shared, Local){
      
      // Dirty as fuck, figure out later
      #[cfg(feature = "run-hitl")]
      let rtt_down_channel: Option<rtt_target::DownChannel>; 
      #[cfg(feature = "run-hitl")]
      let rtt_up_channel:   Option<rtt_target::UpChannel>;
      #[cfg(not(feature = "run-hitl"))]
      let rtt_down_channel: Option<rtt_target::DownChannel> = None; 
      #[cfg(not(feature = "run-hitl"))]
      let rtt_up_channel:   Option<rtt_target::UpChannel>   = None;

      #[cfg(feature = "run-hitl")]
      {
        //rtt_target::rtt_init_print!(); 
        //rtt_target::rtt_init_default!(); // New macro for bidirectional RTT
        // Default init macro for buffer size cfg
        let rtt_channels = rtt_target::rtt_init! {
          up: {
            0: {
              size: 1024, 
              name: "Terminal"
            }
            1: {
              size: 64,
              name: "HITL"
            }
          }
          down: {
            0: {
              size: 64, 
              mode: ChannelMode::NoBlockSkip,
              name: "Terminal"
            }
          }
        };
        rtt_target::set_print_channel(rtt_channels.up.0);
        rtt_target::rprintln!("RTT> Running in HITL mode");
        
        // Assign to tasks  
        rtt_down_channel = Some(rtt_channels.down.0);
        rtt_up_channel   = Some(rtt_channels.up.1);

      }
    
      let dp = con.device; 
      let cp = con.core;

      Mono::start(cp.SYST, 400_000_000);

      // Constrain and Freeze power
      let pwr = dp.PWR.constrain();
      let pwrcfg = pwr.freeze();

      // Constrain and Freeze clock
      let rcc = dp.RCC.constrain();
      //let ccdr = rcc.sys_ck(400.MHz()).freeze(pwrcfg, &dp.SYSCFG);
      let ccdr = rcc.sys_ck(400.MHz()).pll1_q_ck(48.MHz()).freeze(pwrcfg, &dp.SYSCFG);

      // Enable GPIOA/B clocks
      let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
      let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
      let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);
      let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);
      let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);

      // Pins for LEDs
      let mut led_r = gpioa.pa0.into_push_pull_output();
      let mut led_g = gpioa.pa1.into_push_pull_output();
      let mut led_b = gpioa.pa2.into_push_pull_output();

      // Set LEDs to off initially
      led_r.set_high();
      //led_g.set_low();
      //led_b.set_low();
      led_g.set_high();
      led_b.set_high();

      // Pins for USART3
      let tx1 = gpiob.pb10.into_alternate();
      let rx1 = gpiob.pb11.into_alternate();
      
      let tx2 = gpiod.pd1.into_alternate();
      let rx2 = gpiod.pd0.into_alternate();

      // Configure the serial peripheral.
      let mut serial1 = dp
          .USART3
          .serial((tx1, rx1), 115_200.bps(), ccdr.peripheral.USART3, &ccdr.clocks)
          .unwrap();

      let mut _serial2 = dp
          .UART4
          // pass pins uart to explicitly call uart instead of usart
          //.serial((tx2, rx2), 115_200.bps(), ccdr.peripheral.UART4, &ccdr.clocks)
          .serial((tx2, rx2), 38_400.bps(), ccdr.peripheral.UART4, &ccdr.clocks)
          .unwrap();

      serial1.listen(stm32h7xx_hal::serial::Event::Rxne);
      _serial2.listen(stm32h7xx_hal::serial::Event::Rxne);

      //writeln!(serial, "Eredin> Configuring SPI...\r").unwrap();
      //let sck = gpioa.pa5.into_alternate(); // SCK
      //let miso = gpioa.pa6.into_alternate(); // MISO
      //let mosi = gpioa.pa7.into_alternate(); // MOSI
      let sck:     stm32h7xx_hal::gpio::Pin<'A', 5,  stm32h7xx_hal::gpio::Alternate<5>> = gpioa.pa5.into_alternate(); // SCK
      let miso:    stm32h7xx_hal::gpio::Pin<'A', 6,  stm32h7xx_hal::gpio::Alternate<5>> = gpioa.pa6.into_alternate(); // MISO
      //let mosi:    stm32h7xx_hal::gpio::Pin<'A', 7,  stm32h7xx_hal::gpio::Alternate<5>> = gpioa.pa7.into_alternate(); // MOSI
      // Invert mosi logic, whould be 1 instead of 0 and vice versa
      let mosi:    stm32h7xx_hal::gpio::Pin<'A', 7,  stm32h7xx_hal::gpio::Alternate<5>> = gpioa.pa7.into_alternate().internal_pull_up(true); // MOSI with pull-up

      let mut othercs1 = gpiof.pf14.into_push_pull_output();
      let mut othercs2 = gpiof.pf15.into_push_pull_output();
      let mut othercs3 = gpiog.pg0.into_push_pull_output();
      let mut othercs4 = gpiog.pg1.into_push_pull_output();
      othercs1.set_high();
      othercs2.set_high();
      othercs3.set_high();
      othercs4.set_high();
      
      //let mut cs_baro = gpiof.pf13.into_push_pull_output(); // CS
      let mut cs_baro = othercs3; // Use other CS pin for baro 
      //cs_baro.set_high(); // Deassert CS
      cs_baro.set_low(); // Force SPI mode during sensor power-up
    
      let mut cs_gyro = othercs2; // gyro CS
      cs_gyro.set_low();

      // Small delay to ensure sensor sees CSB low during startup
      for _ in 0..2_500_000 { cortex_m::asm::nop(); }

      // Now you can set it high (idle)
      cs_baro.set_high();
      cs_gyro.set_high();

      //writeln!(serial, "Eredin> Configuring SPI2...\r").unwrap();


      let spi_if: stm32h7xx_hal::spi::Spi<stm32h7xx_hal::stm32::SPI1, stm32h7xx_hal::spi::Enabled, u8> = dp.SPI1.spi( 
        (
          sck,
          miso,
          mosi,
         // cs_baro,
        ),
        //spi::MODE_0,
        spi::Config::new(spi::MODE_0)
            // Put 1 us idle time between every word sent
            //.inter_word_delay(0.000001)
            // Specify that we use the hardware cs
            //.swap_mosi_miso()
        ,
        400.kHz(),
        ccdr.peripheral.SPI1,
        &ccdr.clocks,
      );

      // Greet before spinning
      //writeln!(serial, "Eredin> Starting Scheduler...\r").unwrap();

      // Schedule software tasks
      //task_baro::spawn().ok();
      //task_gyro::spawn().ok();
      basic_led::spawn().ok();

      // Odometry init
      let mut odometry = eredin_types::Odometry {
        pose:     [0.0; 7],
        velocity: [0.0; 6],
      };
      odometry.pose[0] = 1.0; // hamilton quaternion w=1 

      // Software task for rtt demo
      #[cfg(feature = "run-hitl")]
      { 
        rtt_target::rprintln!("RTT> Starting RTT task..."); // We dont have ownership anymore
        task_rtt_receive::spawn().ok();
      }

      // Resources for tasks
      (
        Shared {
          led_r,
          led_g,
          led_b,
          serial1,
          _serial2,
          spi: spi_if, 
          cs_baro,
          cs_gyro,
          odometry,
        },
        Local {
          rtt_down_channel,
          rtt_up_channel,
        },
      )
  }

  // External tasks
  extern "Rust" {
    #[task(shared = [led_r])]
    async fn basic_led(con: basic_led::Context);
    #[task(shared = [led_b, odometry], local = [rtt_down_channel, rtt_up_channel])]
    async fn task_rtt_receive(con: task_rtt_receive::Context);
  }

  //#[task(shared = [spi, serial1, cs_baro])]
  #[task(shared = [spi, serial1, cs_baro, led_r, led_g, led_b])]
  async fn task_baro(con: task_baro::Context) {
    let spi1 = con.shared.spi;
    let serial = con.shared.serial1;
    let cs_baro = con.shared.cs_baro;
    let _led_r = con.shared.led_r;
    let _led_g = con.shared.led_g;
    let _led_b = con.shared.led_b;
    let mut p_lock = (spi1, serial, cs_baro);
    //let mut q_lock = (spi1, serial, cs_baro, led_r, led_g, led_b);

    //<<<<<<<<<<< Abstract as begin function
    // Ask for chip id and print to rtt
    loop {
      let mut tx_buf_chipid: [u8; 3] = [0x00 | 0x80, 0x00, 0x00]; // Read register 0x0F, dummy byte
      p_lock.lock(|spi, serial, cs_baro| {
        cs_baro.set_low(); // Assert CS
        spi.transfer(&mut tx_buf_chipid).unwrap();
        cs_baro.set_high(); // Deassert CS
        writeln!(serial, "Baro> Chip ID read: {:02X?}\r", tx_buf_chipid).unwrap();
      });

      // If second element is 0x1E, break
      if tx_buf_chipid[2] == 0x1E {
          break;
      }

      Mono::delay(1000.millis()).await;
    }
    //<<<<<<<<<<<

    /*
    //<<<<<<<<<< Abstract as init function (maybe inside begin)
    // Turn on sensor
    let tx_buf_init: [u8; 2] = [0x7E, 0xB6]; // Buffer to write powerup to sensor
    p_lock.lock(|spi, serial, cs_baro| {
      cs_baro.set_low(); // Assert CS
      spi.write(&tx_buf_init).unwrap();
      cs_baro.set_high(); // Deassert CS
      writeln!(serial, "Baro> Initialization write done\r").unwrap();
    });

    Mono::delay(100.millis()).await;

    //>>>>>>>>>*/
    
    //<<<<<<<<<< Abstract as init function (maybe inside begin)
    // Turn on sensor
    let tx_buf_init: [u8; 2] = [0x7D, 0x04]; // Buffer to write powerup to sensor
    p_lock.lock(|spi, serial, cs_baro| {
      cs_baro.set_low(); // Assert CS
      spi.write(&tx_buf_init).unwrap();
      cs_baro.set_high(); // Deassert CS
      writeln!(serial, "Baro> Initialization write done\r").unwrap();
    });
    //>>>>>>>>>
    

    loop {
      p_lock.lock(|spi, serial, cs_baro| {
      //q_lock.lock(|spi, serial, cs_baro, _led_r, _led_g, _led_b| {
      
        // Reserved readings, for exploiting full-duplex SPI
        let mut tx_buf_data: [u8; 8] = [0x00; 8];
        tx_buf_data[0] = 0x12 | 0x80;
        cs_baro.set_low(); // Assert CS  
        spi.transfer(&mut tx_buf_data).unwrap();
        cs_baro.set_high(); // Deassert CS
        //writeln!(serial, "Baro> Data regs read: {:02X?}\r", tx_buf_data).unwrap();
        
        //writeln!(serial, "Baro> Data regs read: {:02X?}\r", &tx_buf_data[2..8]).unwrap();

        // Post process data
        let raw_accelx: i16 = i16::from_be_bytes([tx_buf_data[3], tx_buf_data[2]]); // MSB, LSB
        let raw_accely: i16 = i16::from_be_bytes([tx_buf_data[5], tx_buf_data[4]]); // MSB, LSB
        let raw_accelz: i16 = i16::from_be_bytes([tx_buf_data[7], tx_buf_data[6]]); // MSB, LSB
                                                                                    
        const SCALE_FACTOR: f32 = 0.183105 * 0.01; // mg/LSB

        let accel_x_g: f32 = (raw_accelx as f32) * SCALE_FACTOR;
        let accel_y_g: f32 = (raw_accely as f32) * SCALE_FACTOR;
        let accel_z_g: f32 = (raw_accelz as f32) * SCALE_FACTOR;

        writeln!(serial, "Accel X: {:.3}, Y: {:.3}, Z: {:.3}\r", accel_x_g, accel_y_g, accel_z_g).unwrap();

      });

      Mono::delay(500.millis()).await;
    }
  }

  #[task(shared = [spi, serial1, cs_gyro, led_r, led_g, led_b])]
  async fn task_gyro(con: task_gyro::Context) {
    let spi1 = con.shared.spi;
    let serial = con.shared.serial1;
    let cs_gyro = con.shared.cs_gyro;
    let _led_r = con.shared.led_r;
    let _led_g = con.shared.led_g;
    let _led_b = con.shared.led_b;
    let mut p_lock = (spi1, serial, cs_gyro);

    //<<<<<<<< Abstract as begin function >>>>>>>>
    // Ask for chip id and print to rtt
    loop {
      let mut tx_buf_chipid: [u8; 3] = [0x00 | 0x80, 0x00, 0x00]; // Read register 0x00, dummy byte
      p_lock.lock(|spi, serial, cs_gyro| {
        cs_gyro.set_low(); // Assert CS
        spi.transfer(&mut tx_buf_chipid).unwrap();
        cs_gyro.set_high(); // Deassert CS
        writeln!(serial, "Gyro> Chip ID read: {:02X?}\r", tx_buf_chipid).unwrap();
      });

      // If second element is 0x0F, break
      if tx_buf_chipid[1] == 0x0F {
          break;
      }

      Mono::delay(1000.millis()).await;
    }

    // Gyro está always-on por hardware
    // SETTING RANGE
    let tx_buf_range: [u8; 2] = [0x0F, 0x00]; // Buffer to write powerup to sensor
    p_lock.lock(|spi, serial, cs_gyro| {
      cs_gyro.set_low(); // Assert CS
      spi.write(&tx_buf_range).unwrap();
      cs_gyro.set_high(); // Deassert CS
      writeln!(serial, "Gyro> Range set write done\r").unwrap();
    });
    Mono::delay(10.millis()).await;

    // READING RANGE
    let mut tx_buf_read_range: [u8; 3] = [0x0F | 0x80, 0x00, 0x00]; // Buffer para lectura

    let gyro_range = p_lock.lock(|spi, serial, cs| {
        cs.set_low();
        spi.transfer(&mut tx_buf_read_range).unwrap(); // TX y RX simultáneo
        cs.set_high();

        let range = tx_buf_read_range[1]; // o tx_buf_read_range[1] según el sensor
        writeln!(serial, "Gyro> Range = 0x{:02X}\r", range).unwrap();
        range
    });
    Mono::delay(10.millis()).await;

    // SETTING BANDWIDTH
    let tx_buf_bw: [u8; 2] = [0x10, 0x80]; // Buffer to reset Bandwidth
    p_lock.lock(|spi, serial, cs_gyro| {
      cs_gyro.set_low(); // Assert CS
      spi.write(&tx_buf_bw).unwrap();
      cs_gyro.set_high(); // Deassert CS
      writeln!(serial, "Gyro> Bandwidth set ODR\r").unwrap();
    });
    Mono::delay(10.millis()).await;
    
    // Definir Scale Factor ( mº/s per LSB )
    let GYRO_SCALE_FACTOR: f32 = match gyro_range {
      0x00 => 61.0 / 1000.0,      // ±2000°/s → 61.0 m°/s per LSB
      0x01 => 30.5 / 1000.0,      // ±1000°/s → 30.5 m°/s per LSB
      0x02 => 15.3 / 1000.0,      // ±500°/s  → 15.3 m°/s per LSB
      0x03 => 7.6 / 1000.0,       // ±250°/s  → 7.6 m°/s per LSB
      0x04 => 3.8 / 1000.0,       // ±125°/s  → 3.8 m°/s per LSB
      _ => 61.0 / 1000.0,         // Default: ±2000°/s
    };

    loop {
      p_lock.lock(|spi, serial, cs_gyro| {
          let mut tx_buf_data: [u8; 8] = [0x00; 8];
          tx_buf_data[0] = 0x02 | 0x80; // 0x02 corresponde a RATE_X_LSB
          
          cs_gyro.set_low();
          spi.transfer(&mut tx_buf_data).unwrap();
          cs_gyro.set_high();

          // IMPRIMIR TODO EL BUFFER
          // writeln!(serial, "Full buffer: {:02X?}\r", tx_buf_data).unwrap();

          // Post process data with Little-Endian ( esto equivale a la fórmula indicadad en datasheet: Rate_X: RATE_X_MSB * 256 + RATE_X_LSB )
          let raw_gyro_y: i16 = i16::from_le_bytes([tx_buf_data[1], tx_buf_data[2]]); // LSB, MSB  (Dummy area)
          let raw_gyro_x: i16 = i16::from_le_bytes([tx_buf_data[3], tx_buf_data[4]]); // LSB, MSB
          let raw_gyro_z: i16 = i16::from_le_bytes([tx_buf_data[5], tx_buf_data[6]]); // LSB, MSB
          
          // Conversión a grados por segundo ( º/s )
          let gyro_x: f32 = (raw_gyro_x as f32) * GYRO_SCALE_FACTOR;
          let gyro_y: f32 = (raw_gyro_y as f32) * GYRO_SCALE_FACTOR;
          let gyro_z: f32 = (raw_gyro_z as f32) * GYRO_SCALE_FACTOR;

          writeln!(serial, "Gyro X:{:.2} Y:{:.2} Z:{:.2}\r", 
                  gyro_x, gyro_y, gyro_z).unwrap();
      });

      Mono::delay(500.millis()).await;
    }
  }
}
