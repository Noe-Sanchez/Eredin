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
  //gpio::PF13,
  gpio::PG0,
  gpio::Output,
  gpio::PushPull,
  stm32::USART3,
  stm32::UART4,
  spi,
  spi::SpiExt,
  //block,
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
      serial1:   stm32h7xx_hal::serial::Serial<USART3>,
      serial2:   stm32h7xx_hal::serial::Serial<UART4>,
      dt:       u32, 
      odometry: eredin_types::Odometry, 
      spi:      stm32h7xx_hal::spi::Spi<stm32h7xx_hal::stm32::SPI1, stm32h7xx_hal::spi::Enabled, u8>,
      // alternate 
      //cs_baro:  PF13<Output<PushPull>>, // Chip select for barometer
      cs_baro:  PG0<Output<PushPull>>, // Chip select for barometer on alternate pin
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

      let mut serial2 = dp
          .UART4
          // pass pins uart to explicitly call uart instead of usart
          //.serial((tx2, rx2), 115_200.bps(), ccdr.peripheral.UART4, &ccdr.clocks)
          .serial((tx2, rx2), 38_400.bps(), ccdr.peripheral.UART4, &ccdr.clocks)
          .unwrap();

      serial1.listen(stm32h7xx_hal::serial::Event::Rxne);
      serial2.listen(stm32h7xx_hal::serial::Event::Rxne);

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

      // Small delay to ensure sensor sees CSB low during startup
      for _ in 0..2_500_000 { cortex_m::asm::nop(); }

      // Now you can set it high (idle)
      cs_baro.set_high();

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


      let dt = 0; // Initialize dt

      // Greet before spinning
      //writeln!(serial, "Eredin> Starting Scheduler...\r").unwrap();

      // Schedule software tasks
      //task_telemetry::spawn().ok();
      //task_compute_control::spawn().ok(); // Actual task lol
      task_baro::spawn().ok();
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
        //task_rtt_receive::spawn().ok();
      }

      // Resources for tasks
      (
        Shared {
          led_r,
          led_g,
          led_b,
          serial1,
          serial2,
          dt,
          odometry,
          spi: spi_if, 
          cs_baro,
        },
        Local {
          read_data,
          idx,
          rtt_channel,
        },
      )
  }

  //#[task(shared = [spi, serial1, cs_baro])]
  #[task(shared = [spi, serial1, cs_baro, led_r, led_g, led_b])]
  async fn task_baro(con: task_baro::Context) {
    let spi1 = con.shared.spi;
    let serial = con.shared.serial1;
    let cs_baro = con.shared.cs_baro;
    let led_r = con.shared.led_r;
    let led_g = con.shared.led_g;
    let led_b = con.shared.led_b;
    let mut p_lock = (spi1, serial, cs_baro);
    let mut q_lock = (spi1, serial, cs_baro, led_r, led_g, led_b);

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
      //p_lock.lock(|spi, serial, cs_baro| {
      q_lock.lock(|spi, serial, cs_baro, led_r, led_g, led_b| {
      
        // Reserved readings, for exploiting full-duplex SPI
        let mut tx_buf_data: [u8; 8] = [0x00; 8];
        tx_buf_data[0] = 0x12 | 0x80;
        cs_baro.set_low(); // Assert CS  
        spi.transfer(&mut tx_buf_data).unwrap();
        cs_baro.set_high(); // Deassert CS
        //writeln!(serial, "Baro> Data regs read: {:02X?}\r", tx_buf_data).unwrap();
        writeln!(serial, "Baro> Data regs read: {:02X?}\r", &tx_buf_data[2..8]).unwrap();

        // Post process data
        let raw_accelx: i16 = i16::from_be_bytes([tx_buf_data[3], tx_buf_data[2]]); // MSB, LSB
        let raw_accely: i16 = i16::from_be_bytes([tx_buf_data[5], tx_buf_data[4]]); // MSB, LSB
        let raw_accelz: i16 = i16::from_be_bytes([tx_buf_data[7], tx_buf_data[6]]); // MSB, LSB
                                                                                    
        const SCALE_FACTOR: f32 = 0.183105 * 0.01; // mg/LSB

        let accel_x_g: f32 = (raw_accelx as f32) * SCALE_FACTOR;
        let accel_y_g: f32 = (raw_accely as f32) * SCALE_FACTOR;
        let accel_z_g: f32 = (raw_accelz as f32) * SCALE_FACTOR;

        writeln!(serial, "Baro> Accels [g]: X: {:.3}, Y: {:.3}, Z: {:.3}\r", accel_x_g, accel_y_g, accel_z_g).unwrap();

      });

      Mono::delay(500.millis()).await;
    }
  }

  #[task(binds = UART4, shared = [serial2, serial1, led_r], local = [read_data, idx])]
  fn task_receive(con: task_receive::Context) {
    let _serial1_if = con.shared.serial1; 
    let _serial2_if = con.shared.serial2;
    let _led_r      = con.shared.led_r;
    
    //let byte = serial2_if.lock(|serial2_if| {
    //  serial2_if.read()
    //});

    //led_r.lock(|led| {
    //    led.toggle();
    //});

    /*serial1_if.lock(|serial1_if| {
      serial1_if.write_str("Interrupt> UART4 RX interrupt\r").ok();
    });*/

    //let mut ifs = (serial1_if, serial2_if, led_r);
    

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

  //#[task(shared = [serial1, led_g, led_b, odometry])]
  #[task(shared = [led_g, led_b])]
  async fn task_telemetry(con: task_telemetry::Context) {
    let ledg = con.shared.led_g;
    let ledb = con.shared.led_b;
    let mut qk = (ledg, ledb);

    loop {
      qk.lock(|ledg, ledb| {
          ledg.set_low();
          ledb.set_low();
      });

      Mono::delay(100.millis()).await;

      qk.lock(|ledg, ledb| {
          ledg.set_high();
          ledb.set_high();
      });

      Mono::delay(100.millis()).await;

      qk.lock(|ledg, ledb| {
          ledg.set_low();
          ledb.set_low();
      });

      Mono::delay(100.millis()).await;

      qk.lock(|ledg, ledb| {
          ledg.set_high();
          ledb.set_high();
      });

      Mono::delay(1_900.millis()).await;

    }
  }
}
