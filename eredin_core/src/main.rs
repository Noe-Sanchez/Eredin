// #![deny(warnings)]
#![no_main]
#![no_std]

#[cfg(feature = "run-hitl")]
mod hitl_imports {
  use panic_rtt_target as _;
}
#[cfg(not(feature = "run-hitl"))]
use panic_halt as _;

pub mod eredin_types{
  pub struct Odometry {
    pub pose:     [f32; 7], // x, y, z, qw, qx, qy, qz
    pub velocity: [f32; 6], // vx, vy, vz, wx, wy, wz
  }
  pub struct SharkData {
    pub baro:     [f32; 2], // pressure, temperature
  }
}

// BMP390 constants module
mod bmp399 {
  pub const CHIP_ID: u8 = 0x00;
  pub const STATUS: u8 = 0x03;
  pub const DATA_0: u8 = 0x04;
  pub const PWR_CTRL: u8 = 0x1B;
  pub const OSR: u8 = 0x1C;
  pub const ODR: u8 = 0x1D;
  pub const CONFIG: u8 = 0x1F;
  pub const CMD: u8 = 0x7E;
  pub const EXPECTED_CHIP_ID: u8 = 0x60;
  pub const SPI_READ: u8 = 0x80;
  pub const SPI_WRITE: u8 = 0x00;
}

use rtic::app;
use rtic_monotonics::systick::prelude::*;

use core::fmt::Write;

use stm32h7xx_hal::{
    prelude::*,
    gpio::{PA0, PA1, PA2, PA5, PA6, PA7, PF13, Output, PushPull},
    stm32::{USART3, SPI1},
    spi,
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
      serial:   stm32h7xx_hal::serial::Serial<USART3>,
      dt:       u32, 
      odometry: eredin_types::Odometry,
      sharkdata: eredin_types::SharkData,
      spi: spi::Spi<SPI1, spi::Enabled>,
    }

    #[local]
    struct Local {
      rtt_channel: Option<rtt_target::DownChannel>,
      bmp390_cs: PF13<Output<PushPull>>,
    }

    #[init]
    fn init(con: init::Context) -> (Shared, Local){
      
      #[cfg(feature = "run-hitl")]
      let rtt_channel: Option<rtt_target::DownChannel>; 
      #[cfg(not(feature = "run-hitl"))]
      let rtt_channel: Option<rtt_target::DownChannel> = None; 

      #[cfg(feature = "run-hitl")]
      {
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
      let ccdr = rcc.sys_ck(400.MHz()).freeze(pwrcfg, &dp.SYSCFG);

      // Enable GPIOA/B/F clocks
      let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
      let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
      let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);

      // Pins for LEDs
      let mut led_r = gpioa.pa0.into_push_pull_output();
      let mut led_g = gpioa.pa1.into_push_pull_output();
      let mut led_b = gpioa.pa2.into_push_pull_output();

      // Set LEDs to off initially
      led_r.set_high();
      led_g.set_high();
      led_b.set_high();

      // SPI1 pins for BMP390
      let sck = gpioa.pa5.into_alternate();
      let miso = gpioa.pa6.into_alternate();
      let mosi = gpioa.pa7.into_alternate();
      let mut bmp390_cs = gpiof.pf13.into_push_pull_output();
      bmp390_cs.set_high(); // CS inactive

      // Pins for USART3
      let tx = gpiob.pb10.into_alternate();
      let rx = gpiob.pb11.into_alternate();

      // Configure the serial peripheral.
      let mut serial = dp
          .USART3
          .serial((tx, rx), 115_200.bps(), ccdr.peripheral.USART3, &ccdr.clocks)
          .unwrap();

      serial.listen(stm32h7xx_hal::serial::Event::Rxne);

      writeln!(serial, "Eredin> Starting TREMEENDO Scheduler...\r").unwrap(); // debug line
      
      // Configure SPI1
      // let mut spi: spi::Spi<_, _, u8> = dp.SPI1.spi(
      let mut spi = dp.SPI1.spi(
        (sck, miso, mosi),
        spi::MODE_0,
        1.MHz(), // Prueba con velocidad más baja primero
        ccdr.peripheral.SPI1,
        &ccdr.clocks
      );

      
      let dt = 0; // Initialize dt

      // Initialize BMP390
      writeln!(serial, "Eredin> Initializing BMP390...\r").unwrap();
      
      // Small delay for sensor boot
      for _ in 0..400_000 { cortex_m::asm::nop(); }
      
      // Read chip ID
      let chip_id = bmp390_read_register(&mut bmp390_cs, &mut spi, bmp399::CHIP_ID);
      
      if chip_id == bmp399::EXPECTED_CHIP_ID {
          writeln!(serial, "Eredin> BMP390 detected! (ID: 0x{:02X})\r", chip_id).unwrap();
          
          // Soft reset
          bmp390_write_register(&mut bmp390_cs, &mut spi, bmp399::CMD, 0xB6);
          for _ in 0..4_000_000 { cortex_m::asm::nop(); } // 10ms delay
          
          // Configure sensor
          bmp390_write_register(&mut bmp390_cs, &mut spi, bmp399::PWR_CTRL, 0x33);
          bmp390_write_register(&mut bmp390_cs, &mut spi, bmp399::OSR, 0x03);
          bmp390_write_register(&mut bmp390_cs, &mut spi, bmp399::ODR, 0x04);
          bmp390_write_register(&mut bmp390_cs, &mut spi, bmp399::CONFIG, 0x02);
          
          writeln!(serial, "Eredin> BMP390 configured!\r").unwrap();
      } else {
          writeln!(serial, "Eredin> BMP390 ERROR: Wrong chip ID 0x{:02X}\r", chip_id).unwrap();
      }

      // Greet before spinning
      writeln!(serial, "Eredin> Starting Scheduler...\r").unwrap();

      // Schedule software tasks
      task_telemetry::spawn().ok();
      task_baro::spawn().ok();
      task_compute_control::spawn().ok();

      let odometry: eredin_types::Odometry = eredin_types::Odometry {
        pose: [0.0; 7], 
        velocity: [0.0; 6], 
      };
      let sharkdata: eredin_types::SharkData = eredin_types::SharkData {
        baro: [0.0; 2],
      };

      // Software task for rtt demo
      #[cfg(feature = "run-hitl")]
      {
        rtt_target::rprintln!("RTT> Starting RTT task...");
        task_rtt_receive::spawn().ok();
      }

      // Resources for tasks
      (
        Shared {
          led_r,
          led_g,
          led_b,
          serial,
          dt,
          odometry,
          sharkdata,
          spi,
        },
        Local {
          rtt_channel,
          bmp390_cs,
        },
      )
  }
    
  #[task(shared = [led_b, dt, odometry], local = [rtt_channel])]
  async fn task_rtt_receive(con: task_rtt_receive::Context) {
    let mut count = 0;
    let mut led = con.shared.led_b;
    let dt = con.shared.dt;
    let odometry = con.shared.odometry;
    let chan_opt = con.local.rtt_channel;
    let channel = chan_opt.as_mut().expect("RTT channel not initialized");

    let mut bq_t = (dt, odometry);
    let mut rtt_buffer: [u8; 64] = [0; 64]; 

    loop {
      led.lock(|led| {
          led.toggle();
      });
      bq_t.lock(|dt, _odometry| {
        let read_bytes = channel.read(&mut rtt_buffer);
        rtt_target::rprintln!("RTT> Read {} bytes: ", read_bytes);
        *dt += 1;
        rtt_target::rprintln!("RTT> Count: {}", count);
      });
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
    let mut bq_t = (dt, odometry);
    loop {
      led.lock(|led| {
          led.toggle();
      });

      bq_t.lock(|dt, odometry| {
        outputs[0] = odometry.pose[0] + (*dt as f32) * 0.001;
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

  #[task(shared = [serial, led_r, odometry])]
  async fn task_telemetry(con: task_telemetry::Context) {
    let serial_if = con.shared.serial; 
    let mut led = con.shared.led_r;
    let odometry = con.shared.odometry;
    let mut bq_t = (serial_if, odometry);
    loop {
      led.lock(|led| {
          led.toggle();
      });
      bq_t.lock(|serial, odometry| {
          writeln!(serial, "Telemetry> Odometry: pose: {:?}, velocity: {:?}\r", 
                   odometry.pose, odometry.velocity).unwrap();
      });

      Mono::delay(1000.millis()).await;
    }
  }

  #[task(shared = [serial, sharkdata, spi], local = [bmp390_cs])]
  async fn task_baro(con: task_baro::Context) {
      let serial_if = con.shared.serial; 
      let sharkdata = con.shared.sharkdata;
      let spi = con.shared.spi;
      let cs = con.local.bmp390_cs;
      
      let mut bq_t = (serial_if, sharkdata, spi);
      
      loop {
          bq_t.lock(|serial, sharkdata, spi| {
              // Check if data ready
              let status = bmp390_read_register(cs, spi, bmp399::STATUS);
              
              if (status & 0x60) == 0x60 {
                  // Read 6 bytes: 3 pressure, 3 temperature
                  let mut data = [0u8; 6];
                  bmp390_read_multiple(cs, spi, bmp399::DATA_0, &mut data);
                  
                  // Parse raw 24-bit values
                  let raw_pressure = ((data[2] as u32) << 16) | 
                                    ((data[1] as u32) << 8) | 
                                    (data[0] as u32);
                  
                  let raw_temp = ((data[5] as u32) << 16) | 
                                ((data[4] as u32) << 8) | 
                                (data[3] as u32);
                  
                  // Simple conversion (needs calibration for accuracy)
                  sharkdata.baro[0] = raw_pressure as f32 / 100.0;
                  sharkdata.baro[1] = raw_temp as f32 / 100.0;
                  
                  writeln!(serial, "Baro> P: {:.2} Pa, T: {:.2} C (raw)\r", 
                          sharkdata.baro[0], sharkdata.baro[1]).unwrap();
              }
          });

          Mono::delay(100.millis()).await;
      }
  }
}

// Helper functions outside the app module
fn bmp390_read_register<SPI, CS>(
    cs: &mut CS,
    spi: &mut SPI,
    reg: u8,
) -> u8
where
    SPI: embedded_hal::blocking::spi::Transfer<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
{
    let mut buf = [reg | bmp399::SPI_READ, 0x00];
    cs.set_low().ok();
    spi.transfer(&mut buf).ok();
    cs.set_high().ok();
    buf[1]
}

fn bmp390_write_register<SPI, CS>(
    cs: &mut CS,
    spi: &mut SPI,
    reg: u8,
    value: u8,
)
where
    SPI: embedded_hal::blocking::spi::Transfer<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
{
    let mut buf = [reg | bmp399::SPI_WRITE, value];
    cs.set_low().ok();
    spi.transfer(&mut buf).ok();
    cs.set_high().ok();
}

fn bmp390_read_multiple<SPI, CS>(
    cs: &mut CS,
    spi: &mut SPI,
    reg: u8,
    data: &mut [u8],
)
where
    SPI: embedded_hal::blocking::spi::Transfer<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
{
    cs.set_low().ok();
    let mut addr = [reg | bmp399::SPI_READ];
    spi.transfer(&mut addr).ok();
    spi.transfer(data).ok();
    cs.set_high().ok();
}