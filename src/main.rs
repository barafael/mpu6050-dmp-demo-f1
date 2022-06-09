#![deny(unsafe_code)]
#![no_std]
#![no_main]

use mpu6050_dmp::{quaternion::Quaternion, yaw_pitch_roll::YawPitchRoll};
use panic_rtt_target as _;

use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::{
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac,
    prelude::*,
    timer::Timer,
};

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    rtt_init_print!();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Acquire the GPIOC peripheral
    let mut gpioc = dp.GPIOC.split();

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // Configure I2C pins
    let mut gpiob = dp.GPIOB.split();
    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

    // Setup i2c
    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 400_000.Hz(),
            duty_cycle: DutyCycle::Ratio16to9,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    let mut delay = cp.SYST.delay(&clocks);

    let mut sensor =
        mpu6050_dmp::sensor::Mpu6050::new(i2c, mpu6050_dmp::address::Address::default()).unwrap();

    sensor.initialize_dmp(&mut delay).unwrap();

    let syst = delay.release().release();

    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(syst, &clocks).counter_hz();
    timer.start(1.Hz()).unwrap();

    loop {
        let len = sensor.get_fifo_count().unwrap();
        if len >= 28 {
            let mut buf = [0; 28];
            let buf = sensor.read_fifo(&mut buf).unwrap();
            let quat = Quaternion::from_bytes(&buf[..16]).unwrap();
            let ypr = YawPitchRoll::from(quat);
            rprintln!("{:.5?}; {:.5?}; {:.5?};", ypr.yaw, ypr.pitch, ypr.roll);
            led.toggle();
        }
    }
}
