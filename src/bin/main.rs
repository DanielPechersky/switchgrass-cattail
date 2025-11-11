#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::fmt::Write as _;

use esp_hal::timer::timg::TimerGroup;
use esp_hal::{clock::CpuClock, time::Rate};
use esp_hal::{
    i2c::master::{Config, I2c},
    uart::{self, UartTx},
};

use defmt::info;
use esp_println as _;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

use esp_backtrace as _;
use mpu6050_dmp::{address::Address, sensor_async::Mpu6050};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Initializing MPU6050");

    let i2c = I2c::new(
        peripherals.I2C0,
        Config::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO37)
    .with_scl(peripherals.GPIO36)
    .into_async();

    let mut mpu = Mpu6050::new(i2c, Address::default()).await.unwrap();
    let mut out = UartTx::new(peripherals.UART1, uart::Config::default())
        .unwrap()
        .with_tx(peripherals.GPIO9)
        .into_async();

    info!("MPU6050 initialized!");

    loop {
        let acc = mpu.accel().await.unwrap();
        let gyro = mpu.gyro().await.unwrap();

        info!("acc: x={}, y={}, z={}", acc.x(), acc.y(), acc.z());
        info!("gyro: x={}, y={}, z={}", gyro.x(), gyro.y(), gyro.z());

        let mut msg = heapless::String::<128>::new();
        writeln!(&mut msg, "x={} y={} z={}", acc.x(), acc.y(), acc.z()).unwrap();
        out.write_async(msg.as_bytes()).await.unwrap();

        Timer::after(Duration::from_millis(100)).await;
    }
}
