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
use embassy_time::{Delay, Duration, Timer};

use esp_backtrace as _;
use mpu6050_dmp::{
    accel::AccelFullScale,
    address::Address,
    calibration::{CalibrationParameters, ReferenceGravity},
    gyro::GyroFullScale,
    sensor_async::Mpu6050,
};

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

    let mpu_calibration = CalibrationParameters::new(
        AccelFullScale::G4,
        GyroFullScale::Deg250,
        ReferenceGravity::Zero,
    );
    mpu.calibrate(&mut Delay, &mpu_calibration).await.unwrap();

    loop {
        let (acc, gyro) = mpu.motion6().await.unwrap();
        let acc = acc.scaled(mpu_calibration.accel_scale);
        let gyro = gyro.scaled(mpu_calibration.gyro_scale);

        let msg = data_message("accel", [acc.x(), acc.y(), acc.z()]);
        info!("sending {}", msg.as_str());
        out.write_async(msg.as_bytes()).await.unwrap();

        let msg = data_message("gyro", [gyro.x(), gyro.y(), gyro.z()]);
        info!("sending {}", msg.as_str());
        out.write_async(msg.as_bytes()).await.unwrap();

        Timer::after(Duration::from_millis(100)).await;
    }
}

const ID: &str = env!("ID");

fn data_message(r#type: &str, [x, y, z]: [f32; 3]) -> heapless::String<256> {
    assert!(r#type.len() <= 16);
    let mut msg = heapless::String::new();
    writeln!(&mut msg, "id={ID} type={type} x={x} y={y} z={z}",).unwrap();
    msg
}
