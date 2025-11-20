#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;

use defmt::info;
use esp_println::{self as _, println};

use embassy_executor::Spawner;
use embassy_time::Delay;

use esp_backtrace as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let mut mpu = switchgrass_cattail::mpu::init(
        peripherals.I2C0,
        peripherals.GPIO37,
        peripherals.GPIO36,
        &mut Delay,
    )
    .await;

    switchgrass_cattail::mpu::save_calibrate(&mut mpu, peripherals.FLASH, &mut Delay).await;

    info!("Successfully calibrated MPU6050! Looping forever.");

    loop {
        let (acc, gyro) = mpu.motion6().await.unwrap();
        let acc = acc.scaled(switchgrass_cattail::mpu::CALIBRATION_PARAMETERS.accel_scale);
        let gyro = gyro.scaled(switchgrass_cattail::mpu::CALIBRATION_PARAMETERS.gyro_scale);
        println!(
            "acc: {:.2} {:.2} {:.2} gyro: {:.2} {:.2} {:.2}",
            acc.x(),
            acc.y(),
            acc.z(),
            gyro.x(),
            gyro.y(),
            gyro.z(),
        );
    }
}
