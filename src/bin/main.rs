#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::fmt::Write as _;

use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;

use defmt::{info, warn};
use esp_println as _;

use embassy_executor::Spawner;
use embassy_time::Delay;

use esp_backtrace as _;
use mpu6050_dmp::config::DigitalLowPassFilter;

use smart_leds::RGB8;
use smart_leds::SmartLedsWriteAsync;

use switchgrass_cattail::{mpu::NotStored, ws281x};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    const STRIP_LENGTH: usize = 200;
    let led_data = [RGB8::new(90, 20, 0); STRIP_LENGTH];
    let mut ws281x = ws281x::init::<{ STRIP_LENGTH * 12 }>(peripherals.SPI2, peripherals.GPIO10);
    ws281x.write(led_data).await.unwrap();

    let mut mpu = switchgrass_cattail::mpu::init(
        peripherals.I2C0,
        peripherals.GPIO37,
        peripherals.GPIO36,
        &mut Delay,
    )
    .await;

    let mut out = switchgrass_cattail::transmission::init(peripherals.UART1, peripherals.GPIO9);

    if let Err(NotStored) =
        switchgrass_cattail::mpu::load_calibrate(&mut mpu, peripherals.FLASH).await
    {
        warn!("MPU6050 was never calibrated!")
    }

    mpu.set_sample_rate_divider(99).await.unwrap();
    mpu.set_digital_lowpass_filter(DigitalLowPassFilter::Filter3)
        .await
        .unwrap();

    info!("Entering main loop");

    loop {
        let (acc, gyro) = mpu.motion6().await.unwrap();
        let acc = acc.scaled(switchgrass_cattail::mpu::CALIBRATION_PARAMETERS.accel_scale);
        let gyro = gyro.scaled(switchgrass_cattail::mpu::CALIBRATION_PARAMETERS.gyro_scale);

        let msg = data_message("accel", [acc.x(), acc.y(), acc.z()]);
        info!("sending {}", msg.as_str());
        out.write_async(msg.as_bytes()).await.unwrap();

        let msg = data_message("gyro", [gyro.x(), gyro.y(), gyro.z()]);
        info!("sending {}", msg.as_str());
        out.write_async(msg.as_bytes()).await.unwrap();
    }
}

const ID: &str = env!("ID");

fn data_message(r#type: &str, [x, y, z]: [f32; 3]) -> heapless::String<256> {
    assert!(r#type.len() <= 16);
    let mut msg = heapless::String::new();
    writeln!(&mut msg, "id={ID} type={type} x={x} y={y} z={z}",).unwrap();
    msg
}
