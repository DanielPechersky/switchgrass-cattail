#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::{fmt::Write as _, sync::atomic::Ordering};

use atomic_float::AtomicF32;
use esp_hal::{Async, clock::CpuClock, i2c::master::I2c};
use esp_hal::{timer::timg::TimerGroup, uart::UartTx};

use defmt::{info, warn};
use esp_println as _;

use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};

use esp_backtrace as _;
use mpu6050_dmp::{config::DigitalLowPassFilter, sensor_async::Mpu6050};

use switchgrass_cattail::{mpu::NotStored, particles::Particles, ws281x};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const STRIP_LENGTH: usize = 200;
const WS281X_BYTES: usize = STRIP_LENGTH * 12;

type Ws281x = switchgrass_cattail::ws281x::Ws281x<'static, WS281X_BYTES>;

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let ws281x: Ws281x =
        ws281x::init::<WS281X_BYTES>(peripherals.SPI2, peripherals.GPIO10, peripherals.DMA_CH0);
    let particles = Particles::new(11.0);
    static PARTICLE_DISPLACEMENT: AtomicF32 = AtomicF32::new(0.0);
    spawner.must_spawn(particle_task(ws281x, particles, &PARTICLE_DISPLACEMENT));

    let mut mpu = switchgrass_cattail::mpu::init(
        peripherals.I2C0,
        peripherals.GPIO37,
        peripherals.GPIO36,
        &mut Delay,
    )
    .await;

    if let Err(NotStored) =
        switchgrass_cattail::mpu::load_calibrate(&mut mpu, peripherals.FLASH).await
    {
        warn!("MPU6050 was never calibrated!")
    }

    mpu.set_sample_rate_divider(99).await.unwrap();
    mpu.set_digital_lowpass_filter(DigitalLowPassFilter::Filter3)
        .await
        .unwrap();

    let out = switchgrass_cattail::transmission::init(peripherals.UART1, peripherals.GPIO9);

    spawner.must_spawn(handle_mpu6050(mpu, out, &PARTICLE_DISPLACEMENT));
}

#[embassy_executor::task]
async fn handle_mpu6050(
    mut mpu: Mpu6050<I2c<'static, Async>>,
    mut out: UartTx<'static, Async>,
    particle_displacement: &'static AtomicF32,
) {
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

        let uprightness = ((-acc.x() - 0.5) * 2.0).clamp(0.0, 1.0);
        let displacement_when_upright = -5.0;
        let displacement_when_down = 40.0;
        let displacement =
            uprightness * displacement_when_upright + (1.0 - uprightness) * displacement_when_down;
        particle_displacement.store(displacement, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
async fn particle_task(
    mut ws281x: Ws281x,
    mut particles: Particles,
    displacement: &'static AtomicF32,
) {
    loop {
        let d = displacement.swap(0.0, Ordering::Relaxed) / 60.0;
        particles.displace_by(d);
        switchgrass_cattail::ws281x::write_particles(&mut ws281x, &particles, STRIP_LENGTH).await;
        Timer::after_millis(1000 / 60).await
    }
}

const ID: &str = env!("ID");

fn data_message(r#type: &str, [x, y, z]: [f32; 3]) -> heapless::String<256> {
    assert!(r#type.len() <= 16);
    let mut msg = heapless::String::new();
    writeln!(&mut msg, "id={ID} type={type} x={x} y={y} z={z}",).unwrap();
    msg
}
