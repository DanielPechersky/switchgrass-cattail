#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

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

    info!("Initializing MPU6050 DMP");
    mpu.initialize_dmp(&mut Delay).await.unwrap();

    info!("Calibrating MPU6050");
    let mpu_calibration = CalibrationParameters::new(
        AccelFullScale::G2,
        GyroFullScale::Deg1000,
        ReferenceGravity::XN,
    );
    mpu.calibrate(&mut Delay, &mpu_calibration).await.unwrap();

    mpu.set_sample_rate_divider(99).await.unwrap();
    mpu.set_digital_lowpass_filter(DigitalLowPassFilter::Filter3)
        .await
        .unwrap();

    info!("Entering main loop");

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
    }
}
