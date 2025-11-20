use defmt::info;
use embassy_time::Delay;
use embedded_storage::{ReadStorage as _, Storage as _};
use esp_hal::{
    Async,
    i2c::master::{Config, I2c},
    peripherals,
    time::Rate,
};
use esp_storage::FlashStorage;
use mpu6050_dmp::{
    accel::{Accel, AccelFullScale},
    address::Address,
    calibration::{CalibrationParameters, ReferenceGravity},
    gyro::{Gyro, GyroFullScale},
    sensor_async::Mpu6050,
};

pub async fn init<'a>(
    i2c: peripherals::I2C0<'a>,
    sda: peripherals::GPIO37<'a>,
    scl: peripherals::GPIO36<'a>,
    delay: &mut Delay,
) -> Mpu6050<I2c<'a, Async>> {
    info!("Initializing MPU6050");

    let i2c = I2c::new(i2c, Config::default().with_frequency(Rate::from_khz(100)))
        .unwrap()
        .with_sda(sda)
        .with_scl(scl)
        .into_async();

    let mut mpu = Mpu6050::new(i2c, Address::default()).await.unwrap();

    info!("Initializing MPU6050 DMP");
    mpu.initialize_dmp(delay).await.unwrap();

    mpu
}

pub static CALIBRATION_PARAMETERS: CalibrationParameters = CalibrationParameters::new(
    AccelFullScale::G2,
    GyroFullScale::Deg1000,
    ReferenceGravity::XN,
);

pub async fn save_calibrate(
    mpu: &mut Mpu6050<I2c<'_, Async>>,
    flash: peripherals::FLASH<'_>,
    delay: &mut Delay,
) {
    info!("Calibrating MPU6050 and saving to flash");
    let (accel, gyro) = mpu.calibrate(delay, &CALIBRATION_PARAMETERS).await.unwrap();

    MpuCalibration::new(flash).save(accel, gyro).await;
}

pub async fn load_calibrate(
    mpu: &mut Mpu6050<I2c<'_, Async>>,
    flash: peripherals::FLASH<'_>,
) -> Result<(), NotStored> {
    info!("Loading MPU6050 calibration from flash");

    let (accel, gyro) = MpuCalibration::new(flash).load().await?;

    mpu.set_accel_calibration(&accel).await.unwrap();
    mpu.set_gyro_calibration(&gyro).await.unwrap();

    Ok(())
}

static START_OF_STORAGE: u32 = 0x110000;
static CALIBRATION_ADDRESS: u32 = START_OF_STORAGE + 0x30;
static MAGIC_NUMBER: u64 = 0x90feff3d6ecbee5e;
const MAGIC_NUMBER_SIZE: usize = const { size_of_val(&MAGIC_NUMBER) };

struct MpuCalibration<'d> {
    pub flash: FlashStorage<'d>,
}

pub struct NotStored;

impl<'d> MpuCalibration<'d> {
    fn new(flash: peripherals::FLASH<'d>) -> Self {
        Self {
            flash: FlashStorage::new(flash),
        }
    }

    fn split_data_mut(
        data: &mut [u8; MAGIC_NUMBER_SIZE + 6 + 6],
    ) -> (&mut [u8; MAGIC_NUMBER_SIZE], &mut [u8; 6], &mut [u8; 6]) {
        let (magic_number, data) = data.split_first_chunk_mut::<MAGIC_NUMBER_SIZE>().unwrap();
        let (accel_bytes, gyro_bytes) = data.split_first_chunk_mut().unwrap();
        let gyro_bytes = gyro_bytes.try_into().unwrap();
        (magic_number, accel_bytes, gyro_bytes)
    }

    async fn save(&mut self, accel: Accel, gyro: Gyro) {
        let mut data = [0; MAGIC_NUMBER_SIZE + 6 + 6];

        let (magic_number, accel_bytes, gyro_bytes) = Self::split_data_mut(&mut data);

        *magic_number = MAGIC_NUMBER.to_ne_bytes();
        *accel_bytes = accel.to_bytes();
        *gyro_bytes = gyro.to_bytes();

        self.flash.write(CALIBRATION_ADDRESS, &data).unwrap();
    }

    async fn load(&mut self) -> Result<(Accel, Gyro), NotStored> {
        let mut data = [0; MAGIC_NUMBER_SIZE + 6 + 6];

        self.flash.read(CALIBRATION_ADDRESS, &mut data).unwrap();

        let (&mut magic_number, &mut accel_bytes, &mut gyro_bytes) =
            Self::split_data_mut(&mut data);

        if u64::from_ne_bytes(magic_number) != MAGIC_NUMBER {
            return Err(NotStored);
        }

        Ok((Accel::from_bytes(accel_bytes), Gyro::from_bytes(gyro_bytes)))
    }
}
