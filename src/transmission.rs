use esp_hal::{
    peripherals,
    uart::{Config, UartTx},
};

pub fn init<'a>(
    uart: peripherals::UART1<'a>,
    tx: peripherals::GPIO9<'a>,
) -> UartTx<'a, esp_hal::Async> {
    UartTx::new(uart, Config::default())
        .unwrap()
        .with_tx(tx)
        .into_async()
}
