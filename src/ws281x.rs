use core::fmt::Debug;

use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers, peripherals,
    spi::master::{Config, Spi, SpiDmaBus},
    time::Rate,
};
use smart_leds::{RGB8, SmartLedsWriteAsync};
use ws2812_async::Rgb;

use crate::particles::Particles;

pub type Ws281x<'a, const N: usize> = ws2812_async::Ws2812<SpiDmaBus<'a, esp_hal::Async>, Rgb, N>;

/// N = 12 * NUM_LEDS
pub fn init<'a, const N: usize>(
    spi: peripherals::SPI2<'a>,
    mosi: peripherals::GPIO10<'a>,
    dma: peripherals::DMA_CH0<'a>,
) -> Ws281x<'a, N> {
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi = Spi::new(spi, Config::default().with_frequency(Rate::from_mhz(3)))
        .unwrap()
        .with_mosi(mosi)
        .with_dma(dma)
        .with_buffers(dma_rx_buf, dma_tx_buf)
        .into_async();
    ws2812_async::Ws2812::new(spi)
}

pub async fn write_particles<W: SmartLedsWriteAsync>(
    ws281x: &mut W,
    particles: &Particles,
    strip_length: usize,
) where
    W::Color: From<RGB8>,
    W::Error: Debug,
{
    const STRIP_COLOR: RGB8 = RGB8::new(90, 20, 0);
    ws281x
        .write(particles.draw(strip_length).map(|b| scale(STRIP_COLOR, b)))
        .await
        .unwrap()
}

fn scale(c: RGB8, amount: f32) -> RGB8 {
    let scale_channel = |ch: u8| -> u8 { (ch as f32 * amount) as u8 };
    RGB8::new(scale_channel(c.r), scale_channel(c.g), scale_channel(c.b))
}
