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
    const STRIP_COLOR: RGB8 = gamma_correct(RGB8::new(195, 103, 0));
    ws281x
        .write(particles.draw(strip_length).map(|b| scale(STRIP_COLOR, b)))
        .await
        .unwrap()
}

const fn gamma_correct(c: RGB8) -> RGB8 {
    const GAMMA8: [u8; 256] = [
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4,
        4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12,
        13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24,
        24, 25, 25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36, 37, 38, 39, 39, 40,
        41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50, 51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
        64, 66, 67, 68, 69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89, 90, 92, 93,
        95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114, 115, 117, 119, 120, 122, 124,
        126, 127, 129, 131, 133, 135, 137, 138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158,
        160, 162, 164, 167, 169, 171, 173, 175, 177, 180, 182, 184, 186, 189, 191, 193, 196, 198,
        200, 203, 205, 208, 210, 213, 215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244,
        247, 249, 252, 255,
    ];
    RGB8::new(
        GAMMA8[c.r as usize],
        GAMMA8[c.g as usize],
        GAMMA8[c.b as usize],
    )
}

fn scale(c: RGB8, amount: f32) -> RGB8 {
    let scale_channel = |ch: u8| -> u8 { (ch as f32 * amount) as u8 };
    RGB8::new(scale_channel(c.r), scale_channel(c.g), scale_channel(c.b))
}
