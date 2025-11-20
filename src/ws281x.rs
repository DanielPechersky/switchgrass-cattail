use esp_hal::{
    Async, peripherals,
    spi::master::{Config, Spi},
    time::Rate,
};
use smart_leds::RGB8;
use ws2812_async::{OrderedColors, Ws2812};

pub struct Brg;

impl OrderedColors for Brg {
    fn order(color: RGB8) -> [u8; 3] {
        [color.b, color.r, color.g]
    }
}

/// N = 12 * NUM_LEDS
pub fn init<'a, const N: usize>(
    spi: peripherals::SPI2<'a>,
    mosi: peripherals::GPIO10<'a>,
) -> Ws2812<Spi<'a, Async>, Brg, N> {
    let spi = Spi::new(spi, Config::default().with_frequency(Rate::from_mhz(3)))
        .unwrap()
        .with_mosi(mosi)
        .into_async();
    ws2812_async::Ws2812::new(spi)
}
