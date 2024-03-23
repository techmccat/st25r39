#![no_main]
#![no_std]

use stm32g4xx_hal as hal;
use st25r39_examples as _;

use st25r39::{SpiInterface, ST25R3916};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use hal::{prelude::*, stm32::Peripherals, pwr::PwrExt, rcc::{Config, SysClockSrc}, time::RateExtU32, spi};

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = rcc.freeze(Config::new(SysClockSrc::HSE(24.MHz())), pwr);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();

    let irq = gpioa.pa0.into_floating_input();
    let mut cs = gpiob.pb6.into_push_pull_output();
    cs.set_high().unwrap();

    let spi_bus = dp.SPI1.spi((sck, miso, mosi), spi::MODE_1, 4.MHz(), &mut rcc);
    defmt::info!("SPI init done");
    let interface = SpiInterface::new(ExclusiveDevice::new(spi_bus, cs, NoDelay));

    let mut driver = ST25R3916::init(interface, irq).unwrap();
    let mv = driver.adjust_regulators().unwrap();
    defmt::info!("Regulated voltage: {=u16}mV", mv);

    st25r39_examples::exit()
}
