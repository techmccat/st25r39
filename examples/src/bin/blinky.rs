#![no_main]
#![no_std]

use stm32g4xx_hal as hal;
use st25r39_examples as _;

use st25r39::{SpiInterface, ST25R3916, GPTDuration};
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
    let gpioc = dp.GPIOC.split(&mut rcc);

    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();

    let irq = gpioa.pa0.into_floating_input();
    let mut cs = gpiob.pb6.into_push_pull_output();
    cs.set_high().unwrap();
    // MCU LEDs on NFC08A1
    let mut leds = (
        gpioa.pa1.into_push_pull_output(),
        gpioa.pa4.into_push_pull_output(),
        gpiob.pb0.into_push_pull_output(),
        gpioc.pc1.into_push_pull_output(),
        gpioc.pc0.into_push_pull_output(),
    );
    // stm32g4xx-hal doesn't yet have complete gpio type erasure
    let mut leds_dyn: [&mut dyn StatefulOutputPin<Error = core::convert::Infallible>; 5] = [
        &mut leds.0,
        &mut leds.1,
        &mut leds.2,
        &mut leds.3,
        &mut leds.4,
    ];

    let spi_bus = dp.SPI1.spi((sck, miso, mosi), spi::MODE_1, 4.MHz(), &mut rcc);
    defmt::info!("SPI init done");
    let interface = SpiInterface::new(ExclusiveDevice::new(spi_bus, cs, NoDelay));

    let mut driver = ST25R3916::init(interface, irq).unwrap();

    loop {
        for led in leds_dyn.iter_mut() {
            let _ = led.toggle();
            driver.delay(GPTDuration::millis(200)).unwrap();
        }
    }
}
