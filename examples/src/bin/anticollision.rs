#![no_main]
#![no_std]

use st25r39_examples as _;
use stm32g4xx_hal as hal;

use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use hal::{
    delay::DelayFromCountDownTimer,
    prelude::*,
    pwr::PwrExt,
    rcc::{Config, SysClockSrc},
    spi,
    stm32::Peripherals,
    time::{ExtU32, RateExtU32},
    timer::Timer,
};
use st25r39::{
    nfc_a::{AntiCollisionHeader, CascadeLevel, self},
    SpiInterface, ST25R3916,
};

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = rcc.freeze(Config::new(SysClockSrc::HSE(24.MHz())), pwr);

    let timer = Timer::new(dp.TIM16, &rcc.clocks);
    let var_name = DelayFromCountDownTimer::new(timer.start_count_down(1.secs()));
    let mut delay = var_name;

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();

    let irq = gpioa.pa0.into_floating_input();
    let mut cs = gpiob.pb6.into_push_pull_output();
    cs.set_high().unwrap();

    let spi_bus = dp
        .SPI1
        .spi((sck, miso, mosi), spi::MODE_1, 4.MHz(), &mut rcc);
    let interface = SpiInterface::new(ExclusiveDevice::new(spi_bus, cs, NoDelay));

    let mut driver = ST25R3916::init(interface, irq).unwrap();
    let _ = driver.adjust_regulators().unwrap();

    let mut nfc_a = driver.into_iso14443a_initiator().unwrap();
    let mut rx_buf = [0u8; 2];

    defmt::info!("Waiting for card");
    loop {
        let res = nfc_a.detect_presence(&mut rx_buf);
        if let Ok(true) = res {
            defmt::warn!("Card detected, ATQA = {=[u8;2]:02X}", rx_buf);
            let mut id_buf = [0; 5];
            let acl_res = nfc_a.transceive_anticollision_frame(
                AntiCollisionHeader::new(CascadeLevel::One, 16),
                &mut id_buf,
                nfc_a::FDT_A_LISTEN_RELAXED,
            );
            match acl_res {
                Ok(bits) => defmt::warn!("Received {=u8} bits, buf = {=[u8;5]:#x}", bits as u8, id_buf),
                Err(e) => defmt::error!("{}", e),
            }
        }
        rx_buf.fill(0);
        delay.delay_ms(1000);
    }
}
