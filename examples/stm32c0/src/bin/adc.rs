#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = Default::default();
    let p = embassy_stm32::init(config);

    info!("ADC STM32C0 example.");

    // We need to set certain sample time to be able to read temp sensor.
    let mut adc = Adc::new(p.ADC1, SampleTime::CYCLES12_5);
    let mut temp = adc.enable_temperature();
    let mut vref = adc.enable_vrefint();

    loop {
        info!("ADC temperature sensor read: {}.", adc.blocking_read(&mut temp));
        info!("ADC vref read: {}.", adc.blocking_read(&mut vref));

        Timer::after_millis(3000).await;
    }
}
