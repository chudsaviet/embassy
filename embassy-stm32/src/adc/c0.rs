use pac::adc::vals::Scandir;
#[allow(unused)]
use pac::adc::vals::{Adstp, Ckmode, Dmacfg, Exten, Ovsr};
use pac::adccommon::vals::Presc;

use super::{blocking_delay_us, Adc, AdcChannel, Instance, Resolution, RxDma, SampleTime, SealedAdcChannel};
use crate::dma::Transfer;
use crate::time::Hertz;
use crate::{pac, rcc, Peripheral};

/// Default VREF voltage used for sample conversion to millivolts.
pub const VREF_DEFAULT_MV: u32 = 3300;
/// VREF voltage used for factory calibration of VREFINTCAL register.
pub const VREF_CALIB_MV: u32 = 3300;

const MAX_ADC_CLK_FREQ: Hertz = Hertz::mhz(25);

const TIME_ADC_VOLTAGE_REGUALTOR_STARTUP_US: u32 = 20;

const TEMP_CHANNEL: u8 = 9;
const VREF_CHANNEL: u8 = 10;

// NOTE: Vrefint/Temperature/Vbat are not available on all ADCs,
// this currently cannot be modeled with stm32-data,
// so these are available from the software on all ADCs.
/// Internal voltage reference channel.
pub struct VrefInt;
impl<T: Instance> AdcChannel<T> for VrefInt {}
impl<T: Instance> SealedAdcChannel<T> for VrefInt {
    fn channel(&self) -> u8 {
        VREF_CHANNEL
    }
}

/// Internal temperature channel.
pub struct Temperature;
impl<T: Instance> AdcChannel<T> for Temperature {}
impl<T: Instance> SealedAdcChannel<T> for Temperature {
    fn channel(&self) -> u8 {
        TEMP_CHANNEL
    }
}

#[derive(Debug)]
pub enum Prescaler {
    NotDivided,
    DividedBy2,
    DividedBy4,
    DividedBy6,
    DividedBy8,
    DividedBy10,
    DividedBy12,
    DividedBy16,
    DividedBy32,
    DividedBy64,
    DividedBy128,
    DividedBy256,
}

impl Prescaler {
    fn from_ker_ck(frequency: Hertz) -> Self {
        let raw_prescaler = frequency.0 / MAX_ADC_CLK_FREQ.0;
        match raw_prescaler {
            0 => Self::NotDivided,
            1 => Self::DividedBy2,
            2..=3 => Self::DividedBy4,
            4..=5 => Self::DividedBy6,
            6..=7 => Self::DividedBy8,
            8..=9 => Self::DividedBy10,
            10..=11 => Self::DividedBy12,
            _ => unimplemented!(),
        }
    }

    #[allow(unused)]
    fn divisor(&self) -> u32 {
        match self {
            Prescaler::NotDivided => 1,
            Prescaler::DividedBy2 => 2,
            Prescaler::DividedBy4 => 4,
            Prescaler::DividedBy6 => 6,
            Prescaler::DividedBy8 => 8,
            Prescaler::DividedBy10 => 10,
            Prescaler::DividedBy12 => 12,
            Prescaler::DividedBy16 => 16,
            Prescaler::DividedBy32 => 32,
            Prescaler::DividedBy64 => 64,
            Prescaler::DividedBy128 => 128,
            Prescaler::DividedBy256 => 256,
        }
    }

    fn presc(&self) -> Presc {
        match self {
            Prescaler::NotDivided => Presc::DIV1,
            Prescaler::DividedBy2 => Presc::DIV2,
            Prescaler::DividedBy4 => Presc::DIV4,
            Prescaler::DividedBy6 => Presc::DIV6,
            Prescaler::DividedBy8 => Presc::DIV8,
            Prescaler::DividedBy10 => Presc::DIV10,
            Prescaler::DividedBy12 => Presc::DIV12,
            Prescaler::DividedBy16 => Presc::DIV16,
            Prescaler::DividedBy32 => Presc::DIV32,
            Prescaler::DividedBy64 => Presc::DIV64,
            Prescaler::DividedBy128 => Presc::DIV128,
            Prescaler::DividedBy256 => Presc::DIV256,
        }
    }
}

#[cfg(feature = "defmt")]
impl<'a> defmt::Format for Prescaler {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Prescaler::NotDivided => defmt::write!(fmt, "Prescaler::NotDivided"),
            Prescaler::DividedBy2 => defmt::write!(fmt, "Prescaler::DividedBy2"),
            Prescaler::DividedBy4 => defmt::write!(fmt, "Prescaler::DividedBy4"),
            Prescaler::DividedBy6 => defmt::write!(fmt, "Prescaler::DividedBy6"),
            Prescaler::DividedBy8 => defmt::write!(fmt, "Prescaler::DividedBy8"),
            Prescaler::DividedBy10 => defmt::write!(fmt, "Prescaler::DividedBy10"),
            Prescaler::DividedBy12 => defmt::write!(fmt, "Prescaler::DividedBy12"),
            Prescaler::DividedBy16 => defmt::write!(fmt, "Prescaler::DividedBy16"),
            Prescaler::DividedBy32 => defmt::write!(fmt, "Prescaler::DividedBy32"),
            Prescaler::DividedBy64 => defmt::write!(fmt, "Prescaler::DividedBy64"),
            Prescaler::DividedBy128 => defmt::write!(fmt, "Prescaler::DividedBy128"),
            Prescaler::DividedBy256 => defmt::write!(fmt, "Prescaler::DividedBy256"),
        }
    }
}

/// Number of samples used for averaging.
/// TODO: Implement hardware averaging setting.
#[allow(unused)]
pub enum Averaging {
    Disabled,
    Samples2,
    Samples4,
    Samples8,
    Samples16,
    Samples32,
    Samples64,
    Samples128,
    Samples256,
    Samples512,
    Samples1024,
}

impl<'d, T: Instance> Adc<'d, T> {
    /// Create a new ADC driver.
    pub fn new(adc: impl Peripheral<P = T> + 'd, sample_time: SampleTime) -> Self {
        embassy_hal_internal::into_ref!(adc);
        debug!("ADC RCC enable and reset.");
        rcc::enable_and_reset::<T>();

        debug!("Setting clock source.");
        T::regs().cfgr2().modify(|w| w.set_ckmode(Ckmode::SYSCLK));

        let prescaler = Prescaler::from_ker_ck(T::frequency());
        debug!("Initializing prescaler = {}.", prescaler);
        T::common_regs().ccr().modify(|w| w.set_presc(prescaler.presc()));

        let frequency = Hertz(T::frequency().0 / prescaler.divisor());
        info!("ADC frequency set to {} Hz", frequency.0);

        if frequency > MAX_ADC_CLK_FREQ {
            panic!("Maximal allowed frequency for the ADC is {} MHz and it varies with different packages, refer to ST docs for more information.", MAX_ADC_CLK_FREQ.0 /  1_000_000 );
        }

        let mut s = Self {
            adc,
            sample_time: SampleTime::from_bits(0),
        };

        debug!("ADC powerup.");
        s.power_up();

        debug!("ADC calibrate.");
        s.calibrate();
        blocking_delay_us(1);

        debug!("ADC enable.");
        s.enable();

        debug!("ADC configure.");
        s.configure_default();
        
        debug!("Set global sample time.");
        s.set_sample_time_all_channels(sample_time);

        s
    }

    fn power_up(&mut self) {
        T::regs().cr().modify(|reg| {
            reg.set_advregen(true);
        });

        // "The software must wait for the ADC voltage regulator startup time."
        // See datasheet for the value.
        blocking_delay_us(TIME_ADC_VOLTAGE_REGUALTOR_STARTUP_US + 1);
    }

    fn calibrate(&mut self) {
        // We have to make sure AUTOFF is OFF, but keep its value after calibration.
        let autoff_value = T::regs().cfgr1().read().autoff();
        T::regs().cfgr1().modify(|w| w.set_autoff(false));

        T::regs().cr().modify(|w| w.set_adcal(true));

        // "ADCAL bit stays at 1 during all the calibration sequence."
        // "It is then cleared by hardware as soon the calibration completes."
        while T::regs().cr().read().adcal() {}

        T::regs().cfgr1().modify(|w| w.set_autoff(autoff_value));
    }

    fn enable(&mut self) {
        T::regs().isr().modify(|w| w.set_adrdy(true));
        T::regs().cr().modify(|w| w.set_aden(true));
        // ADRDY is "ADC ready". Wait until it will be True.
        while !T::regs().isr().read().adrdy() {}
    }

    fn configure_default(&mut self) {
        // single conversion mode, software trigger
        T::regs().cfgr1().modify(|w| {
            w.set_cont(false);
            w.set_exten(Exten::DISABLED);
        });
    }

    /// Enable reading the voltage reference internal channel.
    pub fn enable_vrefint(&self) -> VrefInt {
        T::common_regs().ccr().modify(|reg| {
            reg.set_vrefen(true);
        });

        VrefInt {}
    }

    /// Enable reading the temperature internal channel.
    pub fn enable_temperature(&self) -> Temperature {
        debug!("Ensure that sample time is set to more than temperature sensor T_start from the datasheet!.");
        T::common_regs().ccr().modify(|reg| {
            reg.set_tsen(true);
        });

        Temperature {}
    }

    /// Set the ADC sample time.
    /// Shall only be called when ADC is not converting.
    pub fn set_sample_time_all_channels(&mut self, sample_time: SampleTime) {
        self.sample_time = sample_time;

        // Set all channels to use SMP1 field as source.
        T::regs().smpr().modify(|w| {
            w.smpsel(0);
            w.set_smp1(sample_time);
        });
    }

    /// Set the ADC resolution.
    pub fn set_resolution(&mut self, resolution: Resolution) {
        T::regs().cfgr1().modify(|reg| reg.set_res(resolution.into()));
    }

    /// Perform a single conversion.
    fn convert(&mut self) -> u16 {
        T::regs().isr().modify(|reg| {
            reg.set_eos(true);
            reg.set_eoc(true);
        });

        // Start conversion
        T::regs().cr().modify(|reg| {
            reg.set_adstart(true);
        });

        while !T::regs().isr().read().eos() {}

        T::regs().dr().read().0 as u16
    }

    /// Read an ADC channel.
    pub fn blocking_read(&mut self, channel: &mut impl AdcChannel<T>) -> u16 {
        self.read_channel(channel)
    }

    //// Set channels scanning direction.
    pub fn set_scandir(scandir: Scandir) {
        T::regs().cfgr1().modify(|reg| reg.set_scandir(scandir));
    }

    /// Read one or multiple ADC channels using DMA.
    pub async fn read(&mut self, rx_dma: &mut impl RxDma<T>, scandir: Scandir, readings: &mut [u16]) {
        // Ensure no conversions are ongoing.
        Self::cancel_conversions();

        Self::set_scandir(scandir);

        // Set continuous mode with oneshot dma.
        // Clear overrun flag before starting transfer.
        T::regs().isr().modify(|reg| {
            reg.set_ovr(true);
        });
        T::regs().cfgr1().modify(|reg| {
            reg.set_cont(true);
            reg.set_dmacfg(Dmacfg::DMA_ONE_SHOT);
        });

        let request = rx_dma.request();
        let transfer = unsafe {
            Transfer::new_read(
                rx_dma,
                request,
                T::regs().dr().as_ptr() as *mut u16,
                readings,
                Default::default(),
            )
        };

        // Start conversion.
        T::regs().cr().modify(|reg| {
            reg.set_adstart(true);
        });

        // Wait for conversion sequence to finish.
        transfer.await;

        // Ensure conversions are finished.
        Self::cancel_conversions();

        // Reset configuration.
        T::regs().cfgr1().modify(|reg| {
            reg.set_cont(false);
            reg.set_dmacfg(Dmacfg::from_bits(0));
        });
    }

    fn configure_channel(channel: &mut impl AdcChannel<T>) {
        channel.setup();
    }

    fn read_channel(&mut self, channel: &mut impl AdcChannel<T>) -> u16 {
        Self::configure_channel(channel);
        self.convert()
    }

    fn cancel_conversions() {
        if T::regs().cr().read().adstart() && !T::regs().cr().read().addis() {
            T::regs().cr().modify(|reg| {
                reg.set_adstp(Adstp::STOP);
            });
            while T::regs().cr().read().adstart() {}
        }
    }
}
