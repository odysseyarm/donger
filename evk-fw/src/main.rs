#![no_std]
#![no_main]

#[macro_use]
mod pinout;
mod pixart;

use defmt::{info, Debug2Format};
use embassy_executor::Spawner;
use embassy_nrf::{
    config::{Config, HfclkSource}, pac::{self, power::mainregstatus::MAINREGSTATUS_A}, peripherals::{self, SPI3}, spim::{self, Spim}, usb::{
        self,
        vbus_detect::HardwareVbusDetect,
        Driver,
    }, Peripheral
};
use embassy_usb::{
    driver::EndpointError, Builder, UsbDevice
};
use paj7025_nrf::Paj7025;
use static_cell::StaticCell;
use pixart::PixartClass;

use {defmt_rtt as _, panic_probe as _};

// this could go in pre_init, but it already works so /shrug
fn check_regout0() {
    // if the ats_mot_nrf52840 board is powered from USB
    // (high voltage mode), GPIO output voltage is set to 1.8 volts by
    // default and that is not enough for the vision sensors.
    // Increase GPIO voltage to 2.4 volts.

    let mainregstatus = unsafe { &*pac::POWER::ptr() }
        .mainregstatus
        .read()
        .mainregstatus()
        .variant();
    match mainregstatus {
        MAINREGSTATUS_A::NORMAL => return,
        MAINREGSTATUS_A::HIGH => (),
    }

    let regout0 = &unsafe { &*pac::UICR::ptr() }.regout0;
    if !regout0.read().vout().is_default() {
        return;
    }

    // Enable writing to flash
    let nvmc = unsafe { &*pac::NVMC::ptr() };
    nvmc.config.write(|w| w.wen().wen());
    while !nvmc.ready.read().ready().is_ready() {}

    // Set REGOUT0 to 2.4 volts
    regout0.write(|w| w.vout()._2v4());

    // Wait for write to finish
    while !nvmc.ready.read().ready().is_ready() {}

    pac::SCB::sys_reset()
}

embassy_nrf::bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    POWER_CLOCK => usb::vbus_detect::InterruptHandler;
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
    SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1 => spim::InterruptHandler<peripherals::TWISPI1>;
});

fn log_stuff() {
    let mainregstatus = unsafe { &*pac::POWER::ptr() }
        .mainregstatus
        .read()
        .mainregstatus()
        .variant();
    info!("MAINREGSTATUS is {}", Debug2Format(&mainregstatus));
    let regout0 = unsafe { &*pac::UICR::ptr() }
        .regout0
        .read()
        .vout()
        .variant();
    info!("REGOUT0 is {}", Debug2Format(&regout0));
    let hfclkstat = unsafe { &*pac::CLOCK::ptr() }.hfclkstat.read();
    info!(
        "HFCLKSTAT_SRC is {}",
        Debug2Format(&hfclkstat.src().variant())
    );
    info!(
        "HFCLKSTAT_STATE is {}",
        Debug2Format(&hfclkstat.state().variant())
    );
}

const NUM_BUFFERS: usize = 6;
static SHARED_BUFFERS: StaticCell<[[u8; 98*98 + 98*3]; NUM_BUFFERS]> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // enable instruction cache
    // unsafe { &*pac::NVMC::ptr() }.icachecnf.write(|w| w.cacheen().enabled());
    check_regout0();
    let mut config = Config::default();
    config.hfclk_source = HfclkSource::ExternalXtal;
    let mut p = embassy_nrf::init(config);

    log_stuff();

    let wide = Paj7025::new(
        Spim::new(
            p.SPI3,
            Irqs,
            pinout!(p.wf_sck),
            pinout!(p.wf_miso),
            pinout!(p.wf_mosi),
            Default::default(),
        ),
        pinout!(p.wf_cs),
        pinout!(p.wf_fod),
    ).await;
    let near = Paj7025::new(
        Spim::new(
            &mut p.TWISPI1,
            Irqs,
            &mut pinout!(p.nf_sck),
            &mut pinout!(p.nf_miso),
            &mut pinout!(p.nf_mosi),
            Default::default(),
        ),
        &mut pinout!(p.nf_cs),
        &mut pinout!(p.nf_fod),
    ).await;
    // Run the USB device.
    let mut usb = usb_device(p.USBD, wide);
    usb.run().await;
}

#[embassy_executor::task]
async fn run_usb(mut device: UsbDevice<'static, embassy_nrf::usb::Driver<'static, embassy_nrf::peripherals::USBD, HardwareVbusDetect>>) -> ! {
    device.run().await
}

/// Panics if called more than once.
fn usb_device(
    p: impl Peripheral<P = peripherals::USBD> + 'static,
    sensor: Paj7025<'static, SPI3>,
) -> UsbDevice<'static, usb::Driver<'static, peripherals::USBD, HardwareVbusDetect>>
{
    // Create the driver, from the HAL.
    let driver = Driver::new(p, Irqs, HardwareVbusDetect::new(Irqs));

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0x093a, 0x0207);
    config.manufacturer = Some("thingo");
    config.product = Some("donger");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    static DEVICE_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static PIXART_STATE: StaticCell<pixart::State<'_, SPI3>> = StaticCell::new();

    let device_descriptor = DEVICE_DESCRIPTOR.init_with(|| [0; 256]);
    let config_descriptor = CONFIG_DESCRIPTOR.init_with(|| [0; 256]);
    let bos_descriptor = BOS_DESCRIPTOR.init_with(|| [0; 256]);
    let msos_descriptor = MSOS_DESCRIPTOR.init_with(|| [0; 256]);
    let control_buf = CONTROL_BUF.init_with(|| [0; 64]);
    let pixart_state = PIXART_STATE.init_with(move || pixart::State::new(sensor));

    let mut builder = Builder::new(
        driver,
        config,
        device_descriptor,
        config_descriptor,
        bos_descriptor,
        msos_descriptor,
        control_buf,
    );

    // Create classes on the builder.
    PixartClass::new(&mut builder, pixart_state);

    // Build the builder.
    let usb = builder.build();
    usb
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}
