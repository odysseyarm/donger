#![no_std]
#![no_main]

mod paj7025;
#[macro_use]
mod pinout;

use core::mem::swap;

use defmt::{info, Debug2Format};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::{
    config::{Config, HfclkSource}, gpio::{self, Input, Level, Output, OutputDrive, Pull}, gpiote::{self, InputChannel, InputChannelPolarity, OutputChannel, OutputChannelPolarity}, pac::{self, power::mainregstatus::MAINREGSTATUS_A}, peripherals, ppi::Ppi, spim::{self, Spim}, spis::{self, Spis}, usb::{
        self,
        vbus_detect::{HardwareVbusDetect, VbusDetect},
        Driver, Instance,
    }, Peripheral
};
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State}, driver::EndpointError, Builder, UsbDevice
};
use paj7025::Paj7025;
use static_cell::StaticCell;
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
    SPIM2_SPIS2_SPI2 => spis::InterruptHandler<peripherals::SPI2>;
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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    check_regout0();
    let mut config = Config::default();
    config.hfclk_source = HfclkSource::ExternalXtal;
    let mut p = embassy_nrf::init(config);

    log_stuff();

    let mut wide = Paj7025::new(
        Spim::new(
            &mut p.SPI3,
            Irqs,
            &mut pinout!(p.wf_sck),
            &mut pinout!(p.wf_miso),
            &mut pinout!(p.wf_mosi),
            Default::default(),
        ),
        &mut pinout!(p.wf_cs),
        &mut pinout!(p.wf_fod),
    )
    .await;
    // Run the USB device.
    let (mut class, usb) = usb_device(p.USBD);
    spawner.must_spawn(run_usb(usb));
    class.wait_connection().await;

    // Wait for something to get sent
    static FEATURE_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let feature_buf = FEATURE_BUF.init_with(|| [0; 256]);
    loop {
        let v = wait_for_serial(&mut class).await;
        if v == b'a' {
            break;
        } else {
            wide.trigger_fod().await;
            embassy_time::Timer::after_millis(1000/200).await;
            wide.set_bank(0x05).await;
            wide.read_register_array(0x00, feature_buf).await;
            write_serial(&mut class, feature_buf).await;
        }
    }

    info!("Transitioning to image mode");
    wide.image_mode().await;

    // let mut cursed_spim = paj7025::image_mode_spim(
    //     &mut p.SPI3,
    //     Irqs,
    //     &mut p.P1_00, // sample SCK
    // );

    // let mut cs = gpio::Input::new(p.P0_06, Pull::None);
    // cs.wait_for_falling_edge().await;
    // let mut data = [0; 32];
    // cursed_spim.read(&mut data).await;
    // info!("{:08b}", &data);

    let mut config = spis::Config::default();
    config.mode = spis::MODE_3;
    let mut spis = Spis::new_rxonly(
        &mut p.SPI2,
        Irqs,
        &mut pinout!(p.wf_cs),
        &mut pinout!(p.wf_sck),
        &mut pinout!(p.wf_mosi),
        config,
    );
    static DATA1: StaticCell<[u8; 98*98 + 98*3]> = StaticCell::new();
    static DATA2: StaticCell<[u8; 98*98 + 98*3]> = StaticCell::new();
    let mut data1 = DATA1.init_with(|| [0; 98*98 + 98*3]);
    let mut data2 = DATA2.init_with(|| [0; 98*98 + 98*3]);
    let mut result = spis.read(data1).await;
    loop {
        if let Ok(len) = result {
            swap(&mut data1, &mut data2);
            let fut = async {
                let len16 = len as u16;
                write_serial(&mut class, &len16.to_le_bytes()).await;
                write_serial(&mut class, &data2[..len]).await;
                let c = wait_for_serial(&mut class).await;
            };
            (result, _) = join(spis.read(data1), fut).await;
        }
    }
}

async fn write_serial<'d, D: embassy_usb::driver::Driver<'d>>(class: &mut CdcAcmClass<'d, D>, data: &[u8]) {
    let max_packet_size = usize::from(class.max_packet_size());
    for chunk in data.chunks(max_packet_size) {
        class.write_packet(chunk).await.unwrap();
    }
    if data.len() % usize::from(max_packet_size) == 0 {
        class.write_packet(&[]).await.unwrap();
    }
}

async fn wait_for_serial<'d, D: embassy_usb::driver::Driver<'d>>(class: &mut CdcAcmClass<'d, D>) -> u8 {
    'outer: loop {
        let mut buf = [0; 64];
        loop {
            let Ok(_n) = class.read_packet(&mut buf).await else { break };
            return buf[0];
        }
    }
}

#[embassy_executor::task]
async fn run_usb(mut device: UsbDevice<'static, embassy_nrf::usb::Driver<'static, embassy_nrf::peripherals::USBD, HardwareVbusDetect>>) -> ! {
    device.run().await
}

/// Panics if called more than once.
fn usb_device(p: impl Peripheral<P = peripherals::USBD> + 'static) -> (
    CdcAcmClass<'static, Driver<'static, peripherals::USBD, HardwareVbusDetect>>,
    UsbDevice<'static, usb::Driver<'static, peripherals::USBD, HardwareVbusDetect>>,
) {
    // Create the driver, from the HAL.
    let driver = Driver::new(p, Irqs, HardwareVbusDetect::new(Irqs));

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    static DEVICE_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<State> = StaticCell::new();

    let device_descriptor = DEVICE_DESCRIPTOR.init_with(|| [0; 256]);
    let config_descriptor = CONFIG_DESCRIPTOR.init_with(|| [0; 256]);
    let bos_descriptor = BOS_DESCRIPTOR.init_with(|| [0; 256]);
    let msos_descriptor = MSOS_DESCRIPTOR.init_with(|| [0; 256]);
    let control_buf = CONTROL_BUF.init_with(|| [0; 64]);

    let state = STATE.init_with(State::new);
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
    let class = CdcAcmClass::new(&mut builder, state, 64);

    // Build the builder.
    let usb = builder.build();
    (class, usb)
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

// #[embassy_executor::main]
async fn _blinky(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let mut led1 = Output::new(p.P0_06, Level::Low, OutputDrive::Standard);
    let blue_led = Output::new(p.P0_12, Level::Low, OutputDrive::Standard);
    let button = Input::new(p.P1_06, Pull::Up);
    let button_ch = InputChannel::new(p.GPIOTE_CH0, button, InputChannelPolarity::HiToLo);
    let led_ch = OutputChannel::new(p.GPIOTE_CH1, blue_led, OutputChannelPolarity::Toggle);
    let mut ppi_ch = Ppi::new_one_to_one(p.PPI_CH0, button_ch.event_in(), led_ch.task_out());
    ppi_ch.enable();

    loop {
        embassy_time::Timer::after_secs(1).await;
        led1.set_high();
        embassy_time::Timer::after_secs(1).await;
        led1.set_low();
        // led.set_high();
        // button.wait_for_low().await;
        // info!("button pressed");
        // led.set_low();
        // button.wait_for_high().await;
    }
}
