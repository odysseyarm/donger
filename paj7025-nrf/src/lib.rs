#![no_std]
// TODO remove all the nrf52840 (and embassy?) specific stuff
use defmt::{info, trace};
use embassy_embedded_hal::SetConfig;
use embassy_nrf::{
    gpio::{AnyPin, Level, Output, OutputDrive, Pin}, pac, spim::{Config, Frequency, Instance, Spim, MODE_3}, Peripheral
};
///
/// Nothing here is cancel safe, if a future is dropped, the device will be left in an
/// indeterminate state.
pub struct Paj7025<'d, T: Instance> {
    spim: Spim<'d, T>,
    cs: Output<'d, AnyPin>,
    fod: Output<'d, AnyPin>,
}

impl<'d, T: Instance> Paj7025<'d, T> {
    pub async fn write_register(&mut self, address: u8, data: u8) {
        let cmd = [0];
        self.cs.set_low();
        self.spim.write(&cmd).await.unwrap();
        self.spim.write(&[address]).await.unwrap();
        self.spim.write(&[data]).await.unwrap();
        self.cs.set_high();
    }
    pub fn write_register_blocking(&mut self, address: u8, data: u8) {
        let cmd = [0];
        self.cs.set_low();
        self.spim.blocking_write(&cmd).unwrap();
        self.spim.blocking_write(&[address]).unwrap();
        self.spim.blocking_write(&[data]).unwrap();
        self.cs.set_high();
    }

    pub async fn read_register(&mut self, address: u8) -> u8 {
        let cmd = [1 << 7];
        self.cs.set_low();
        self.spim.write(&cmd).await.unwrap();
        self.spim.write(&[address]).await.unwrap();
        let mut data = [0];
        self.spim.read(&mut data).await.unwrap();
        self.cs.set_high();
        data[0]
    }

    pub fn read_register_blocking(&mut self, address: u8) -> u8 {
        let cmd = [1 << 7];
        self.cs.set_low();
        self.spim.blocking_write(&cmd).unwrap();
        self.spim.blocking_write(&[address]).unwrap();
        let mut data = [0];
        self.spim.blocking_read(&mut data).unwrap();
        self.cs.set_high();
        data[0]
    }

    pub fn read_register_slice_blocking(&mut self, address: u8, buf: &mut [u8]) {
        if buf.len() == 0 {
            return;
        } else if buf.len() == 1 {
            buf[0] = self.read_register_blocking(address);
            return;
        }

        let cmd = [(1 << 7) | 1];
        self.cs.set_low();
        self.spim.blocking_write(&cmd).unwrap();
        self.spim.blocking_write(&[address]).unwrap();
        self.spim.blocking_read(buf).unwrap();
        self.cs.set_high();
    }

    pub async fn read_register_array<const N: usize>(&mut self, address: u8, buf: &mut [u8; N]) {
        if N == 0 {
            return;
        } else if N == 1 {
            buf[0] = self.read_register(address).await;
            return;
        }

        let cmd = [(1 << 7) | 1];
        self.cs.set_low();
        self.spim.write(&cmd).await.unwrap();
        self.spim.write(&[address]).await.unwrap();
        self.spim.read(buf).await.unwrap();
        self.cs.set_high();
    }

    pub async fn write_register_array<const N: usize>(&mut self, address: u8, buf: &[u8; N]) {
        if N == 0 {
            return;
        } else if N == 1 {
            self.write_register(address, buf[0]).await;
            return;
        }

        let cmd = [1];
        self.cs.set_low();
        self.spim.write(&cmd).await.unwrap();
        self.spim.write(&[address]).await.unwrap();
        self.spim.write(buf).await.unwrap();
        self.cs.set_high();
    }

    pub fn write_register_array_blocking<const N: usize>(&mut self, address: u8, buf: &[u8; N]) {
        if N == 0 {
            return;
        } else if N == 1 {
            self.write_register_blocking(address, buf[0]);
            return;
        }

        let cmd = [1];
        self.cs.set_low();
        self.spim.blocking_write(&cmd).unwrap();
        self.spim.blocking_write(&[address]).unwrap();
        self.spim.blocking_write(buf).unwrap();
        self.cs.set_high();
    }

    pub async fn set_bank(&mut self, bank: u8) {
        self.write_register(0xef, bank).await;
    }

    pub async fn trigger_fod(&mut self) {
        self.fod.set_high();
        // could replace this with clock + ppi
        embassy_time::Timer::after_micros(1).await;
        self.fod.set_low();
    }

    pub async fn new<P1: Pin, P2: Pin>(
        mut spim: Spim<'d, T>,
        cs: impl Peripheral<P = P1> + 'd,
        fod: impl Peripheral<P = P2> + 'd,
    ) -> Self {
        trace!("Paj7025::new");
        let cs = Output::new(cs.into_ref().map_into(), Level::High, OutputDrive::Standard);
        let fod = Output::new(fod.into_ref().map_into(), Level::High, OutputDrive::Standard);
        let mut config = Config::default();
        config.frequency = Frequency::M8;
        config.mode = MODE_3;
        // https://github.com/embassy-rs/embassy/pull/2485 :pray:
        // config.bit_order = BitOrder::LSB_FIRST;
        spim.set_config(&config).unwrap();
        regs::<T>().config.write(|w| w.order().lsb_first());
        let mut paj = Self { spim, cs, fod };
        paj.initial_power().await;
        info!("Paj7025 initial power");

        // Check Product ID
        let mut retry_cnt = 0;
        loop {
            embassy_time::Timer::after_millis(1).await;
            let mut buf = [0; 2];
            // Bank is already set to 0x00 by initial_power()
            paj.read_register_array(0x02, &mut buf).await;
            let product_id = u16::from_le_bytes(buf);
            info!("Read Product ID, got {:x}", product_id);
            if product_id == 0x7025 {
                break;
            } else {
                if retry_cnt == 3 {
                    panic!("Sadge");
                }
                info!("Retrying in 1 ms");
                retry_cnt += 1;
            }
        }
        paj.initialize_settings().await;
        // paj.initialize_settings2();
        paj
    }

    /// Transition from Ready to Operation state.
    async fn initial_power(&mut self) {
        self.set_bank(0x00).await;
        self.write_register(0x2f, 0x05).await;
        self.write_register(0x30, 0x00).await;
        self.write_register(0x30, 0x01).await;
    }

    pub async fn initialize_settings2(&mut self) {
        let mut bank = 0x00;
        self.write_register(0xef, bank).await;     //Switching RegBank to Bank0

        let mut tmp_data = 0x00;
        self.write_register(0xdc, tmp_data).await; //internal_system_control_disable

        tmp_data = 0x04;
        self.write_register(0xfb, tmp_data).await; //[2]LEDDAC disable

        tmp_data = 0x05;
        self.write_register(0x2f, tmp_data).await; //sensor_on

        tmp_data = 0x00;
        self.write_register(0x30, tmp_data).await; //Manual_PowerControl_Update_Req

        tmp_data = 0x01;
        self.write_register(0x30, tmp_data).await; //Manual_PowerControl_Update_Req

        tmp_data = 0x00;
        self.write_register(0x1f, tmp_data).await; //freerun_irtx_disable

        bank = 0x01;
        self.write_register(0xef, bank).await;     //Switching RegBank to Bank1

        tmp_data = 0x00;
        self.write_register(0x2d, tmp_data).await; //V flip

        bank = 0x0c;
        self.write_register(0xef, bank).await;     //Switching RegBank to Bank12

        let tmp_data_buf_xy = [(4095 & 0xff) as u8, (4095 >> 8) as u8];
        self.write_register_array(0x60, &tmp_data_buf_xy).await;
        self.write_register_array(0x62, &tmp_data_buf_xy).await;

        tmp_data = 0x00;
        self.write_register(0x64, tmp_data).await; //G0 mode setting
        tmp_data = 0x00;
        self.write_register(0x65, tmp_data).await; //G1 mode setting
        tmp_data = 0x00;
        self.write_register(0x66, tmp_data).await; //G2 mode setting
        tmp_data = 0x00;
        self.write_register(0x67, tmp_data).await; //G3 mode setting
        tmp_data = 0x00;
        self.write_register(0x68, tmp_data).await; //G4 mode setting
        tmp_data = 0x00;
        self.write_register(0x69, tmp_data).await; //G5 mode setting
        tmp_data = 0x07; // FOD mode
        self.write_register(0x6a, tmp_data).await; //G6 mode setting
        tmp_data = 0x00;
        self.write_register(0x6b, tmp_data).await; //G7 mode setting
        tmp_data = 0x00;
        self.write_register(0x6c, tmp_data).await; //G8 mode setting
        tmp_data = 0x00;
        self.write_register(0x71, tmp_data).await; //G13 mode setting
        tmp_data = 0x00;
        self.write_register(0x72, tmp_data).await; //G14 mode setting
        tmp_data = 0x00;
        self.write_register(0x12, tmp_data).await; //keyscan disable
        tmp_data = 0x00;
        self.write_register(0x13, tmp_data).await; //keyscan disable
        bank = 0x00;
        self.write_register(0xef, bank).await;     //Switching RegBank to Bank0
        tmp_data = 0x01;
        self.write_register(0x01, tmp_data).await; //update flag enable

        bank = 0x0c;
        self.write_register(0xef, bank).await;     //Switching RegBank to Bank12
        tmp_data = 0x10;
        self.write_register(0x0b, tmp_data).await; //global = 16
        tmp_data = 0x03; // should be 0 for nearfield
        self.write_register(0x0c, tmp_data).await; //ggh=0 total gain=2x
        tmp_data = 0x00;
        self.write_register(0x0f, tmp_data).await; //Texp=8192
        tmp_data = 0x20;
        self.write_register(0x10, tmp_data).await; //Texp=8192
        tmp_data = 0x00;
        self.write_register(0x46, tmp_data).await; //oalb
        tmp_data = 0x6e;
        self.write_register(0x47, tmp_data).await; //Yth = 110

        bank = 0x01;
        self.write_register(0xef, bank).await;     //Switching RegBank to Bank1
        tmp_data = 0x01;
        self.write_register(0x01, tmp_data).await; //update flag enable
    }
    async fn initialize_settings(&mut self) {
        self.write_register(0xEF, 0x00).await;
        self.write_register(0xDC, 0x00).await;
        self.write_register(0xFB, 0x04).await;
        self.write_register(0xEF, 0x00).await;
        self.write_register(0x2F, 0x05).await;
        self.write_register(0x30, 0x00).await;
        self.write_register(0x30, 0x01).await;
        self.write_register(0x1F, 0x00).await;
        self.write_register(0xEF, 0x01).await;
        self.write_register(0x2D, 0x00).await;
        self.write_register(0xEF, 0x0C).await;
        self.write_register(0x64, 0x00).await;
        self.write_register(0x65, 0x00).await;
        self.write_register(0x66, 0x00).await;
        self.write_register(0x67, 0x00).await;
        self.write_register(0x68, 0x00).await;
        self.write_register(0x69, 0x00).await;
        self.write_register(0x6A, 0x00).await;
        self.write_register(0x6B, 0x00).await;
        self.write_register(0x6C, 0x00).await;
        self.write_register(0x71, 0x00).await;
        self.write_register(0x72, 0x00).await;
        self.write_register(0x12, 0x00).await;
        self.write_register(0x13, 0x00).await;
        self.write_register(0xEF, 0x00).await;
        self.write_register(0x01, 0x01).await;
    }

    /// Transition from Operation to Power Down state.
    pub async fn power_off(&mut self) {
        self.set_bank(0x00).await;
        self.write_register(0x2f, 0x00).await;
        self.write_register(0x30, 0x00).await;
        self.write_register(0x30, 0x01).await;
    }

    /// Transition from Power Down to Operation state.
    pub async fn power_on(&mut self) {
        self.read_register(0x02).await;
        embassy_time::Timer::after_millis(1).await;
        self.initial_power().await;
    }

    /// Begin image mode. This consumes the driver, freeing the pins to be reassigned to the image
    /// mode driver.
    pub async fn image_mode(mut self) {
        info!("Entering image mode");
        self.write_register(0xef, 0x00).await;
        self.write_register(0xdc, 0x00).await;
        self.write_register(0xfb, 0x00).await;
        self.write_register(0x7a, 0x07).await;
        self.write_register(0x72, 0x03).await;
        self.write_register(0x82, 0x03).await;
        self.write_register(0x2f, 0x07).await;
        self.write_register(0x30, 0x00).await;
        self.write_register(0x30, 0x01).await;
        self.write_register(0xdc, 0x00).await;
        self.write_register(0xef, 0x0c).await;
        self.write_register(0x6d, 0x06).await;
        self.write_register(0x6e, 0x06).await;
        self.write_register(0x6f, 0x06).await;
        self.write_register(0x70, 0x06).await;
        self.write_register(0xef, 0x00).await;
        self.write_register(0x01, 0x01).await;
    }
}

fn regs<I: Instance>() -> &'static pac::spim0::RegisterBlock {
    I::regs()
}
