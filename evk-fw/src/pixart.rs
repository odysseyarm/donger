use core::{cell::{Cell, RefCell}, mem::MaybeUninit};

use defmt::{info, trace, warn};
use embassy_nrf::spim;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_usb::{
    control::{InResponse, OutResponse, Request, RequestType},
    driver::{Driver, Endpoint, EndpointIn},
    Builder,
};

use paj7025_nrf::Paj7025;

pub struct Shared<'d, T: spim::Instance> {
    sensor: RefCell<Paj7025<'d, T>>,
    running: Cell<bool>,
}

pub struct State<'d, T: spim::Instance> {
    shared: Shared<'d, T>,
    handler: MaybeUninit<ControlHandler<'d, T>>, // scary self reference
}

pub struct PixartClass<'d, D: Driver<'d>, T: spim::Instance> {
    bulk_ep: D::EndpointIn,
    shared: &'d Shared<'d, T>,
}

impl<'d, D: Driver<'d>, T: spim::Instance> PixartClass<'d, D, T> {
    pub fn new(builder: &mut Builder<'d, D>, state: &'d mut State<'d, T>) -> Self {
        let handler = state.handler.write(ControlHandler { shared: &state.shared });
        builder.handler(handler);
        let mut function = builder.function(255, 0, 0);
        let mut interface = function.interface();
        let mut setting = interface.alt_setting(255, 0, 0, None);
        let _ep1 = setting.endpoint_interrupt_in(64, 4);
        let bulk_ep = setting.endpoint_bulk_in(64);

        Self { bulk_ep, shared: &state.shared }
    }

    pub async fn run(&mut self) {
        let mut counter = 0u32;
        loop {
            if self.shared.running.get() {
                if let Ok(mut sensor) = self.shared.sensor.try_borrow_mut() {
                    // idk what this is for
                    let c = counter.to_le_bytes();
                    let _ = self.bulk_ep.write(&[0xff, 0xff, 0x00, 0xff, 0x96, 0x64, c[0], c[1], c[2], c[3]]).await.inspect_err(|_| warn!("failed to send thingo"));
                    counter = counter.wrapping_add(1);

                    // send feature data
                    sensor.write_register(0xef, 0x05).await;
                    let mut feature_data_buf = [0; 257];
                    let [feature_data @ .., cnt] = &mut feature_data_buf;
                    sensor.read_register_array(0x00, feature_data).await;
                    *cnt = counter as u8;
                    let info = self.bulk_ep.info();
                    let max_pkt_size = usize::from(info.max_packet_size);
                    for chunk in feature_data_buf.chunks(max_pkt_size) {
                        let _ = self.bulk_ep.write(chunk).await.inspect_err(|_| warn!("failed to send feature data"));
                    }
                    if feature_data_buf.len() % max_pkt_size == 0 {
                        let _ = self.bulk_ep.write(&[]).await.inspect_err(|_| warn!("failed to send feature data"));
                    }

                }
            }
            embassy_time::Timer::after_millis(5).await;
        }
    }
}

impl<'d, T: spim::Instance> State<'d, T> {
    pub fn new(sensor: Paj7025<'d, T>) -> Self {
        Self {
            shared: Shared {
                sensor: RefCell::new(sensor),
                running: Cell::new(false),
            },
            handler: MaybeUninit::uninit(),
        }
    }
}

struct ControlHandler<'d, T: spim::Instance> {
    shared: &'d Shared<'d, T>,
}

impl<'d, T: spim::Instance> embassy_usb::Handler for ControlHandler<'d, T> {
    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        trace!("Received control out bRequest = {}, wIndex = {}, data = {}", req.request, req.index, data);
        let should_handle = req.request_type == RequestType::Vendor && matches!(req.request, 178 | 187);
        if !should_handle {
            return None;
        }

        if req.index == 0x5501 {
            // I think initialization is supposed to happen here, but we do initialization
            // beforehand so just look at the value in `data` to enable/disable the stream.
            if let &[v] = data {
                trace!("running set to {}", v != 0);
                self.shared.running.set(v != 0);
            }
        } else if let &[v] = data {
            self.shared.sensor.borrow_mut().write_register_blocking(req.index as u8, v);
        }
        Some(OutResponse::Accepted)
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        trace!("Received control in bRequest = {}, wIndex = {}", req.request, req.index);
        let should_handle = req.request_type == RequestType::Vendor && matches!(req.request, 179..=183 | 240);
        if !should_handle {
            return None;
        }

        match req.request {
            // Read register
            179 => {
                self.shared.sensor.borrow_mut().read_register_slice_blocking(req.index as u8, buf);
                Some(InResponse::Accepted(buf))
            }

            // Don't really know what the rest of these do
            240 => Some(InResponse::Accepted(&[3, 0])),
            _ if req.length == 1 => {
                buf[0] = req.request;
                if req.request == 183 {
                    trace!("running set to false");
                    self.shared.running.set(false);
                }
                Some(InResponse::Accepted(&buf[..1]))
            }
            _ => Some(InResponse::Rejected),
        }
    }
}
