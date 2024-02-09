use core::{cell::{Cell, RefCell}, mem::MaybeUninit};

use defmt::info;
use embassy_nrf::spim;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_usb::{
    control::{InResponse, OutResponse, Request, RequestType},
    driver::Driver,
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

        Self { _bulk_ep: bulk_ep, shared: &state.shared }
    }

    pub async fn run(&self) {
        loop {
            if self.shared.running.get() {
                // TODO
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
        info!("Received control out bRequest = {}, wIndex = {}, data = {}", req.request, req.index, data);
        let should_handle = req.request_type == RequestType::Vendor && matches!(req.request, 178 | 187);
        if !should_handle {
            return None;
        }

        if req.index == 0x5501 {
            // initialization is done beforehand
            if let &[v] = data {
                self.shared.running.set(v != 0);
            }
        } else if let &[v] = data {
            self.shared.sensor.borrow_mut().write_register_blocking(req.index as u8, v);
        }
        Some(OutResponse::Accepted)
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        info!("Received control in bRequest = {}, wIndex = {}", req.request, req.index);
        let should_handle = req.request_type == RequestType::Vendor && matches!(req.request, 179..=183 | 240);
        if !should_handle {
            return None;
        }
        if req.request == 179 {
            // read sensor registers (TODO)
            self.shared.sensor.borrow_mut().read_register_slice_blocking(req.index as u8, buf);
            Some(InResponse::Accepted(buf))
        } else if req.length == 1 {
            buf[0] = req.request;
            if req.request == 183 {
                self.shared.running.set(false);
            }
            Some(InResponse::Accepted(&buf[..1]))
        } else {
            Some(InResponse::Rejected)
        }
    }
}
