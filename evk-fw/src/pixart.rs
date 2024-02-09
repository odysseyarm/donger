use defmt::info;
use embassy_nrf::spim;
use embassy_usb::{
    control::{InResponse, OutResponse, Request, RequestType},
    driver::Driver,
    Builder,
};

use crate::paj7025::Paj7025;

pub struct State {
    handler: ControlHandler,
}

pub struct PixartClass<'d, D: Driver<'d>> {
    bulk_ep: D::EndpointIn,
}

impl<'d, D: Driver<'d>> PixartClass<'d, D> {
    pub fn new(builder: &mut Builder<'d, D>, state: &'d mut State) -> Self {
        builder.handler(&mut state.handler);
        let mut function = builder.function(255, 0, 0);
        let mut interface = function.interface();
        let mut setting = interface.alt_setting(255, 0, 0, None);
        let _ep1 = setting.endpoint_interrupt_in(64, 4);
        let bulk_ep = setting.endpoint_bulk_in(64);

        Self { bulk_ep }
    }
}

impl State {
    pub fn new() -> Self {
        Self {
            handler: ControlHandler
        }
    }
}

struct ControlHandler;

impl embassy_usb::Handler for ControlHandler {
    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        info!("Received control out bRequest = {}, wIndex = {}, data = {}", req.request, req.index, data);
        let should_handle = req.request_type == RequestType::Vendor && matches!(req.request, 178 | 187);
        if !should_handle {
            return None;
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
            Some(InResponse::Rejected)
        } else if req.length == 1 {
            buf[0] = req.request;
            info!("respond with {:x}", req.request);
            Some(InResponse::Accepted(&buf[..1]))
        } else {
            Some(InResponse::Rejected)
        }
    }
}
