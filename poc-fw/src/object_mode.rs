use core::{
    cell::RefCell,
    mem::{MaybeUninit, transmute},
    sync::atomic::{AtomicBool, AtomicU8, Ordering},
};

use bytemuck::{cast_slice, from_bytes};
use defmt::Format;
use embassy_futures::{
    join::join4,
    select::{Either, select},
};
use embassy_nrf::nvmc::Nvmc;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex as AsyncMutex, watch::Watch};
use embassy_usb::{class::cdc_acm, driver::EndpointError};
use icm426xx::fifo::FifoPacket4;
use nalgebra::Vector3;
use pag7661qn::{
    Interface,
    mode::{self, ModeT},
};
use protodongers::{
    Packet, PacketData as P, PacketType, Props, ReadRegisterResponse,
    slip::{SlipDecodeError, encode_slip_frame},
};
use static_cell::ConstStaticCell;

use crate::{
    CommonContext, Pag, UsbDriver, device_id,
    imu::{Imu, ImuInterrupt},
    settings::{PagSettings, Settings},
    usb::write_serial,
};

type Channel<T, const N: usize> = embassy_sync::channel::Channel<NoopRawMutex, T, N>;
type Sender<'a, T, const N: usize> = embassy_sync::channel::Sender<'a, NoopRawMutex, T, N>;
type Receiver<'a, T, const N: usize> = embassy_sync::channel::Receiver<'a, NoopRawMutex, T, N>;
type ExitReceiver<'a> = embassy_sync::watch::Receiver<'a, NoopRawMutex, (), 4>;

#[allow(unreachable_code)]
pub async fn object_mode(mut ctx: CommonContext) -> CommonContext {
    defmt::info!("Entering Object Mode");
    let pag = AsyncMutex::<NoopRawMutex, _>::new(ctx.pag);
    let pkt_writer = PacketWriter {
        snd: &mut ctx.usb_snd,
        packet_serialize_buffer: ctx.obj.packet_serialize_buffer,
        packet_slip_encode_buffer: ctx.obj.packet_slip_encode_buffer,
    };
    let mut pkt_rcv = PacketReader::new(&mut ctx.usb_rcv, ctx.obj.packet_receive_buffer);
    let pkt_channel = Channel::new();
    let stream_infos = StreamInfos::new();
    let exit = Watch::<_, _, 4>::new();
    let exit0 = exit.receiver().unwrap();
    let _exit1 = exit.receiver().unwrap();
    let _exit2 = exit.receiver().unwrap();
    let _exit3 = exit.receiver().unwrap();

    let a = usb_rcv_loop(
        &pag,
        &stream_infos,
        pkt_channel.sender(),
        &mut pkt_rcv,
        ctx.settings,
        &ctx.nvmc,
    );
    let b = usb_snd_loop(pkt_writer, pkt_channel.receiver(), exit0);
    let c = imu_loop(
        &mut ctx.imu,
        &mut ctx.imu_int,
        pkt_channel.sender(),
        &stream_infos,
    );
    let d = obj_loop(&pag, &mut ctx.pag_int, pkt_channel.sender(), &stream_infos);
    // TODO turn these into tasks
    let r = join4(a, b, c, d).await;

    r.0
    // ctx.pag = pag.into_inner().as_dynamic_mode();
    // ctx
}

async fn usb_rcv_loop(
    pag: &AsyncMutex<NoopRawMutex, Pag<mode::Mode>>,
    stream_infos: &StreamInfos,
    pkt_snd: Sender<'_, Packet, 4>,
    pkt_rcv: &mut PacketReader<'_>,
    settings: &mut Settings,
    nvmc: &RefCell<Nvmc<'static>>,
) -> ! {
    loop {
        let pkt = match pkt_rcv.wait_for_packet().await {
            Err(WaitForPacketError::Usb(EndpointError::Disabled)) => {
                pkt_rcv.rcv.wait_connection().await;
                continue;
            }
            Err(e) => {
                defmt::error!("Failed to read packet: {}", e);
                continue;
            }
            Ok(p) => p,
        };

        let response = match pkt.data {
            P::WriteRegister(register) => {
                let mut pag = pag.lock().await;
                pag.switch_mode_mut(mode::Mode::Idle).await.unwrap();
                pag.set_bank(register.bank).await.unwrap();
                pag.bus_unchecked()
                    .write(register.address, &[register.data])
                    .await
                    .unwrap();
                Some(Packet {
                    id: pkt.id,
                    data: P::Ack(),
                })
            }
            P::ReadRegister(register) => {
                let mut pag = pag.lock().await;
                pag.set_bank(register.bank).await.unwrap();
                let data = pag
                    .bus_unchecked()
                    .read_byte(register.address)
                    .await
                    .unwrap();
                Some(Packet {
                    id: pkt.id,
                    data: P::ReadRegisterResponse(ReadRegisterResponse {
                        bank: register.bank,
                        address: register.address,
                        data,
                    }),
                })
            }
            P::WriteConfig(config) => {
                settings.general.set_general_config(&config.into());
                Some(Packet {
                    id: pkt.id,
                    data: P::Ack(),
                })
            }
            P::ReadConfig() => Some(Packet {
                id: pkt.id,
                data: P::ReadConfigResponse((*settings.general.general_config()).into()),
            }),
            P::FlashSettings() => {
                settings.pag = PagSettings::read_from_pag(&mut *pag.lock().await).await;
                settings.write(nvmc);
                Some(Packet {
                    id: pkt.id,
                    data: P::Ack(),
                })
            }
            P::ReadProps() => Some(Packet {
                id: pkt.id,
                data: P::ReadPropsResponse(Props {
                    uuid: device_id()[..6].try_into().unwrap(),
                    product_id: crate::usb::PID,
                }),
            }),
            P::StreamUpdate(s) => {
                use protodongers::StreamUpdateAction as S;
                match s.action {
                    S::Enable => stream_infos.enable(pkt.id, s.packet_id),
                    S::Disable => stream_infos.disable(s.packet_id),
                    S::DisableAll => stream_infos.disable_all(),
                }
                Some(Packet {
                    id: pkt.id,
                    data: P::Ack(),
                })
            }
            P::WriteMode(mode) => {
                settings.transient.mode = mode;
                settings.transient_write(nvmc);
                Some(Packet {
                    id: pkt.id,
                    data: P::Ack(),
                })
            }
            P::ObjectReportRequest() => None,
            P::Ack() => None,
            P::ReadConfigResponse(_) => None,
            P::ReadPropsResponse(_) => None,
            P::ObjectReport(_) => None,
            P::CombinedMarkersReport(_) => None,
            P::PocMarkersReport(_) => None,
            P::AccelReport(_) => None,
            P::ImpactReport(_) => None,
            P::Vendor(_, _) => None,
            P::ReadRegisterResponse(_) => None,
        };

        if let Some(r) = response {
            if r.id != 255 {
                pkt_snd.send(r).await;
            }
        }
    }
}

async fn usb_snd_loop(
    mut pkt_writer: PacketWriter<'_>,
    pkt_rcv: Receiver<'_, Packet, 4>,
    mut exit: ExitReceiver<'_>,
) -> ! {
    loop {
        let Either::First(pkt) = select(pkt_rcv.receive(), exit.get()).await else {
            break;
        };
        pkt_writer.write_packet(&pkt).await;
    }
    todo!()
}

async fn imu_loop(
    imu: &mut Imu,
    imu_int: &mut ImuInterrupt,
    pkt_snd: Sender<'_, Packet, 4>,
    stream_infos: &StreamInfos,
) {
    static BUFFER: ConstStaticCell<[u32; 521]> = ConstStaticCell::new([0; 521]);
    let buffer = BUFFER.take();
    let mut ts_micros = 0;
    // TODO handle ODR change and impact stream
    loop {
        imu_int.wait().await;
        let _items = imu.read_fifo(buffer).await.unwrap();
        let pkt = from_bytes::<FifoPacket4>(&cast_slice(buffer)[4..24]);
        // CLKIN ticks (32 kHz)
        ts_micros += 1000 * (pkt.timestamp() as u32) / 32;
        // TODO make the driver handle scale
        // 32768 LSB/g
        let ax = pkt.accel_data_x();
        let ay = pkt.accel_data_y();
        let az = pkt.accel_data_z();
        // 262.144 LSB/(º/s)
        let gx = pkt.gyro_data_x();
        let gy = pkt.gyro_data_y();
        let gz = pkt.gyro_data_z();

        let (ax, ay, az) = (-ay, ax, az);
        let (gx, gy, gz) = (-gy, gx, gz);

        if stream_infos.imu.enabled() {
            // defmt::info!("IMU {=usize} fifo items, {}", items, pkt.fifo_header());
            // defmt::info!("ax={} ay={} az={} gx={} gy={} gz={}", ax, ay, az, gx, gy, gz);
            let id = stream_infos.imu.req_id();
            let pkt = Packet {
                id,
                data: P::AccelReport(protodongers::AccelReport {
                    timestamp: ts_micros,
                    accel: Vector3::new(ax, ay, az).cast() / 32768.0 * 9.806650,
                    gyro: Vector3::new(
                        (gx as f32 / 262.144).to_radians(),
                        (gy as f32 / 262.144).to_radians(),
                        (gz as f32 / 262.144).to_radians(),
                    ),
                }),
            };
            let r = pkt_snd.try_send(pkt);
            if r.is_err() {
                defmt::warn!("Failed to send IMU data due to full channel");
            }
        }
    }
}

async fn obj_loop(
    pag: &AsyncMutex<NoopRawMutex, Pag<mode::Mode>>,
    pag_int: &mut crate::PagInt,
    pkt_snd: Sender<'_, Packet, 4>,
    stream_infos: &StreamInfos,
) {
    loop {
        if stream_infos.marker.enabled() {
            let mut objs = [pag7661qn::types::Object::DEFAULT; 16];

            {
                let mut pag = pag.lock().await;
                pag.switch_mode_mut(mode::Object.mode()).await.unwrap();
            }

            // TODO polling necessary rn from confusing bug where both imu and pag interrupts timeout forever eventually if both are enabled in loops
            if pag_int.int_o.is_low() {
                embassy_time::Timer::after_millis(1).await;
                continue;
            }

            // match embassy_time::with_timeout(embassy_time::Duration::from_millis(100), pag_int.wait()).await {
            //     Ok(_) => {},
            //     Err(embassy_time::TimeoutError) => {
            //         defmt::info!("obj timeout");
            //         continue;
            //     }
            // }
            let mut pag = pag.lock().await;
            let Some(n) = pag
                .try_get_objects::<core::convert::Infallible>(&mut objs)
                .await
                .unwrap()
            else {
                continue;
            };

            // x and image are both mirrored to be similar to paj
            for obj in objs[..n as usize].iter_mut() {
                const RANGE: u16 = 319 * u16::pow(2, 6);
                obj.set_x(RANGE - obj.x());
            }

            let id = stream_infos.marker.req_id();
            let pkt = Packet {
                id,
                data: P::PocMarkersReport(protodongers::PocMarkersReport {
                    points: objs.map(|o| nalgebra::Point2::new(o.x(), o.y())),
                }),
            };
            let r = pkt_snd.try_send(pkt);
            if r.is_err() {
                defmt::warn!("Failed to send marker data due to full channel");
            }
        } else {
            embassy_time::Timer::after_millis(50).await;
        }
    }
}

/// Singleton to hold static resources.
pub struct Context {
    packet_receive_buffer: &'static mut [u8; 1024],
    packet_serialize_buffer: &'static mut [u8; 1024],
    packet_slip_encode_buffer: &'static mut [u8; 2049],
}

impl Context {
    /// Can only be called once
    pub fn take() -> Self {
        Self {
            packet_receive_buffer: static_byte_buffer!(1024),
            packet_serialize_buffer: static_byte_buffer!(1024),
            packet_slip_encode_buffer: static_byte_buffer!(2049),
        }
    }
}

struct PacketReader<'a> {
    rcv: &'a mut cdc_acm::Receiver<'static, UsbDriver>,
    buf: &'a mut [u8; 1024],
    n: usize,
}

impl<'a> PacketReader<'a> {
    fn new(rcv: &'a mut cdc_acm::Receiver<'static, UsbDriver>, buf: &'a mut [u8; 1024]) -> Self {
        Self { rcv, buf, n: 0 }
    }

    async fn wait_for_packet(&mut self) -> Result<Packet, WaitForPacketError> {
        // ignore ff lolol
        // gonna replace with slip anyways
        while self.n < 3 {
            self.n += self.rcv.read_packet(&mut self.buf[self.n..]).await?;
        }
        let size = usize::from(u16::from_le_bytes([self.buf[1], self.buf[2]])) * 2;
        while self.n < size + 1 {
            self.n += self.rcv.read_packet(&mut self.buf[self.n..]).await?;
        }

        let pkt = Packet::parse(&mut &self.buf[1..size + 1]);
        self.buf.copy_within(size + 1..self.n, 0);
        self.n -= size + 1;
        Ok(pkt?)
    }
}

struct PacketWriter<'a> {
    snd: &'a mut cdc_acm::Sender<'static, UsbDriver>,
    packet_serialize_buffer: &'a mut [u8; 1024],
    packet_slip_encode_buffer: &'a mut [u8; 2049],
}

impl PacketWriter<'_> {
    async fn write_packet(&mut self, pkt: &Packet) {
        let bytes_written = {
            let ser = unsafe {
                transmute::<&mut [u8], &mut [MaybeUninit<u8>]>(self.packet_serialize_buffer)
            };
            pkt.serialize_raw(ser)
        };
        let pkt_bytes = &self.packet_serialize_buffer[..bytes_written];

        let slip_frame = encode_slip_frame(pkt_bytes, self.packet_slip_encode_buffer).unwrap();
        write_serial(self.snd, slip_frame).await;
    }
}

#[derive(Format)]
enum WaitForPacketError {
    Slip(SlipDecodeError),
    Packet(protodongers::Error),
    Usb(EndpointError),
}

impl_from!(WaitForPacketError::Slip(SlipDecodeError));
impl_from!(WaitForPacketError::Packet(protodongers::Error));
impl_from!(WaitForPacketError::Usb(EndpointError));

#[derive(Default)]
struct StreamInfos {
    imu: StreamInfo,
    impact: StreamInfo,
    marker: StreamInfo,
}

impl StreamInfos {
    fn new() -> Self {
        Self::default()
    }

    fn disable_all(&self) {
        self.imu.enable.store(false, Ordering::Relaxed);
        self.impact.enable.store(false, Ordering::Relaxed);
        self.marker.enable.store(false, Ordering::Relaxed);
    }

    fn enable(&self, req_id: u8, ty: PacketType) {
        defmt::info!("Enable {}, id = {=u8}", ty, req_id);
        let si = match ty {
            PacketType::AccelReport() => &self.imu,
            PacketType::ImpactReport() => &self.impact,
            PacketType::PocMarkersReport() => &self.marker,
            _ => return,
        };
        si.set_req_id(req_id);
        si.set_enable(true);
    }

    fn disable(&self, ty: PacketType) {
        let si = match ty {
            PacketType::AccelReport() => &self.imu,
            PacketType::ImpactReport() => &self.impact,
            PacketType::PocMarkersReport() => &self.marker,
            _ => return,
        };
        si.set_enable(false);
    }
}

#[derive(Default)]
struct StreamInfo {
    req_id: AtomicU8,
    enable: AtomicBool,
}

impl StreamInfo {
    fn enabled(&self) -> bool {
        self.enable.load(Ordering::Relaxed)
    }

    fn req_id(&self) -> u8 {
        self.req_id.load(Ordering::Relaxed)
    }

    fn set_req_id(&self, req_id: u8) {
        self.req_id.store(req_id, Ordering::Relaxed);
    }

    fn set_enable(&self, enable: bool) {
        self.enable.store(enable, Ordering::Relaxed);
    }
}
