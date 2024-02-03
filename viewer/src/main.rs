use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use ggez::{
    conf::{WindowMode, WindowSetup},
    event,
    glam::*,
    graphics::{self, Color, DrawParam, ImageFormat, Sampler, Transform},
    Context, GameResult,
};
use mot_data::MotData;
use serialport::{ClearBuffer, SerialPort};

mod mot_data;

struct PajData {
    image: [u8; 98 * 98 * 4],
    mot_data: [MotData; 16],
}

impl Default for PajData {
    fn default() -> Self {
        Self {
            image: [255; 98*98*4],
            mot_data: Default::default(),
        }
    }
}

struct MainState {
    wf_data: Arc<Mutex<PajData>>,
    nf_data: Arc<Mutex<PajData>>,
    circle: ggez::graphics::Mesh,
}

impl MainState {
    fn new(ctx: &mut Context) -> GameResult<MainState> {
        let circle = graphics::Mesh::new_circle(
            ctx,
            graphics::DrawMode::fill(),
            vec2(0., 0.),
            10.0,
            2.0,
            Color::RED,
        )?;

        // Ok(MainState { pos_x: 0.0, circle })
        let wf_data = Arc::new(Mutex::new(Default::default()));
        let nf_data = Arc::new(Mutex::new(Default::default()));
        let main_state = MainState {
            wf_data: wf_data.clone(),
            nf_data: nf_data.clone(),
            circle,
        };
        std::thread::spawn(move || {
            reader_thread("/dev/ttyACM1".into(), wf_data, nf_data);
        });
        Ok(main_state)
    }
}

impl event::EventHandler<ggez::GameError> for MainState {
    fn update(&mut self, _ctx: &mut Context) -> GameResult {
        // self.pos_x = self.pos_x % 800.0 + 1.0;
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let mut canvas =
            graphics::Canvas::from_frame(ctx, graphics::Color::from([0.1, 0.2, 0.3, 1.0]));
        canvas.set_sampler(Sampler::nearest_clamp());

        let wf_data = self.wf_data.lock().unwrap();
        let nf_data = self.nf_data.lock().unwrap();
        let wf_image = graphics::Image::from_pixels(
            ctx,
            &wf_data.image[..98 * 98 * 4],
            ImageFormat::Rgba8UnormSrgb,
            98,
            98,
        );
        let nf_image = graphics::Image::from_pixels(
            ctx,
            &nf_data.image[..98 * 98 * 4],
            ImageFormat::Rgba8UnormSrgb,
            98,
            98,
        );

        canvas.draw(
            &wf_image,
            DrawParam {
                transform: Transform::Matrix(
                    [
                        [-7., 0., 0., 0.],
                        [0., 7., 0., 0.],
                        [0., 0., 1., 0.],
                        [7. * 98., 0., 0., 1.],
                    ]
                    .into(),
                ),
                ..Default::default()
            },
        );
        canvas.draw(
            &nf_image,
            DrawParam {
                transform: Transform::Matrix(
                    [
                        [7., 0., 0., 0.],
                        [0., -7., 0., 0.],
                        [0., 0., 1., 0.],
                        [700., 7. * 98., 0., 1.],
                    ]
                    .into(),
                ),
                ..Default::default()
            },
        );

        for mot_data in &wf_data.mot_data {
            if mot_data.area > 0 {
                let x = mot_data.cx as f32 / 2940.;
                let y = mot_data.cy as f32 / 2940.;
                canvas.draw(&self.circle, Vec2::new((1.-x)*7.*98., y*7.*98.));
            }
        }

        for mot_data in &nf_data.mot_data {
            if mot_data.area > 0 {
                let x = mot_data.cx as f32 / 2940.;
                let y = mot_data.cy as f32 / 2940.;
                canvas.draw(&self.circle, Vec2::new(x*7.*98.+700., (1.-y)*7.*98.));
            }
        }

        canvas.finish(ctx)?;

        Ok(())
    }
}

pub fn main() -> GameResult {
    let cb = ggez::ContextBuilder::new("super_simple", "ggez")
        .window_setup(WindowSetup {
            title: "donger viewer".into(),
            ..Default::default()
        })
        .window_mode(WindowMode {
            width: 1400.,
            height: 700.,
            ..Default::default()
        });
    let (mut ctx, event_loop) = cb.build()?;
    let state = MainState::new(&mut ctx)?;
    event::run(ctx, event_loop, state)
}

fn reader_thread(
    path: String,
    wf_data: Arc<Mutex<PajData>>,
    nf_data: Arc<Mutex<PajData>>,
) {
    let mut port = serialport::new(path, 115200)
        .timeout(Duration::from_secs(3))
        .open()
        .unwrap();
    let drained = drain(&mut *port);
    if drained > 0 {
        eprintln!("drained {drained} bytes");
    }

    let mut buf = [0; 9898];
    let mut time = Instant::now();
    let mut avg = 0.0;
    loop {
        port.write(b"a").unwrap();
        port.read_exact(&mut buf).unwrap();
        let [.., id, len0, len1] = buf;
        let len = u16::from_le_bytes([len0, len1]);
        if len == 9898 {
            shiftl(&mut buf, 4);
        } else if len == 9897 {
            shiftr(&mut buf, 3);
        }
        let mut paj_data = if id == 0 {
            wf_data.lock().unwrap()
        } else {
            nf_data.lock().unwrap()
        };
        for i in 0..98 * 98 {
            paj_data.image[4 * i] = buf[i];
            paj_data.image[4 * i + 1] = buf[i];
            paj_data.image[4 * i + 2] = buf[i];
        }
        for (i, mut raw_mot) in buf[98*98..][..256].chunks(16).enumerate() {
            paj_data.mot_data[i] = MotData::parse(&mut raw_mot);
        }
        drop(paj_data);
        avg = avg / 2. + time.elapsed().as_secs_f64() / 2.;
        time = Instant::now();
        println!("{id} {len}, {:.4}/s", 1. / avg);
    }
}

/// Shift the data to the left n bits
fn shiftl(d: &mut [u8], n: u8) {
    d[0] <<= n;
    for i in 1..d.len() {
        d[i - 1] |= d[i] >> (8 - n);
        d[i] <<= n;
    }
}

/// Shift the data to the right n bits
fn shiftr(d: &mut [u8], n: u8) {
    let mut t = 0;
    for i in 0..d.len() {
        let t2 = d[i];
        d[i] >>= n;
        d[i] |= t << (8 - n);
        t = t2;
    }
}
// r = [(d[0] << n) & 0xff]
// for i in range(1, len(d)):
//     r[-1] |= d[i]>>(8-n)
//     r.append((d[i] << n) & 0xff)
// return bytes(r)

fn drain<T: SerialPort + ?Sized>(port: &mut T) -> u32 {
    let mut drained = 0;
    loop {
        let bytes_to_read = port.bytes_to_read().unwrap();
        if bytes_to_read > 0 {
            port.clear(ClearBuffer::Input).unwrap();
            drained += bytes_to_read;
            std::thread::sleep(Duration::from_millis(7));
        } else {
            return drained;
        }
    }
}
