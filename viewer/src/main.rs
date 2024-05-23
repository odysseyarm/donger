use std::{
    fs::File, io::BufWriter, path::Path, sync::{atomic::{AtomicBool, AtomicU16, Ordering}, mpsc::{Receiver, SyncSender}, Arc, Mutex}, time::{Duration, Instant}
};

use ggez::{
    conf::{WindowMode, WindowSetup}, event, glam::*, graphics::{self, Color, DrawParam, ImageFormat, Sampler, Text, Transform}, input::{keyboard::{KeyCode, KeyMods}, mouse::{cursor_grabbed, set_cursor_grabbed, set_position}}, Context, GameResult
};
use image::{codecs::png::PngEncoder, ColorType, ImageEncoder};
use mot_data::MotData;
use serialport::{ClearBuffer, SerialPort};

mod mot_data;
mod charuco;
mod chessboard;
mod circles;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Port {
    Wf,
    Nf,
}

struct PajData {
    image: [u8; 98 * 98 * 4],
    gray: [u8; 98 * 98],
    mot_data: [MotData; 16],
    avg_frame_period: f64,
    last_frame: Instant,
}

impl Default for PajData {
    fn default() -> Self {
        Self {
            image: [255; 98*98*4],
            gray: [255; 98*98],
            mot_data: Default::default(),
            avg_frame_period: 1.0,
            last_frame: Instant::now(),
        }
    }
}

struct MainState {
    wf_data: Arc<Mutex<PajData>>,
    nf_data: Arc<Mutex<PajData>>,
    circle: ggez::graphics::Mesh,
    capture_count: usize,
    captured_nf: Vec<[u8; 98 * 98]>,
    captured_wf: Vec<[u8; 98 * 98]>,
    board_size: Arc<(AtomicU16, AtomicU16)>, // (width, height)
    upside_down: bool,
    reset: Arc<AtomicBool>,
    quit: Arc<AtomicBool>,
}

impl MainState {
    fn new(ctx: &mut Context, capture_count: usize) -> GameResult<MainState> {
        let circle = graphics::Mesh::new_circle(
            ctx,
            graphics::DrawMode::fill(),
            vec2(0., 0.),
            4.0,
            2.0,
            Color::RED,
        )?;

        let wf_data = Arc::new(Mutex::new(Default::default()));
        let nf_data = Arc::new(Mutex::new(Default::default()));
        let img_channel = std::sync::mpsc::sync_channel(2);
        let reset = Arc::new(AtomicBool::new(false));
        let quit = Arc::new(AtomicBool::new(false));
        let path = std::env::args().nth(1).unwrap_or("/dev/ttyACM1".into());
        let binding = std::env::args().nth(2).unwrap_or("".into());
        let upside_down = binding.as_str();
        let upside_down = match upside_down {
            "--upside-down" | "-ud" => true,
            _ => false,
        };
        println!("upside_down: {}", upside_down);
        let main_state = MainState {
            wf_data: wf_data.clone(),
            nf_data: nf_data.clone(),
            circle,
            capture_count,
            captured_nf: vec![],
            captured_wf: vec![],
            board_size: Arc::new((4.into(), 4.into())),
            reset: reset.clone(),
            quit: quit.clone(),
            upside_down,
        };
        std::thread::spawn(move || {
            reader_thread(path, wf_data, nf_data, img_channel.0, reset, quit);
        });
        let board_size = main_state.board_size.clone();
        std::thread::spawn(move || opencv_thread(img_channel.1, board_size, upside_down));
        Ok(main_state)
    }
}

impl event::EventHandler<ggez::GameError> for MainState {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        // self.pos_x = self.pos_x % 800.0 + 1.0;
        if self.quit.load(Ordering::Relaxed) {
            ctx.request_quit();
        }
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
        canvas.draw(&Text::new(format!("{:.1} fps", 1. / nf_data.avg_frame_period)), DrawParam {
            color: Color::WHITE,
            transform: Transform::Matrix(
                [
                    [1., 0., 0., 0.],
                    [0., 1., 0., 0.],
                    [0., 0., 1., 0.],
                    [700., 7. * 98., 0., 1.],
                ]
                .into(),
            ),
            ..Default::default()
        });
        canvas.draw(&Text::new(format!("{:.1} fps", 1. / nf_data.avg_frame_period)), DrawParam {
            color: Color::WHITE,
            transform: Transform::Matrix(
                [
                    [1., 0., 0., 0.],
                    [0., 1., 0., 0.],
                    [0., 0., 1., 0.],
                    [0., 7. * 98., 0., 1.],
                ]
                .into(),
            ),
            ..Default::default()
        });
        let cols = self.board_size.0.load(Ordering::Relaxed);
        let rows = self.board_size.1.load(Ordering::Relaxed);
        canvas.draw(&Text::new(format!("Captures: {}    Board: ({}, {})", self.captured_wf.len(), cols, rows)), DrawParam {
            color: Color::WHITE,
            transform: Transform::Matrix(
                [
                    [1., 0., 0., 0.],
                    [0., 1., 0., 0.],
                    [0., 0., 1., 0.],
                    [0., 7. * 98. + 18., 0., 1.],
                ]
                .into(),
            ),
            ..Default::default()
        });
        canvas.draw(&Text::new("<1> calib nf    <2> calib wf    <3> stereo calib    <Space> capture    <Backspace> clear    <Esc> close    <R> rows    <C> cols"), DrawParam {
            color: Color::WHITE,
            transform: Transform::Matrix(
                [
                    [1., 0., 0., 0.],
                    [0., 1., 0., 0.],
                    [0., 0., 1., 0.],
                    [0., 7. * 98. + 36., 0., 1.],
                ]
                .into(),
            ),
            ..Default::default()
        });

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

    fn key_down_event(
        &mut self,
        ctx: &mut Context,
        input: ggez::input::keyboard::KeyInput,
        repeated: bool,
    ) -> Result<(), ggez::GameError> {
        if repeated {
            return Ok(());
        }
        match input.keycode {
            Some(KeyCode::Escape) => ctx.request_quit(),
            Some(KeyCode::Space) => {
                // capture and save image
                let wf_data = self.wf_data.lock().unwrap();
                PngEncoder::new(BufWriter::new(File::create(format!("images/widefield_{:02}.png", self.capture_count)).unwrap())).write_image(&wf_data.gray, 98, 98, ColorType::L8).unwrap();
                self.captured_wf.push(wf_data.gray);
                drop(wf_data);
                println!("Saved images/widefield_{:02}.png", self.capture_count);
                let nf_data = self.nf_data.lock().unwrap();
                PngEncoder::new(BufWriter::new(File::create(format!("images/nearfield_{:02}.png", self.capture_count)).unwrap())).write_image(&nf_data.gray, 98, 98, ColorType::L8).unwrap();
                self.captured_nf.push(nf_data.gray);
                println!("Saved images/nearfield_{:02}.png", self.capture_count);
                self.capture_count += 1;
            }
            Some(KeyCode::Back) => {
                self.captured_nf.clear();
                self.captured_wf.clear();
            }
            Some(KeyCode::Key1) => {
                // calibrate nf
                println!("Calibrating nearfield from {} captures", self.captured_nf.len());
                let captures = self.captured_nf.clone();
                let board_cols = self.board_size.0.load(Ordering::Relaxed);
                let board_rows = self.board_size.1.load(Ordering::Relaxed);
                std::thread::spawn({
                    let upside_down = self.upside_down;
                    move || {
                        chessboard::calibrate_single(&captures, Port::Nf, board_rows, board_cols, upside_down);
                    }
                });
            }
            Some(KeyCode::Key2) => {
                // calibrate wf
                println!("Calibrating widefield from {} captures", self.captured_wf.len());
                let captures = self.captured_wf.clone();
                let board_cols = self.board_size.0.load(Ordering::Relaxed);
                let board_rows = self.board_size.1.load(Ordering::Relaxed);
                std::thread::spawn({
                    let upside_down = self.upside_down;
                    move || {
                        chessboard::calibrate_single(&captures, Port::Wf, board_rows, board_cols, upside_down);
                    }
                });
            }
            Some(KeyCode::Key3) => {
                println!("Calibrating stereo from {} captures", self.captured_wf.len());
                let wf = self.captured_wf.clone();
                let nf = self.captured_nf.clone();
                let board_cols = self.board_size.0.load(Ordering::Relaxed);
                let board_rows = self.board_size.1.load(Ordering::Relaxed);
                std::thread::spawn({
                    let upside_down = self.upside_down;
                    move || {
                        chessboard::my_stereo_calibrate(&wf, &nf, board_rows, board_cols, upside_down);
                    }
                });
            }
            Some(KeyCode::R) => {
                if input.mods.contains(KeyMods::SHIFT) {
                    self.board_size.1.fetch_sub(1, Ordering::Relaxed);
                } else {
                    self.board_size.1.fetch_add(1, Ordering::Relaxed);
                }
            }
            Some(KeyCode::C) => {
                if input.mods.contains(KeyMods::SHIFT) {
                    self.board_size.0.fetch_sub(1, Ordering::Relaxed);
                } else {
                    self.board_size.0.fetch_add(1, Ordering::Relaxed);
                }
            }
            Some(KeyCode::F1) => {
                self.reset.store(true, Ordering::Relaxed);
            }
            _ => (),
        }
        Ok(())
    }

    // fn mouse_button_down_event(
    //         &mut self,
    //         ctx: &mut Context,
    //         button: event::MouseButton,
    //         _x: f32,
    //         _y: f32,
    //     ) -> Result<(), ggez::GameError> {
    //     match button {
    //         event::MouseButton::Left => set_cursor_grabbed(ctx, true),
    //         _ => Ok(()),
    //     }
    // }
    //
    // fn mouse_button_up_event(
    //         &mut self,
    //         ctx: &mut Context,
    //         button: event::MouseButton,
    //         _x: f32,
    //         _y: f32,
    //     ) -> Result<(), ggez::GameError> {
    //     match button {
    //         event::MouseButton::Left => set_cursor_grabbed(ctx, false),
    //         _ => Ok(()),
    //     }
    // }
    //
    // fn mouse_motion_event(
    //         &mut self,
    //         ctx: &mut Context,
    //         x: f32,
    //         y: f32,
    //         dx: f32,
    //         dy: f32,
    //     ) -> Result<(), ggez::GameError> {
    //     if cursor_grabbed(ctx) {
    //         let res = ctx.gfx.drawable_size();
    //         let scale = ctx.gfx.window().scale_factor() as f32;
    //         set_position(ctx, Vec2::new(700., 350.) / scale)?;
    //     }
    //     Ok(())
    // }
}

pub fn main() -> GameResult {
    std::fs::create_dir_all("images").unwrap();
    let capture_count = {
        let mut i = 0;
        loop {
            let path = format!("images/nearfield_{:02}.png", i);
            if !Path::new(&path).exists() {
                break i;
            }
            i += 1;
        }
    };
    let cb = ggez::ContextBuilder::new("super_simple", "ggez")
        .window_setup(WindowSetup {
            title: "donger viewer".into(),
            ..Default::default()
        })
        .window_mode(WindowMode {
            width: 1400.,
            height: 750.,
            ..Default::default()
        });
    let (mut ctx, event_loop) = cb.build()?;
    let state = MainState::new(&mut ctx, capture_count)?;
    event::run(ctx, event_loop, state)
}

fn reader_thread(
    path: String,
    wf_data: Arc<Mutex<PajData>>,
    nf_data: Arc<Mutex<PajData>>,
    img_channel: SyncSender<(Port, [u8; 98*98])>,
    reset: Arc<AtomicBool>,
    quit: Arc<AtomicBool>,
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
    loop {
        if reset.load(Ordering::Relaxed) {
            port.write(b"r").unwrap();
            port.flush().unwrap();
            quit.store(true, Ordering::Relaxed);
            break;
        } else {
            port.write(b"a").unwrap();
        }
        port.read_exact(&mut buf).unwrap();
        let [.., id, len0, len1] = buf;
        let len = u16::from_le_bytes([len0, len1]);
        if len == 9898 {
            shiftl(&mut buf, 4);
        } else {
            if len < 9897 {
                buf.rotate_right(9897 - len as usize);
                buf[..9897 - len as usize].fill(0);
            }
            for (i, &byte) in [0x2d, 0x5a, 0xb4, 0x69, 0xd2, 0xa5, 0x4b].iter().enumerate() {
                if buf[buf.len()-6] == byte {
                    shiftr(&mut buf, i as u8+1);
                }
            }
        }
        let mut paj_data = if id == 0 {
            let _ = img_channel.try_send((Port::Wf, buf[..98*98].try_into().unwrap()));
            wf_data.lock().unwrap()
        } else {
            let _ = img_channel.try_send((Port::Nf, buf[..98*98].try_into().unwrap()));
            nf_data.lock().unwrap()
        };
        for i in 0..98 * 98 {
            paj_data.image[4 * i] = buf[i];
            paj_data.image[4 * i + 1] = buf[i];
            paj_data.image[4 * i + 2] = buf[i];
        }
        paj_data.gray.copy_from_slice(&buf[..98*98]);
        for (i, mut raw_mot) in buf[98*98..][..256].chunks(16).enumerate() {
            paj_data.mot_data[i] = MotData::parse(&mut raw_mot);
        }
        paj_data.avg_frame_period = paj_data.avg_frame_period * 7. / 8. + paj_data.last_frame.elapsed().as_secs_f64() / 8.;
        paj_data.last_frame = Instant::now();
        drop(paj_data);
    }
}

fn opencv_thread(raw_image: Receiver<(Port, [u8; 98*98])>, board_size: Arc<(AtomicU16, AtomicU16)>, upside_down: bool) {
    loop {
        let (port, image) = raw_image.recv().unwrap();
        let board_cols = board_size.0.load(Ordering::Relaxed);
        let board_rows = board_size.1.load(Ordering::Relaxed);
        match port {
            Port::Wf => chessboard::get_chessboard_corners(&image, Port::Wf, board_rows, board_cols, true, upside_down),
            Port::Nf => chessboard::get_chessboard_corners(&image, Port::Nf, board_rows, board_cols, true, upside_down),
        };
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
