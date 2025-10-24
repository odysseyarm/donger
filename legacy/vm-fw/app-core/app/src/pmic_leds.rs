use core::pin::pin;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use npm1300_rs::NPM1300;
use npm1300_rs::leds::LedMode;

static CMD_CH: Channel<ThreadModeRawMutex, LedCmd, 4> = Channel::new();

#[derive(Copy, Clone, Debug)]
pub enum LedIdx {
    Ch0,
    Ch1,
    Ch2,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum LedState {
    // Immediate entry states
    Off,
    TurningOff,
    TurningOn,
    ProgramError,

    // On, battery status, BLE not connected
    BattCharging,
    BattShutoff,
    BattLow,
    BattMed,
    BattHigh,

    // On, battery status, BLE connected
    BleBattCharging,
    BleBattShutoff,
    BleBattLow,
    BleBattMed,
    BleBattHigh,
}

impl LedState {
    pub fn frames(self) -> (u64, u64) {
        match self {
            // Fast blink
            LedState::TurningOff | LedState::TurningOn => (100, 150),

            // Slow blink
            LedState::BattLow
            | LedState::BattMed
            | LedState::BattHigh
            | LedState::BleBattShutoff
            | LedState::BleBattLow
            | LedState::BleBattMed
            | LedState::BleBattHigh => (1000, 4000),

            // "Solid"
            LedState::Off | LedState::ProgramError | LedState::BattShutoff => (250, 0),

            LedState::BattCharging | LedState::BleBattCharging => (1000, 4000),
        }
    }
}

pub const RGB_OFF: (bool, bool, bool) = (false, false, false);
pub const RGB_RED: (bool, bool, bool) = (true, false, false);
pub const RGB_GRN: (bool, bool, bool) = (false, true, false);
pub const RGB_BLU: (bool, bool, bool) = (false, false, true);
pub const RGB_YEL: (bool, bool, bool) = (true, true, false);
pub const RGB_WHT: (bool, bool, bool) = (true, true, true);

#[derive(Clone, Copy, Debug)]
enum LedCmd {
    Set(LedState),
    SetSeamless(LedState),
}

#[derive(Debug)]
pub struct PmicLedsHandle {
    locked: AtomicBool,
}

#[allow(dead_code)]
impl PmicLedsHandle {
    pub fn new() -> Self {
        Self {
            locked: AtomicBool::new(false),
        }
    }
    pub async fn set_state(&self, s: LedState) {
        if self.locked.load(Ordering::Relaxed) {
            return;
        }
        CMD_CH.send(LedCmd::Set(s)).await;
    }
    pub async fn set_state_seamless(&self, s: LedState) {
        CMD_CH.send(LedCmd::SetSeamless(s)).await;
    }
    pub async fn lock_and_set_state(&self, s: LedState) {
        self.locked.store(true, Ordering::Relaxed);
        CMD_CH.send(LedCmd::Set(s)).await;
    }
    pub async fn lock_and_set_state_seamless(&self, s: LedState) {
        self.locked.store(true, Ordering::Relaxed);
        CMD_CH.send(LedCmd::SetSeamless(s)).await;
    }
    pub async fn unlock_state(&self) {
        self.locked.store(false, Ordering::Relaxed);
    }
}

fn frame_durations(state: LedState) -> (Duration, Duration) {
    let (on, off) = state.frames();
    (Duration::from_millis(on), Duration::from_millis(off))
}

fn color_for(state: LedState, frame_idx: u8) -> (bool, bool, bool) {
    // frame_idx: 0 = "on" frame, 1 = "off/alt" frame
    match state {
        // immediate/solid-ish
        LedState::Off => RGB_OFF,
        LedState::ProgramError => RGB_WHT,
        LedState::TurningOff => {
            if frame_idx == 0 {
                RGB_RED
            } else {
                RGB_OFF
            }
        }
        LedState::TurningOn => {
            if frame_idx == 0 {
                RGB_GRN
            } else {
                RGB_OFF
            }
        }

        // battery only
        LedState::BattCharging => RGB_RED, // your code used red here
        LedState::BattShutoff => RGB_RED,
        LedState::BattLow => {
            if frame_idx == 0 {
                RGB_RED
            } else {
                RGB_OFF
            }
        }
        LedState::BattMed => {
            if frame_idx == 0 {
                RGB_YEL
            } else {
                RGB_OFF
            }
        }
        LedState::BattHigh => {
            if frame_idx == 0 {
                RGB_GRN
            } else {
                RGB_OFF
            }
        }

        // BLE + battery (alternate with blue in "off" frame)
        LedState::BleBattCharging => {
            if frame_idx == 0 {
                RGB_BLU
            } else {
                RGB_RED
            }
        }
        LedState::BleBattShutoff => RGB_RED,
        LedState::BleBattLow => {
            if frame_idx == 0 {
                RGB_RED
            } else {
                RGB_BLU
            }
        }
        LedState::BleBattMed => {
            if frame_idx == 0 {
                RGB_YEL
            } else {
                RGB_BLU
            }
        }
        LedState::BleBattHigh => {
            if frame_idx == 0 {
                RGB_GRN
            } else {
                RGB_BLU
            }
        }
    }
}

fn frames_in_state(state: LedState) -> u8 {
    let (_, off) = state.frames();
    if off == 0 { 1 } else { 2 }
}

pub struct PmicLedAnimator<'a, I2c, Del>
where
    I2c: embedded_hal_async::i2c::I2c,
    Del: embedded_hal_async::delay::DelayNs,
{
    pmic: &'a Mutex<NoopRawMutex, NPM1300<I2c, Del>>,
    red: LedIdx,
    green: LedIdx,
    blue: LedIdx,
}

impl<'a, I2c, Del> PmicLedAnimator<'a, I2c, Del>
where
    I2c: embedded_hal_async::i2c::I2c,
    Del: embedded_hal_async::delay::DelayNs,
{
    pub async fn new(
        pmic: &'a Mutex<NoopRawMutex, NPM1300<I2c, Del>>,
        red: LedIdx,
        green: LedIdx,
        blue: LedIdx,
    ) -> Result<(Self, PmicLedsHandle), npm1300_rs::NPM1300Error<I2c::Error>> {
        {
            let mut pmic = pmic.lock().await;
            pmic.configure_led0_mode(LedMode::Host).await?;
            pmic.configure_led1_mode(LedMode::Host).await?;
            pmic.configure_led2_mode(LedMode::Host).await?;
        }
        let mut me = Self { pmic, red, green, blue };
        let _ = me.turn_rgb(false, false, false).await;
        Ok((me, PmicLedsHandle::new()))
    }

    pub async fn run(&mut self) -> ! {
        // Initial state
        let mut state = LedState::Off;
        let mut frame_idx = 0u8;

        // Frame timing bookkeeping
        let (mut on_dur, mut off_dur) = frame_durations(state);
        let mut frame_len = if frame_idx == 0 { on_dur } else { off_dur };
        let mut frame_start = Instant::now();

        // Apply initial LEDs
        self.apply_frame(state, frame_idx).await;

        loop {
            // Remaining time for current frame
            let elapsed = Instant::now() - frame_start;
            let remaining = if frame_len > elapsed {
                frame_len - elapsed
            } else {
                Duration::from_millis(0)
            };

            // Race: next command vs frame timeout
            let wait_timer = Timer::after(remaining);
            let recv_cmd = CMD_CH.receive();

            let mut wait_timer = pin!(wait_timer);
            let mut recv_cmd = pin!(recv_cmd);

            match select(&mut wait_timer, &mut recv_cmd).await {
                // Frame finished: advance to next frame and re-apply
                Either::First(()) => {
                    let nframes = frames_in_state(state);
                    if nframes > 1 {
                        frame_idx = (frame_idx + 1) % nframes;
                    }
                    (on_dur, off_dur) = frame_durations(state);
                    frame_len = if frame_idx == 0 { on_dur } else { off_dur };
                    frame_start = Instant::now();
                    self.apply_frame(state, frame_idx).await;
                }

                // Command received: apply immediately
                Either::Second(cmd) => {
                    match cmd {
                        LedCmd::Set(new) => {
                            // non-seamless: restart at frame 0
                            state = new;
                            frame_idx = 0;
                            (on_dur, _) = frame_durations(state);
                            frame_len = if frames_in_state(state) == 1 { on_dur } else { on_dur };
                            frame_start = Instant::now();
                            self.apply_frame(state, frame_idx).await;
                        }
                        LedCmd::SetSeamless(new) => {
                            // seamless: keep current frame index and remaining time
                            state = new;
                            (on_dur, off_dur) = frame_durations(state);
                            // Keep frame_idx and keep the *same* frame_end instant
                            self.apply_frame(state, frame_idx).await;
                            // Do NOT change frame_start; recompute frame_len so the remaining stays the same
                            frame_len = if frame_idx == 0 { on_dur } else { off_dur };
                            // frame_start unchanged → remaining continues naturally
                        }
                    }
                }
            }
        }
    }

    async fn apply_frame(&mut self, state: LedState, frame_idx: u8) {
        let (r, g, b) = color_for(state, frame_idx);
        let _ = self.turn_rgb(r, g, b).await;
    }

    async fn turn_rgb(&mut self, r: bool, g: bool, b: bool) -> Result<(), npm1300_rs::NPM1300Error<I2c::Error>> {
        self.set_chan(self.red, r).await?;
        self.set_chan(self.green, g).await?;
        self.set_chan(self.blue, b).await?;
        Ok(())
    }

    async fn set_chan(&mut self, ch: LedIdx, on: bool) -> Result<(), npm1300_rs::NPM1300Error<I2c::Error>> {
        let mut pmic = self.pmic.lock().await;
        match (ch, on) {
            (LedIdx::Ch0, true) => pmic.enable_led0().await,
            (LedIdx::Ch0, false) => pmic.disable_led0().await,
            (LedIdx::Ch1, true) => pmic.enable_led1().await,
            (LedIdx::Ch1, false) => pmic.disable_led1().await,
            (LedIdx::Ch2, true) => pmic.enable_led2().await,
            (LedIdx::Ch2, false) => pmic.disable_led2().await,
        }
    }
}
