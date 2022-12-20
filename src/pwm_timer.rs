use core::sync::atomic::{AtomicU32, Ordering};
use embedded_hal_nb::nb;
use rp2040_hal::pwm::{FreeRunning, Slice, SliceId};
use rp2040_monotonic::fugit;

const TIMER_HZ: u32 = 1000;

pub struct PwmTimerDriver<'a, T: SliceId> {
    pwm: Slice<T, FreeRunning>,
    ticks: &'a AtomicU32,
}

impl<'a, T: SliceId> PwmTimerDriver<'a, T> {
    pub fn new(
        mut pwm: Slice<T, FreeRunning>,
        ticks: &'a AtomicU32,
        periph_timer_freq: u32,
    ) -> Self {
        pwm.enable();
        pwm.clr_ph_correct();

        let div_int: u32 = periph_timer_freq / (TIMER_HZ * TIMER_HZ);
        assert!(div_int < 256);
        pwm.set_div_int(div_int as u8);

        let frac_mod = periph_timer_freq % (TIMER_HZ * TIMER_HZ);
        if frac_mod > 0 {
            let frac_mul = (TIMER_HZ * TIMER_HZ) / frac_mod;
            let div_frac = 0xFFFF / frac_mul;
            pwm.set_div_frac(div_frac as u8);
        } else {
            pwm.set_div_frac(0);
        }

        pwm.set_top(TIMER_HZ as u16);
        pwm.enable_interrupt();

        Self { pwm, ticks }
    }

    pub fn on_wrap(&mut self) {
        if self.pwm.has_overflown() {
            let t = self.ticks.load(Ordering::Relaxed);
            self.ticks.store(t + 1, Ordering::Relaxed);
            self.pwm.clear_interrupt();
        }
    }
}

pub struct PwmTimer<'a> {
    ticks: &'a AtomicU32,
    end_time: Option<fugit::TimerInstantU32<TIMER_HZ>>,
}

impl<'a> PwmTimer<'a> {
    pub fn new(ticks: &'a AtomicU32) -> Self {
        Self {
            ticks: ticks,
            end_time: None,
        }
    }

    pub fn ticks(&mut self) -> u64 {
        self.ticks.load(Ordering::Relaxed) as u64
    }
}

impl<'a> fugit_timer::Timer<TIMER_HZ> for PwmTimer<'a> {
    type Error = core::convert::Infallible;

    fn now(&mut self) -> fugit::TimerInstantU32<TIMER_HZ> {
        fugit::TimerInstantU32::from_ticks(self.ticks() as u32)
    }

    fn start(&mut self, duration: fugit::TimerDurationU32<TIMER_HZ>) -> Result<(), Self::Error> {
        let end = self.now() + duration;
        self.end_time.replace(end);
        Ok(())
    }

    fn cancel(&mut self) -> Result<(), Self::Error> {
        self.end_time.take();
        Ok(())
    }

    fn wait(&mut self) -> nb::Result<(), Self::Error> {
        let now = self.now();
        match self.end_time {
            Some(end) if end <= now => Ok(()),
            _ => Err(nb::Error::WouldBlock),
        }
    }
}
