use cortex_m::interrupt;
use embedded_hal_nb::nb;
use rp2040_hal::pwm::{FreeRunning, Slice, SliceId};
use rp2040_monotonic::fugit;

pub struct PwmTimer<const TIMER_HZ: u32, T: SliceId> {
    pwm: Slice<T, FreeRunning>,
    end_time: Option<fugit::TimerInstantU32<TIMER_HZ>>,
}

impl<const TIMER_HZ: u32, T: SliceId> PwmTimer<TIMER_HZ, T> {
    pub fn new(mut pwm: Slice<T, FreeRunning>, periph_timer_freq: u32) -> Self {
        pwm.enable();
        pwm.clr_ph_correct();

        let mut div_int: u32 = periph_timer_freq / TIMER_HZ;
        if div_int > 255 {
            div_int = 255;
        }
        pwm.set_div_int(div_int as u8);

        let frac_mod = periph_timer_freq % TIMER_HZ;
        if frac_mod > 0 {
            let frac_mul = TIMER_HZ / frac_mod;
            let div_frac = 0xFFFF / frac_mul;
            pwm.set_div_frac(div_frac as u8);
        } else {
            pwm.set_div_frac(0);
        }

        Self {
            pwm: pwm,
            end_time: None,
        }
    }

    pub fn ticks(&mut self) -> u64 {
        static mut SYSTICK_OVERFLOWS: u32 = 0;
        static mut OLD_SYSTICK: u16 = 0;

        interrupt::free(|_| {
            // Safety: These static mut variables are accessed in an interrupt free section.
            let (overflows, last_cnt) = unsafe { (&mut SYSTICK_OVERFLOWS, &mut OLD_SYSTICK) };

            let cyccnt = self.pwm.get_counter();

            if cyccnt <= *last_cnt {
                *overflows += 1;
            }

            let ticks = (*overflows as u64) << 16 | (cyccnt as u64);
            *last_cnt = cyccnt;

            ticks
        })
    }
}

impl<const TIMER_HZ: u32, T: SliceId> fugit_timer::Timer<TIMER_HZ> for PwmTimer<TIMER_HZ, T> {
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
